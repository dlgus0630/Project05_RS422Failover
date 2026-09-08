`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// raw_frame_buffer
//
// 역할
//   duplicate_guard가 전달한 병렬 프레임을 내부 FIFO에 저장한 뒤,
//   uart_tx가 받을 수 있도록 한 바이트씩 순서대로 출력한다.
//
// 출력 바이트 순서
//   SYNC1 -> SYNC2 -> LEN -> DEVICE_ID -> CMD -> SEQ
//   -> PAYLOAD[0] ... PAYLOAD[N-1] -> CRC_H -> CRC_L
//
// LEN 정의
//   LEN = DEVICE_ID(1) + CMD(1) + SEQ(1) + PAYLOAD 길이
//       = 3 ~ 19
//
// UART 쪽 Handshake
//   tx_valid=1인 동안 tx_data가 유효하다.
//   tx_valid && tx_ready인 클럭에서 한 바이트가 전달된다.
//   tx_ready=0이면 tx_data와 현재 Byte 위치를 그대로 유지한다.
//
// Payload 저장 규칙
//   frame_parser와 동일하게 먼저 수신한 바이트가 사용 구간의 상위에 있다.
//   예: Payload DE, AD -> in_payload_data[15:0] = 16'hDEAD
//
// in_seq_gap은 전송 프레임의 Byte가 아닌 내부 Metadata이다.
// 따라서 UART로 직렬화하지 않고 frame_done_seq_gap으로만 전달한다.
//////////////////////////////////////////////////////////////////////////////////

module raw_frame_buffer #(
    parameter [7:0] SYNC1 = 8'hA5,
    parameter [7:0] SYNC2 = 8'h5A,
    parameter integer FRAME_DEPTH = 2
)(
    input  wire         clk,
    input  wire         reset_p,
    input  wire         clear,
    input  wire         statistics_clear,

    // duplicate_guard 출력과 연결되는 Frame 입력
    input  wire         in_valid,
    output wire         in_ready,

    input  wire [7:0]   in_frame_length,
    input  wire [7:0]   in_device_id,
    input  wire [7:0]   in_command,
    input  wire [7:0]   in_sequence,
    input  wire [127:0] in_payload_data,
    input  wire [15:0]  in_received_crc,
    input  wire         in_seq_gap,

    // uart_tx와 연결되는 Byte 출력
    output wire         tx_valid,
    input  wire         tx_ready,
    output reg  [7:0]   tx_data,

    // 상태 및 Event
    output wire         buffer_busy,
    output wire         buffer_full,
    output wire [7:0]   buffered_count,

    output reg          frame_done,
    output reg          frame_done_seq_gap,
    output reg          overflow_pulse,
    output reg  [15:0]  overflow_count,
    output reg          length_error_pulse
);

    localparam integer PTR_WIDTH =
        (FRAME_DEPTH <= 1) ? 1 : $clog2(FRAME_DEPTH);

    localparam integer COUNT_WIDTH =
        (FRAME_DEPTH <= 1) ? 1 : $clog2(FRAME_DEPTH + 1);

    // -------------------------------------------------------------------------
    // Frame FIFO
    // -------------------------------------------------------------------------
    // Frame 하나를 177bit Record로 묶어 저장한다.
    // [176:169] LEN, [168:161] DEVICE_ID, [160:153] CMD
    // [152:145] SEQ, [144:17] PAYLOAD, [16:1] CRC, [0] SEQ_GAP
    reg [176:0] fifo_mem [0:FRAME_DEPTH-1];
    wire [176:0] fifo_read_data;

    reg [PTR_WIDTH-1:0]   write_ptr;
    reg [PTR_WIDTH-1:0]   read_ptr;
    reg [COUNT_WIDTH-1:0] frame_count;

    // -------------------------------------------------------------------------
    // 현재 UART로 출력 중인 Frame
    // -------------------------------------------------------------------------
    reg         active;
    reg [5:0]   byte_index;

    reg [7:0]   current_length;
    reg [7:0]   current_device_id;
    reg [7:0]   current_command;
    reg [7:0]   current_sequence;
    reg [127:0] current_payload;
    reg [15:0]  current_crc;
    reg         current_seq_gap;

    wire length_valid;
    wire fifo_empty;
    wire fifo_full;
    wire load_frame;
    wire fifo_has_space;
    wire push_frame;
    wire overflow_event;
    wire length_error_event;
    reg  blocked_request;

    assign length_valid =
        (in_frame_length >= 8'd3) &&
        (in_frame_length <= 8'd19);

    assign fifo_empty = (frame_count == 0);
    assign fifo_full  = (frame_count == FRAME_DEPTH);
    assign fifo_read_data = fifo_mem[read_ptr];

    // Serializer가 비어 있으면 같은 클럭에 FIFO 한 칸을 비울 수 있다.
    assign load_frame     = !active && !fifo_empty;
    assign fifo_has_space = !fifo_full || load_frame;

    assign in_ready           = fifo_has_space;
    assign push_frame         = in_valid && length_valid && fifo_has_space;
    // ready/valid Backpressure 중에는 Producer가 valid와 데이터를 유지한다.
    // 이 정상 Stall 자체는 Overflow가 아니다. Stall된 요청이 Handshake 없이
    // 철회된 경우에만 실제 유실로 보고 1회 Overflow를 발생시킨다.
    assign overflow_event     = blocked_request && !in_valid;
    assign length_error_event = in_valid && !length_valid;

    assign tx_valid       = active;
    assign buffer_busy    = active || !fifo_empty;
    assign buffer_full    = fifo_full;
    assign buffered_count = frame_count;

    // FIFO 데이터 배열은 Reset/Clear가 필요하지 않다. 쓰기 동작을
    // 비동기 Reset 제어 블록과 분리하여 RAM으로 추론될 수 있게 한다.
    // Reset/Clear가 유효한 클럭에는 기존과 동일하게 쓰기를 수행하지 않는다.
    always @(posedge clk) begin
        if (!reset_p && !clear && push_frame) begin
            fifo_mem[write_ptr] <= {
                in_frame_length,
                in_device_id,
                in_command,
                in_sequence,
                in_payload_data,
                in_received_crc,
                in_seq_gap
            };
        end
    end

    // -------------------------------------------------------------------------
    // 현재 Byte 선택
    // -------------------------------------------------------------------------
    always @(*) begin
        tx_data = 8'd0;

        if (active) begin
            case (byte_index)
                6'd0: tx_data = SYNC1;
                6'd1: tx_data = SYNC2;
                6'd2: tx_data = current_length;
                6'd3: tx_data = current_device_id;
                6'd4: tx_data = current_command;
                6'd5: tx_data = current_sequence;

                default: begin
                    // PAYLOAD 구간: Byte Index 6 ~ current_length+2
                    if ((byte_index >= 6) &&
                        (byte_index < current_length + 8'd3)) begin

                        // 첫 Payload가 사용 구간의 상위 Byte에 있으므로
                        // LEN과 현재 Byte 위치로 필요한 Byte를 선택한다.
                        tx_data = current_payload >>
                                  ((current_length + 8'd2 - byte_index) * 8);
                    end
                    // PAYLOAD 다음에는 CRC High, CRC Low를 보낸다.
                    else if (byte_index == current_length + 8'd3) begin
                        tx_data = current_crc[15:8];
                    end
                    else if (byte_index == current_length + 8'd4) begin
                        tx_data = current_crc[7:0];
                    end
                    else begin
                        tx_data = 8'd0;
                    end
                end
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // FIFO 저장, Frame Load, Byte Handshake
    // -------------------------------------------------------------------------
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            write_ptr            <= {PTR_WIDTH{1'b0}};
            read_ptr             <= {PTR_WIDTH{1'b0}};
            frame_count          <= {COUNT_WIDTH{1'b0}};

            active               <= 1'b0;
            byte_index           <= 6'd0;
            current_length       <= 8'd0;
            current_device_id    <= 8'd0;
            current_command      <= 8'd0;
            current_sequence     <= 8'd0;
            current_payload      <= 128'd0;
            current_crc          <= 16'd0;
            current_seq_gap      <= 1'b0;

            frame_done           <= 1'b0;
            frame_done_seq_gap   <= 1'b0;
            overflow_pulse       <= 1'b0;
            overflow_count       <= 16'd0;
            length_error_pulse   <= 1'b0;
            blocked_request      <= 1'b0;

            // FIFO Data 자체는 Reset할 필요가 없다.
            // frame_count=0이므로 Reset 이전 값은 읽히지 않는다.
        end
        else if (clear) begin
            write_ptr            <= {PTR_WIDTH{1'b0}};
            read_ptr             <= {PTR_WIDTH{1'b0}};
            frame_count          <= {COUNT_WIDTH{1'b0}};

            active               <= 1'b0;
            byte_index           <= 6'd0;
            current_length       <= 8'd0;
            current_device_id    <= 8'd0;
            current_command      <= 8'd0;
            current_sequence     <= 8'd0;
            current_payload      <= 128'd0;
            current_crc          <= 16'd0;
            current_seq_gap      <= 1'b0;

            frame_done           <= 1'b0;
            frame_done_seq_gap   <= 1'b0;
            overflow_pulse       <= 1'b0;
            overflow_count       <= 16'd0;
            length_error_pulse   <= 1'b0;
            blocked_request      <= 1'b0;

            // FIFO Data 자체는 Clear할 필요가 없다.
            // frame_count=0이므로 Clear 이전 값은 읽히지 않는다.
        end
        else begin
            // Event 출력은 발생한 순간에만 1이 되는 1클럭 Pulse이다.
            frame_done         <= 1'b0;
            frame_done_seq_gap <= 1'b0;
            overflow_pulse     <= 1'b0;
            length_error_pulse <= 1'b0;

            if (statistics_clear)
                overflow_count <= 16'd0;

            if (length_error_event)
                length_error_pulse <= 1'b1;

            if (!blocked_request) begin
                if (in_valid && length_valid && !fifo_has_space)
                    blocked_request <= 1'b1;
            end
            else if (!in_valid || push_frame) begin
                blocked_request <= 1'b0;
            end

            if (overflow_event) begin
                overflow_pulse <= 1'b1;

                if (!statistics_clear &&
                    (overflow_count != 16'hFFFF))
                    overflow_count <= overflow_count + 1'b1;
            end

            // 새 Frame의 데이터 저장은 위의 Reset 없는 FIFO 쓰기 블록에서
            // 수행하고, 여기서는 Write Pointer만 갱신한다.
            if (push_frame) begin
                if (write_ptr == FRAME_DEPTH - 1)
                    write_ptr <= {PTR_WIDTH{1'b0}};
                else
                    write_ptr <= write_ptr + 1'b1;
            end

            // Serializer가 비었으면 FIFO의 가장 오래된 Frame을 꺼낸다.
            if (load_frame) begin
                current_length    <= fifo_read_data[176:169];
                current_device_id <= fifo_read_data[168:161];
                current_command   <= fifo_read_data[160:153];
                current_sequence  <= fifo_read_data[152:145];
                current_payload   <= fifo_read_data[144:17];
                current_crc       <= fifo_read_data[16:1];
                current_seq_gap   <= fifo_read_data[0];

                active     <= 1'b1;
                byte_index <= 6'd0;

                if (read_ptr == FRAME_DEPTH - 1)
                    read_ptr <= {PTR_WIDTH{1'b0}};
                else
                    read_ptr <= read_ptr + 1'b1;
            end

            // Push/Load가 동시에 일어나면 FIFO에 저장된 Frame 수는 유지된다.
            case ({push_frame, load_frame})
                2'b10: frame_count <= frame_count + 1'b1;
                2'b01: frame_count <= frame_count - 1'b1;
                default: frame_count <= frame_count;
            endcase

            // UART가 현재 Byte를 수락한 경우에만 다음 Byte로 이동한다.
            if (active && tx_ready) begin
                if (byte_index == current_length + 8'd4) begin
                    active             <= 1'b0;
                    byte_index         <= 6'd0;
                    frame_done         <= 1'b1;
                    frame_done_seq_gap <= current_seq_gap;
                end
                else begin
                    byte_index <= byte_index + 1'b1;
                end
            end
        end
    end

endmodule
