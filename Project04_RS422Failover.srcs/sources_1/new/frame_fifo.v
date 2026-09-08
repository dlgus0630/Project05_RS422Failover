`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// frame_fifo
//
// 이 모듈은 seq_monitor가 허용한 완성 프레임을 도착 순서대로 저장하는
// 깊이 2의 동기식 FIFO이다. 채널 A와 B에 각각 하나씩 사용한다.
//
// 연결 기준
//   push       = seq_monitor의 seq_accept
//   in_seq_gap = seq_monitor의 seq_gap
//   나머지 in_* = frame_parser가 저장한 현재 프레임 필드
//
// 한 Entry에 저장하는 값
//   LEN, DEVICE_ID, CMD, SEQ, PAYLOAD, 수신 CRC, Sequence Gap 여부
//
// 동작 규칙
//   1. push=1이고 저장 공간이 있으면 현재 입력 프레임을 Tail에 저장한다.
//   2. pop=1이고 FIFO가 비어 있지 않으면 현재 Head를 제거한다.
//   3. push와 pop이 동시에 유효하면 프레임 수는 유지된다.
//   4. Full 상태에서도 같은 클럭에 pop이 유효하면 새 프레임을 저장한다.
//   5. Full 상태에서 pop 없이 push가 들어오면 새 프레임을 폐기하고
//      overflow_pulse를 1클럭 출력한다.
//   6. empty=0일 때 out_*이 현재 가장 오래된 Head 프레임이다.
//
// 주의
//   crc_ok인 프레임만 seq_monitor를 통과하므로 CRC 정상 여부는 따로
//   저장하지 않는다. Gap 프레임은 후보로 허용되므로 in_seq_gap을
//   메타데이터로 함께 저장한다.
//////////////////////////////////////////////////////////////////////////////////

module frame_fifo (
    input  wire         clk,
    input  wire         reset_p,
    input  wire         clear,

    // seq_monitor와 frame_parser에서 들어오는 Push 입력
    input  wire         push,
    input  wire [7:0]   in_frame_length,
    input  wire [7:0]   in_device_id,
    input  wire [7:0]   in_command,
    input  wire [7:0]   in_sequence,
    input  wire [127:0] in_payload_data,
    input  wire [15:0]  in_received_crc,
    input  wire         in_seq_gap,

    // 다음 단계인 pair_matcher가 Head 프레임을 사용한 뒤 발생시키는 신호
    input  wire         pop,

    // 현재 Head 프레임
    output wire [7:0]   out_frame_length,
    output wire [7:0]   out_device_id,
    output wire [7:0]   out_command,
    output wire [7:0]   out_sequence,
    output wire [127:0] out_payload_data,
    output wire [15:0]  out_received_crc,
    output wire         out_seq_gap,

    // FIFO 상태
    output wire         empty,
    output wire         full,
    output reg  [1:0]   count,
    output reg          overflow_pulse
);

    // 깊이가 2이므로 Read/Write Pointer는 각각 1비트면 충분하다.
    reg write_ptr;
    reg read_ptr;

    // 두 Entry의 프레임 필드를 종류별로 저장한다.
    reg [7:0]   frame_length_mem [0:1];
    reg [7:0]   device_id_mem    [0:1];
    reg [7:0]   command_mem      [0:1];
    reg [7:0]   sequence_mem     [0:1];
    reg [127:0] payload_data_mem [0:1];
    reg [15:0]  received_crc_mem [0:1];
    reg         seq_gap_mem      [0:1];

    assign empty = (count == 2'd0);
    assign full  = (count == 2'd2);

    // 실제로 수행할 Pop과 Push를 현재 상태에서 결정한다.
    wire pop_allowed;
    wire push_allowed;

    assign pop_allowed  = pop && !empty;
    assign push_allowed = push && (!full || pop_allowed);

    // FIFO가 비어 있으면 오래된 메모리 값 대신 0을 출력한다.
    // empty=0일 때만 out_*을 유효한 Head 프레임으로 사용한다.
    assign out_frame_length = empty ? 8'd0   : frame_length_mem[read_ptr];
    assign out_device_id    = empty ? 8'd0   : device_id_mem[read_ptr];
    assign out_command      = empty ? 8'd0   : command_mem[read_ptr];
    assign out_sequence     = empty ? 8'd0   : sequence_mem[read_ptr];
    assign out_payload_data = empty ? 128'd0 : payload_data_mem[read_ptr];
    assign out_received_crc = empty ? 16'd0  : received_crc_mem[read_ptr];
    assign out_seq_gap      = empty ? 1'b0   : seq_gap_mem[read_ptr];

    // FIFO 데이터 배열은 Reset/Clear하지 않아도 된다. count=0이면 기존
    // 저장값은 출력되지 않는다. 데이터 쓰기를 Reset 제어 블록과 분리하면
    // Channel Timeout 비교 경로가 모든 Payload 비트의 Clear Mux를 거치지 않는다.
    always @(posedge clk) begin
        if (!reset_p && !clear && push_allowed) begin
            frame_length_mem[write_ptr] <= in_frame_length;
            device_id_mem[write_ptr]    <= in_device_id;
            command_mem[write_ptr]      <= in_command;
            sequence_mem[write_ptr]     <= in_sequence;
            payload_data_mem[write_ptr] <= in_payload_data;
            received_crc_mem[write_ptr] <= in_received_crc;
            seq_gap_mem[write_ptr]      <= in_seq_gap;
        end
    end

    // -------------------------------------------------------------------------
    // 프레임 저장, Head 제거, Pointer와 Count 관리
    // -------------------------------------------------------------------------
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            write_ptr      <= 1'b0;
            read_ptr       <= 1'b0;
            count          <= 2'd0;
            overflow_pulse <= 1'b0;
        end
        else if (clear) begin
            write_ptr      <= 1'b0;
            read_ptr       <= 1'b0;
            count          <= 2'd0;
            overflow_pulse <= 1'b0;
        end
        else begin
            // Overflow는 저장하지 못한 순간만 알리는 1클럭 펄스이다.
            overflow_pulse <= push && full && !pop_allowed;

            // Frame 데이터 저장은 위의 Reset 없는 쓰기 블록에서 수행한다.
            if (push_allowed) begin
                write_ptr <= ~write_ptr;
            end

            // Pop이 허용되면 다음 Entry가 새로운 Head가 된다.
            if (pop_allowed)
                read_ptr <= ~read_ptr;

            // Push와 Pop이 동시에 수행되면 Entry 수는 변하지 않는다.
            case ({push_allowed, pop_allowed})
                2'b10: count <= count + 1'b1;
                2'b01: count <= count - 1'b1;
                default: count <= count;
            endcase
        end
    end

endmodule
