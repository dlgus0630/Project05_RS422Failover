`timescale 1ns / 1ps

// -----------------------------------------------------------------------------
// duplicate_guard
//
// 역할
//   decision_unit에서 채택된 프레임 중 최근에 이미 출력한 프레임과
//   DEVICE_ID와 SEQUENCE가 같은 프레임을 중복으로 판단하여 차단한다.
//
// 사용 예
//   1. A 채널의 SEQ=10 프레임이 Single A로 먼저 출력된다.
//   2. 그 뒤 B 채널의 같은 SEQ=10 프레임이 늦게 도착한다.
//   3. 최근 출력 이력에서 같은 프레임을 찾고 duplicate_drop만 발생시킨다.
//
// 출력 규칙
//   out_valid      : UART로 전달할 새 프레임이 있을 때 1클럭 Pulse
//   duplicate_drop : 중복 프레임을 차단한 순간 1클럭 Pulse
//
// 중요
//   중복 식별 기준은 {DEVICE_ID, SEQUENCE}이다.
//   out_valid=0일 때 out_* 데이터는 이전 값을 유지하며 유효하지 않다.
// -----------------------------------------------------------------------------

module duplicate_guard #(
    parameter integer HISTORY_DEPTH = 4
)(
    input  wire         clk,
    input  wire         reset_p,
    input  wire         clear,

    // decision_unit 출력
    input  wire         decision_valid,
    input  wire         decision_accept,
    input  wire         statistics_clear,

    input  wire [7:0]   in_frame_length,
    input  wire [7:0]   in_device_id,
    input  wire [7:0]   in_command,
    input  wire [7:0]   in_sequence,
    input  wire [127:0] in_payload_data,
    input  wire [15:0]  in_received_crc,
    input  wire         in_seq_gap,
    input  wire         in_selected_b,

    // uart_tx로 전달할 프레임
    output reg          out_valid,
    output reg  [7:0]   out_frame_length,
    output reg  [7:0]   out_device_id,
    output reg  [7:0]   out_command,
    output reg  [7:0]   out_sequence,
    output reg  [127:0] out_payload_data,
    output reg  [15:0]  out_received_crc,
    output reg          out_seq_gap,
    output reg          out_selected_b,

    // 상태/통계
    output reg          duplicate_drop,
    output reg  [15:0]  duplicate_count
);

    reg [HISTORY_DEPTH-1:0] history_valid;
    reg [7:0] history_device_id [0:HISTORY_DEPTH-1];
    reg [7:0] history_sequence  [0:HISTORY_DEPTH-1];

    reg duplicate_hit;

    integer search_index;
    integer shift_index;
    integer reset_index;

    // 현재 입력 프레임이 최근 출력 이력에 있는지 검사한다.
    always @(*) begin
        duplicate_hit = 1'b0;

        for (search_index = 0;
             search_index < HISTORY_DEPTH;
             search_index = search_index + 1) begin

            if (history_valid[search_index] &&
                (history_device_id[search_index] == in_device_id) &&
                (history_sequence[search_index]  == in_sequence)) begin
                duplicate_hit = 1'b1;
            end
        end
    end

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            out_valid          <= 1'b0;
            out_frame_length   <= 8'd0;
            out_device_id      <= 8'd0;
            out_command        <= 8'd0;
            out_sequence       <= 8'd0;
            out_payload_data   <= 128'd0;
            out_received_crc   <= 16'd0;
            out_seq_gap        <= 1'b0;
            out_selected_b     <= 1'b0;

            duplicate_drop     <= 1'b0;
            duplicate_count    <= 16'd0;
            history_valid      <= {HISTORY_DEPTH{1'b0}};

            for (reset_index = 0;
                 reset_index < HISTORY_DEPTH;
                 reset_index = reset_index + 1) begin
                history_device_id[reset_index] <= 8'd0;
                history_sequence[reset_index]  <= 8'd0;
            end
        end
        else if (clear) begin
            out_valid          <= 1'b0;
            out_frame_length   <= 8'd0;
            out_device_id      <= 8'd0;
            out_command        <= 8'd0;
            out_sequence       <= 8'd0;
            out_payload_data   <= 128'd0;
            out_received_crc   <= 16'd0;
            out_seq_gap        <= 1'b0;
            out_selected_b     <= 1'b0;

            duplicate_drop     <= 1'b0;
            duplicate_count    <= 16'd0;
            history_valid      <= {HISTORY_DEPTH{1'b0}};

            for (reset_index = 0;
                 reset_index < HISTORY_DEPTH;
                 reset_index = reset_index + 1) begin
                history_device_id[reset_index] <= 8'd0;
                history_sequence[reset_index]  <= 8'd0;
            end
        end
        else begin
            // 두 출력은 사건이 발생한 클럭에만 1이 되는 Pulse이다.
            out_valid      <= 1'b0;
            duplicate_drop <= 1'b0;

            if (statistics_clear)
                duplicate_count <= 16'd0;

            if (decision_valid && decision_accept) begin
                if (duplicate_hit) begin
                    // 이미 출력한 프레임이므로 UART 쪽으로 전달하지 않는다.
                    duplicate_drop <= 1'b1;

                    // out_valid는 0이지만 Event Log가 어떤 Frame을 차단했는지
                    // 기록할 수 있도록 현재 중복 Frame의 Metadata를 보존한다.
                    out_frame_length <= in_frame_length;
                    out_device_id    <= in_device_id;
                    out_command      <= in_command;
                    out_sequence     <= in_sequence;
                    out_payload_data <= in_payload_data;
                    out_received_crc <= in_received_crc;
                    out_seq_gap      <= in_seq_gap;
                    out_selected_b   <= in_selected_b;

                    // 통계 카운터는 최대값에서 포화시킨다.
                    if (!statistics_clear &&
                        (duplicate_count != 16'hFFFF))
                        duplicate_count <= duplicate_count + 1'b1;
                end
                else begin
                    // 새 프레임은 UART 쪽으로 전달한다.
                    out_valid        <= 1'b1;
                    out_frame_length <= in_frame_length;
                    out_device_id    <= in_device_id;
                    out_command      <= in_command;
                    out_sequence     <= in_sequence;
                    out_payload_data <= in_payload_data;
                    out_received_crc <= in_received_crc;
                    out_seq_gap      <= in_seq_gap;
                    out_selected_b   <= in_selected_b;

                    // 가장 오래된 이력을 제거하고 새 이력을 0번에 넣는다.
                    for (shift_index = HISTORY_DEPTH - 1;
                         shift_index > 0;
                         shift_index = shift_index - 1) begin
                        history_valid[shift_index]
                            <= history_valid[shift_index - 1];
                        history_device_id[shift_index]
                            <= history_device_id[shift_index - 1];
                        history_sequence[shift_index]
                            <= history_sequence[shift_index - 1];
                    end

                    history_valid[0]     <= 1'b1;
                    history_device_id[0] <= in_device_id;
                    history_sequence[0]  <= in_sequence;
                end
            end
        end
    end

endmodule
