`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// pair_matcher
//
// A/B frame_fifo의 Head를 비교한다.
//   - 같은 SEQ: 두 Frame을 Pair로 판정하고 내용 일치 여부를 출력
//   - 다른 SEQ: 더 오래된 쪽을 Single로 먼저 출력
//   - 한쪽만 존재: pair_timeout_cycles 동안 기다린 뒤 Single 출력
//   - SEQ 차이가 정확히 128: 앞/뒤를 구분할 수 없어 둘 다 폐기 후보로 출력
//
// result_kind
//   2'b00 NONE, 2'b01 PAIR, 2'b10 SINGLE_A, 2'b11 SINGLE_B
//
// mismatch_flags
//   [5] LEN, [4] DEVICE_ID, [3] CMD, [2] PAYLOAD, [1] CRC, [0] SEQ
//////////////////////////////////////////////////////////////////////////////////

module pair_matcher #(
    parameter integer PAIR_TIMEOUT_CYCLES = 1_000_000
)(
    input  wire         clk,
    input  wire         reset_p,
    input  wire         clear,
    input  wire [31:0]  pair_timeout_cycles,

    input  wire         a_empty,
    input  wire [7:0]   a_frame_length,
    input  wire [7:0]   a_device_id,
    input  wire [7:0]   a_command,
    input  wire [7:0]   a_sequence,
    input  wire [127:0] a_payload_data,
    input  wire [15:0]  a_received_crc,
    input  wire         a_seq_gap,
    output reg          pop_a,

    input  wire         b_empty,
    input  wire [7:0]   b_frame_length,
    input  wire [7:0]   b_device_id,
    input  wire [7:0]   b_command,
    input  wire [7:0]   b_sequence,
    input  wire [127:0] b_payload_data,
    input  wire [15:0]  b_received_crc,
    input  wire         b_seq_gap,
    output reg          pop_b,

    output reg          result_valid,
    output reg  [1:0]   result_kind,
    output reg          result_pair_equal,
    output reg  [5:0]   mismatch_flags,
    output reg          result_timeout,
    output reg          result_seq_skew,
    output reg          result_seq_ambiguous,
    output reg          result_a_seq_gap,
    output reg          result_b_seq_gap,
    output reg          pair_wait_active,

    output reg  [7:0]   out_frame_length,
    output reg  [7:0]   out_device_id,
    output reg  [7:0]   out_command,
    output reg  [7:0]   out_sequence,
    output reg  [127:0] out_payload_data,
    output reg  [15:0]  out_received_crc,
    output reg          out_seq_gap
);

    localparam [1:0] RESULT_NONE     = 2'b00;
    localparam [1:0] RESULT_PAIR     = 2'b01;
    localparam [1:0] RESULT_SINGLE_A = 2'b10;
    localparam [1:0] RESULT_SINGLE_B = 2'b11;

    reg [31:0] wait_count;
    reg        consume_holdoff;

    wire [31:0] effective_timeout;
    wire [7:0]  b_minus_a;
    wire [5:0]  pair_mismatch;

    assign effective_timeout =
        (pair_timeout_cycles == 32'd0) ?
        PAIR_TIMEOUT_CYCLES : pair_timeout_cycles;

    assign b_minus_a = b_sequence - a_sequence;

    assign pair_mismatch[5] = (a_frame_length != b_frame_length);
    assign pair_mismatch[4] = (a_device_id    != b_device_id);
    assign pair_mismatch[3] = (a_command      != b_command);
    assign pair_mismatch[2] = (a_payload_data != b_payload_data);
    assign pair_mismatch[1] = (a_received_crc != b_received_crc);
    assign pair_mismatch[0] = (a_sequence     != b_sequence);

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            wait_count            <= 32'd0;
            consume_holdoff       <= 1'b0;
            pop_a                 <= 1'b0;
            pop_b                 <= 1'b0;
            result_valid          <= 1'b0;
            result_kind           <= RESULT_NONE;
            result_pair_equal     <= 1'b0;
            mismatch_flags        <= 6'd0;
            result_timeout        <= 1'b0;
            result_seq_skew       <= 1'b0;
            result_seq_ambiguous  <= 1'b0;
            result_a_seq_gap      <= 1'b0;
            result_b_seq_gap      <= 1'b0;
            pair_wait_active      <= 1'b0;
            out_frame_length      <= 8'd0;
            out_device_id         <= 8'd0;
            out_command           <= 8'd0;
            out_sequence          <= 8'd0;
            out_payload_data      <= 128'd0;
            out_received_crc      <= 16'd0;
            out_seq_gap           <= 1'b0;
        end
        else if (clear) begin
            wait_count            <= 32'd0;
            consume_holdoff       <= 1'b0;
            pop_a                 <= 1'b0;
            pop_b                 <= 1'b0;
            result_valid          <= 1'b0;
            result_kind           <= RESULT_NONE;
            result_pair_equal     <= 1'b0;
            mismatch_flags        <= 6'd0;
            result_timeout        <= 1'b0;
            result_seq_skew       <= 1'b0;
            result_seq_ambiguous  <= 1'b0;
            result_a_seq_gap      <= 1'b0;
            result_b_seq_gap      <= 1'b0;
            pair_wait_active      <= 1'b0;
            out_frame_length      <= 8'd0;
            out_device_id         <= 8'd0;
            out_command           <= 8'd0;
            out_sequence          <= 8'd0;
            out_payload_data      <= 128'd0;
            out_received_crc      <= 16'd0;
            out_seq_gap           <= 1'b0;
        end
        else begin
            pop_a                <= 1'b0;
            pop_b                <= 1'b0;
            result_valid         <= 1'b0;
            result_kind          <= RESULT_NONE;
            result_pair_equal    <= 1'b0;
            mismatch_flags       <= 6'd0;
            result_timeout       <= 1'b0;
            result_seq_skew      <= 1'b0;
            result_seq_ambiguous <= 1'b0;
            result_a_seq_gap     <= 1'b0;
            result_b_seq_gap     <= 1'b0;

            if (consume_holdoff) begin
                // pop_a/pop_b는 FIFO가 다음 상승 에지에서 처리한다.
                // 그 사이 같은 Head를 다시 판정하지 않도록 1클럭 대기한다.
                consume_holdoff  <= 1'b0;
                wait_count       <= 32'd0;
                pair_wait_active <= 1'b0;
            end
            else if (!a_empty && !b_empty) begin
                wait_count       <= 32'd0;
                pair_wait_active <= 1'b0;

                if (a_sequence == b_sequence) begin
                    pop_a             <= 1'b1;
                    pop_b             <= 1'b1;
                    result_valid      <= 1'b1;
                    consume_holdoff   <= 1'b1;
                    result_kind       <= RESULT_PAIR;
                    mismatch_flags    <= pair_mismatch;
                    result_pair_equal <= (pair_mismatch == 6'd0);
                    result_a_seq_gap  <= a_seq_gap;
                    result_b_seq_gap  <= b_seq_gap;

                    out_frame_length <= a_frame_length;
                    out_device_id    <= a_device_id;
                    out_command      <= a_command;
                    out_sequence     <= a_sequence;
                    out_payload_data <= a_payload_data;
                    out_received_crc <= a_received_crc;
                    out_seq_gap      <= a_seq_gap | b_seq_gap;
                end
                else if (b_minus_a == 8'd128) begin
                    // 정확히 128 차이는 어느 쪽이 과거인지 판정할 수 없다.
                    pop_a                <= 1'b1;
                    pop_b                <= 1'b1;
                    result_valid         <= 1'b1;
                    consume_holdoff      <= 1'b1;
                    result_kind          <= RESULT_PAIR;
                    result_pair_equal    <= 1'b0;
                    mismatch_flags       <= 6'b00_0001;
                    result_seq_ambiguous <= 1'b1;
                    result_a_seq_gap     <= a_seq_gap;
                    result_b_seq_gap     <= b_seq_gap;

                    out_frame_length <= a_frame_length;
                    out_device_id    <= a_device_id;
                    out_command      <= a_command;
                    out_sequence     <= a_sequence;
                    out_payload_data <= a_payload_data;
                    out_received_crc <= a_received_crc;
                    out_seq_gap      <= a_seq_gap;
                end
                else if (b_minus_a < 8'd128) begin
                    // B가 앞선 번호이므로 A가 더 오래된 Frame이다.
                    pop_a            <= 1'b1;
                    result_valid     <= 1'b0;
                    consume_holdoff  <= 1'b1;
                    result_kind      <= RESULT_SINGLE_A;
                    result_seq_skew  <= 1'b1;
                    mismatch_flags   <= 6'b00_0001;
                    result_a_seq_gap <= 1'b0;
                    result_b_seq_gap <= 1'b0;

                    out_frame_length <= a_frame_length;
                    out_device_id    <= a_device_id;
                    out_command      <= a_command;
                    out_sequence     <= a_sequence;
                    out_payload_data <= a_payload_data;
                    out_received_crc <= a_received_crc;
                    out_seq_gap      <= a_seq_gap;
                end
                else begin
                    // A가 앞선 번호이므로 B가 더 오래된 Frame이다.
                    pop_b            <= 1'b1;
                    result_valid     <= 1'b0;
                    consume_holdoff  <= 1'b1;
                    result_kind      <= RESULT_SINGLE_B;
                    result_seq_skew  <= 1'b1;
                    mismatch_flags   <= 6'b00_0001;
                    result_a_seq_gap <= 1'b0;
                    result_b_seq_gap <= 1'b0;

                    out_frame_length <= b_frame_length;
                    out_device_id    <= b_device_id;
                    out_command      <= b_command;
                    out_sequence     <= b_sequence;
                    out_payload_data <= b_payload_data;
                    out_received_crc <= b_received_crc;
                    out_seq_gap      <= b_seq_gap;
                end
            end
            else if (!a_empty || !b_empty) begin
                pair_wait_active <= 1'b1;

                if (wait_count >= (effective_timeout - 1'b1)) begin
                    wait_count       <= 32'd0;
                    pair_wait_active <= 1'b0;
                    result_valid     <= 1'b1;
                    result_timeout   <= 1'b1;
                    consume_holdoff  <= 1'b1;

                    if (!a_empty) begin
                        pop_a            <= 1'b1;
                        result_kind      <= RESULT_SINGLE_A;
                        result_a_seq_gap <= a_seq_gap;
                        result_b_seq_gap <= 1'b0;
                        out_frame_length <= a_frame_length;
                        out_device_id    <= a_device_id;
                        out_command      <= a_command;
                        out_sequence     <= a_sequence;
                        out_payload_data <= a_payload_data;
                        out_received_crc <= a_received_crc;
                        out_seq_gap      <= a_seq_gap;
                    end
                    else begin
                        pop_b            <= 1'b1;
                        result_kind      <= RESULT_SINGLE_B;
                        result_a_seq_gap <= 1'b0;
                        result_b_seq_gap <= b_seq_gap;
                        out_frame_length <= b_frame_length;
                        out_device_id    <= b_device_id;
                        out_command      <= b_command;
                        out_sequence     <= b_sequence;
                        out_payload_data <= b_payload_data;
                        out_received_crc <= b_received_crc;
                        out_seq_gap      <= b_seq_gap;
                    end
                end
                else begin
                    wait_count <= wait_count + 1'b1;
                end
            end
            else begin
                wait_count       <= 32'd0;
                pair_wait_active <= 1'b0;
            end
        end
    end

endmodule
