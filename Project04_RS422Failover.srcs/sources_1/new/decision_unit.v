`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// decision_unit
//
// pair_matcher 결과와 현재 Channel 사용 가능 상태를 조합하여 최종 Frame을
// 채택하거나 폐기한다. Pair 내용 불일치에서는 어느 채널이 맞는지 임의로
// 고르지 않고 반드시 폐기한다.
//////////////////////////////////////////////////////////////////////////////////

module decision_unit (
    input  wire         clk,
    input  wire         reset_p,

    input  wire         system_enable,
    input  wire         preferred_channel_b,
    input  wire         channel_a_usable,
    input  wire         channel_b_usable,

    input  wire         result_valid,
    input  wire [1:0]   result_kind,
    input  wire         result_pair_equal,

    input  wire [7:0]   in_frame_length,
    input  wire [7:0]   in_device_id,
    input  wire [7:0]   in_command,
    input  wire [7:0]   in_sequence,
    input  wire [127:0] in_payload_data,
    input  wire [15:0]  in_received_crc,
    input  wire         in_seq_gap,

    output reg          decision_valid,
    output reg          decision_accept,
    output reg          decision_degraded,
    output reg          decision_mismatch_drop,
    output reg          decision_both_invalid,
    output reg          decision_selected_b,

    output reg  [7:0]   out_frame_length,
    output reg  [7:0]   out_device_id,
    output reg  [7:0]   out_command,
    output reg  [7:0]   out_sequence,
    output reg  [127:0] out_payload_data,
    output reg  [15:0]  out_received_crc,
    output reg          out_seq_gap
);

    localparam [1:0] RESULT_PAIR     = 2'b01;
    localparam [1:0] RESULT_SINGLE_A = 2'b10;
    localparam [1:0] RESULT_SINGLE_B = 2'b11;

    reg accept_comb;
    reg degraded_comb;
    reg mismatch_comb;
    reg both_invalid_comb;
    reg selected_b_comb;

    always @(*) begin
        accept_comb       = 1'b0;
        degraded_comb     = 1'b0;
        mismatch_comb     = 1'b0;
        both_invalid_comb = 1'b0;
        selected_b_comb   = 1'b0;

        if (system_enable && result_valid) begin
            case (result_kind)
                RESULT_PAIR: begin
                    if (!result_pair_equal) begin
                        mismatch_comb = 1'b1;
                    end
                    else if (channel_a_usable || channel_b_usable) begin
                        accept_comb   = 1'b1;
                        degraded_comb =
                            !(channel_a_usable && channel_b_usable);

                        if (channel_a_usable && channel_b_usable)
                            selected_b_comb = preferred_channel_b;
                        else
                            selected_b_comb = channel_b_usable;
                    end
                    else begin
                        both_invalid_comb = 1'b1;
                    end
                end

                RESULT_SINGLE_A: begin
                    degraded_comb = 1'b1;

                    if (channel_a_usable) begin
                        accept_comb     = 1'b1;
                        selected_b_comb = 1'b0;
                    end
                    else begin
                        both_invalid_comb = 1'b1;
                    end
                end

                RESULT_SINGLE_B: begin
                    degraded_comb = 1'b1;

                    if (channel_b_usable) begin
                        accept_comb     = 1'b1;
                        selected_b_comb = 1'b1;
                    end
                    else begin
                        both_invalid_comb = 1'b1;
                    end
                end

                default: begin
                    // RESULT_NONE은 출력하지 않는다.
                end
            endcase
        end
    end

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            decision_valid         <= 1'b0;
            decision_accept        <= 1'b0;
            decision_degraded      <= 1'b0;
            decision_mismatch_drop <= 1'b0;
            decision_both_invalid  <= 1'b0;
            decision_selected_b    <= 1'b0;
            out_frame_length       <= 8'd0;
            out_device_id          <= 8'd0;
            out_command            <= 8'd0;
            out_sequence           <= 8'd0;
            out_payload_data       <= 128'd0;
            out_received_crc       <= 16'd0;
            out_seq_gap            <= 1'b0;
        end
        else if (!system_enable) begin
            decision_valid         <= 1'b0;
            decision_accept        <= 1'b0;
            decision_degraded      <= 1'b0;
            decision_mismatch_drop <= 1'b0;
            decision_both_invalid  <= 1'b0;
            decision_selected_b    <= 1'b0;
            out_frame_length       <= 8'd0;
            out_device_id          <= 8'd0;
            out_command            <= 8'd0;
            out_sequence           <= 8'd0;
            out_payload_data       <= 128'd0;
            out_received_crc       <= 16'd0;
            out_seq_gap            <= 1'b0;
        end
        else begin
            decision_valid         <= result_valid;
            decision_accept        <= accept_comb;
            decision_degraded      <= degraded_comb && result_valid;
            decision_mismatch_drop <= mismatch_comb && result_valid;
            decision_both_invalid  <= both_invalid_comb && result_valid;
            decision_selected_b    <= selected_b_comb;

            if (result_valid && accept_comb) begin
                out_frame_length <= in_frame_length;
                out_device_id    <= in_device_id;
                out_command      <= in_command;
                out_sequence     <= in_sequence;
                out_payload_data <= in_payload_data;
                out_received_crc <= in_received_crc;
                out_seq_gap      <= in_seq_gap;
            end
            else if (result_valid) begin
                out_frame_length <= 8'd0;
                out_device_id    <= 8'd0;
                out_command      <= 8'd0;
                out_sequence     <= 8'd0;
                out_payload_data <= 128'd0;
                out_received_crc <= 16'd0;
                out_seq_gap      <= 1'b0;
            end
        end
    end

endmodule
