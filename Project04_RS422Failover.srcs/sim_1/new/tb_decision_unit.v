`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// tb_decision_unit
//
// 최종 decision_unit 정책을 자동 검사한다.
//   - 일치 Pair: 두 채널이 정상일 때 preferred_channel_b를 따른다.
//   - 선호 채널이 사용 불가이면 정상 채널로 폴백한다.
//   - 불일치 Pair는 채널 상태와 관계없이 폐기한다.
//   - 결과에 해당하는 채널이 사용 불가이면 both_invalid를 알린다.
//   - Single 결과는 채택 여부와 관계없이 degraded로 분류한다.
//   - system_enable=0은 클럭 에지에서 동기식으로 출력을 지운다.
//////////////////////////////////////////////////////////////////////////////////

module tb_decision_unit;

    localparam [1:0] RESULT_PAIR     = 2'b01;
    localparam [1:0] RESULT_SINGLE_A = 2'b10;
    localparam [1:0] RESULT_SINGLE_B = 2'b11;

    reg         clk;
    reg         reset_p;
    reg         system_enable;
    reg         preferred_channel_b;
    reg         channel_a_usable;
    reg         channel_b_usable;
    reg         result_valid;
    reg  [1:0]  result_kind;
    reg         result_pair_equal;

    reg  [7:0]   in_frame_length;
    reg  [7:0]   in_device_id;
    reg  [7:0]   in_command;
    reg  [7:0]   in_sequence;
    reg  [127:0] in_payload_data;
    reg  [15:0]  in_received_crc;
    reg          in_seq_gap;

    wire         decision_valid;
    wire         decision_accept;
    wire         decision_degraded;
    wire         decision_mismatch_drop;
    wire         decision_both_invalid;
    wire         decision_selected_b;

    wire [7:0]   out_frame_length;
    wire [7:0]   out_device_id;
    wire [7:0]   out_command;
    wire [7:0]   out_sequence;
    wire [127:0] out_payload_data;
    wire [15:0]  out_received_crc;
    wire         out_seq_gap;

    integer error_count;
    integer test_number;

    decision_unit dut (
        .clk                    (clk),
        .reset_p                (reset_p),
        .system_enable          (system_enable),
        .preferred_channel_b    (preferred_channel_b),
        .channel_a_usable       (channel_a_usable),
        .channel_b_usable       (channel_b_usable),
        .result_valid           (result_valid),
        .result_kind            (result_kind),
        .result_pair_equal      (result_pair_equal),
        .in_frame_length        (in_frame_length),
        .in_device_id           (in_device_id),
        .in_command             (in_command),
        .in_sequence            (in_sequence),
        .in_payload_data        (in_payload_data),
        .in_received_crc        (in_received_crc),
        .in_seq_gap             (in_seq_gap),
        .decision_valid         (decision_valid),
        .decision_accept        (decision_accept),
        .decision_degraded      (decision_degraded),
        .decision_mismatch_drop (decision_mismatch_drop),
        .decision_both_invalid  (decision_both_invalid),
        .decision_selected_b    (decision_selected_b),
        .out_frame_length       (out_frame_length),
        .out_device_id          (out_device_id),
        .out_command            (out_command),
        .out_sequence           (out_sequence),
        .out_payload_data       (out_payload_data),
        .out_received_crc       (out_received_crc),
        .out_seq_gap            (out_seq_gap)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    task check_event;
        input [1:0] kind;
        input       pair_equal;
        input       a_usable;
        input       b_usable;
        input       prefer_b;
        input [7:0] sequence_value;
        input       expected_accept;
        input       expected_degraded;
        input       expected_mismatch;
        input       expected_both_invalid;
        input       expected_selected_b;
        reg         test_pass;
        begin
            test_number = test_number + 1;
            test_pass   = 1'b1;

            @(negedge clk);
            preferred_channel_b = prefer_b;
            channel_a_usable    = a_usable;
            channel_b_usable    = b_usable;
            result_kind         = kind;
            result_pair_equal   = pair_equal;
            in_frame_length     = 8'd5;
            in_device_id        = 8'h21;
            in_command          = 8'h42;
            in_sequence         = sequence_value;
            in_payload_data     =
                128'h0000000000000000000000000000A55A;
            in_received_crc     = 16'hBEEF;
            in_seq_gap          = sequence_value[0];
            result_valid        = 1'b1;

            // 중간 상승 에지에서 DUT가 결정을 저장한다.
            @(negedge clk);
            result_valid = 1'b0;
            #1;

            if ((decision_valid         !== 1'b1)                 ||
                (decision_accept        !== expected_accept)      ||
                (decision_degraded      !== expected_degraded)    ||
                (decision_mismatch_drop !== expected_mismatch)    ||
                (decision_both_invalid  !== expected_both_invalid)||
                (decision_selected_b    !== expected_selected_b)) begin
                $display(
                    "[FAIL][TEST %0d] flags valid=%b accept=%b degraded=%b mismatch=%b both_invalid=%b selected_b=%b",
                    test_number,
                    decision_valid,
                    decision_accept,
                    decision_degraded,
                    decision_mismatch_drop,
                    decision_both_invalid,
                    decision_selected_b
                );
                test_pass = 1'b0;
            end

            if (expected_accept) begin
                if ((out_frame_length !== 8'd5)       ||
                    (out_device_id    !== 8'h21)      ||
                    (out_command      !== 8'h42)      ||
                    (out_sequence     !== sequence_value) ||
                    (out_payload_data !==
                        128'h0000000000000000000000000000A55A) ||
                    (out_received_crc !== 16'hBEEF)   ||
                    (out_seq_gap      !== sequence_value[0])) begin
                    $display(
                        "[FAIL][TEST %0d] accepted frame field capture",
                        test_number
                    );
                    test_pass = 1'b0;
                end
            end
            else begin
                if ((out_frame_length !== 8'd0)   ||
                    (out_device_id    !== 8'd0)   ||
                    (out_command      !== 8'd0)   ||
                    (out_sequence     !== 8'd0)   ||
                    (out_payload_data !== 128'd0) ||
                    (out_received_crc !== 16'd0)  ||
                    (out_seq_gap      !== 1'b0)) begin
                    $display(
                        "[FAIL][TEST %0d] rejected frame was not cleared",
                        test_number
                    );
                    test_pass = 1'b0;
                end
            end

            // 결과 알림 신호는 모두 한 클럭 펄스여야 한다.
            @(negedge clk);
            #1;
            if ((decision_valid         !== 1'b0) ||
                (decision_accept        !== 1'b0) ||
                (decision_degraded      !== 1'b0) ||
                (decision_mismatch_drop !== 1'b0) ||
                (decision_both_invalid  !== 1'b0) ||
                (decision_selected_b    !== 1'b0)) begin
                $display(
                    "[FAIL][TEST %0d] decision output is not one clock",
                    test_number
                );
                test_pass = 1'b0;
            end

            if (test_pass)
                $display("[PASS][TEST %0d] kind=%b seq=%h",
                         test_number, kind, sequence_value);
            else
                error_count = error_count + 1;
        end
    endtask

    task check_synchronous_disable;
        reg test_pass;
        begin
            test_number = test_number + 1;
            test_pass   = 1'b1;

            // 직전 채택 프레임은 result_valid가 내려간 뒤에도 보존된다.
            if (out_sequence !== 8'h18) begin
                $display("[FAIL][TEST %0d] setup frame is not held",
                         test_number);
                test_pass = 1'b0;
            end

            @(negedge clk);
            system_enable = 1'b0;
            result_valid  = 1'b1;
            in_sequence   = 8'hEE;
            #1;

            // system_enable은 비동기 Reset이 아니므로 다음 posedge 전에는
            // 이미 저장된 프레임이 그대로 남아 있어야 한다.
            if (out_sequence !== 8'h18) begin
                $display("[FAIL][TEST %0d] disable acted asynchronously",
                         test_number);
                test_pass = 1'b0;
            end

            @(negedge clk);
            result_valid = 1'b0;
            #1;

            if ((decision_valid         !== 1'b0)   ||
                (decision_accept        !== 1'b0)   ||
                (decision_degraded      !== 1'b0)   ||
                (decision_mismatch_drop !== 1'b0)   ||
                (decision_both_invalid  !== 1'b0)   ||
                (decision_selected_b    !== 1'b0)   ||
                (out_frame_length       !== 8'd0)   ||
                (out_sequence           !== 8'd0)   ||
                (out_payload_data       !== 128'd0)) begin
                $display("[FAIL][TEST %0d] synchronous disable clear",
                         test_number);
                test_pass = 1'b0;
            end

            system_enable = 1'b1;
            repeat (2) @(negedge clk);

            if (test_pass)
                $display("[PASS][TEST %0d] synchronous system disable",
                         test_number);
            else
                error_count = error_count + 1;
        end
    endtask

    initial begin
        reset_p             = 1'b1;
        system_enable       = 1'b0;
        preferred_channel_b = 1'b0;
        channel_a_usable    = 1'b0;
        channel_b_usable    = 1'b0;
        result_valid        = 1'b0;
        result_kind         = 2'b00;
        result_pair_equal   = 1'b0;
        in_frame_length     = 8'd0;
        in_device_id        = 8'd0;
        in_command          = 8'd0;
        in_sequence         = 8'd0;
        in_payload_data     = 128'd0;
        in_received_crc     = 16'd0;
        in_seq_gap          = 1'b0;
        error_count         = 0;
        test_number         = 0;

        repeat (3) @(negedge clk);
        reset_p       = 1'b0;
        system_enable = 1'b1;
        repeat (2) @(negedge clk);

        // 두 채널이 정상이면 software의 선호 채널을 따른다.
        check_event(RESULT_PAIR, 1'b1, 1'b1, 1'b1, 1'b0, 8'h10,
                    1'b1, 1'b0, 1'b0, 1'b0, 1'b0);
        check_event(RESULT_PAIR, 1'b1, 1'b1, 1'b1, 1'b1, 8'h11,
                    1'b1, 1'b0, 1'b0, 1'b0, 1'b1);

        // 선호 채널이 불량이면 사용 가능한 반대 채널로 폴백한다.
        check_event(RESULT_PAIR, 1'b1, 1'b0, 1'b1, 1'b0, 8'h12,
                    1'b1, 1'b1, 1'b0, 1'b0, 1'b1);
        check_event(RESULT_PAIR, 1'b1, 1'b1, 1'b0, 1'b1, 8'h13,
                    1'b1, 1'b1, 1'b0, 1'b0, 1'b0);

        // 일치하더라도 두 채널 모두 사용 불가이면 폐기한다.
        check_event(RESULT_PAIR, 1'b1, 1'b0, 1'b0, 1'b0, 8'h14,
                    1'b0, 1'b0, 1'b0, 1'b1, 1'b0);

        // 내용 불일치는 어느 채널도 임의로 고르지 않고 폐기한다.
        check_event(RESULT_PAIR, 1'b0, 1'b1, 1'b1, 1'b1, 8'h15,
                    1'b0, 1'b0, 1'b1, 1'b0, 1'b0);

        // Single은 해당 채널만 검사하며 항상 degraded 결과이다.
        check_event(RESULT_SINGLE_A, 1'b0, 1'b1, 1'b1, 1'b1, 8'h16,
                    1'b1, 1'b1, 1'b0, 1'b0, 1'b0);
        check_event(RESULT_SINGLE_A, 1'b0, 1'b0, 1'b1, 1'b0, 8'h17,
                    1'b0, 1'b1, 1'b0, 1'b1, 1'b0);
        check_event(RESULT_SINGLE_B, 1'b0, 1'b1, 1'b1, 1'b0, 8'h18,
                    1'b1, 1'b1, 1'b0, 1'b0, 1'b1);

        // 비활성화는 일반 신호이므로 다음 클럭에서 동기식 Clear가 된다.
        check_synchronous_disable();

        if (error_count == 0)
            $display("DECISION UNIT TB: ALL TESTS PASSED");
        else
            $display("DECISION UNIT TB: FAILED, error_count=%0d",
                     error_count);

        $finish;
    end

    initial begin
        #10000;
        $display("[FAIL] decision_unit simulation timeout");
        $finish;
    end

endmodule
