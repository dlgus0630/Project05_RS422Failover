`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// tb_seq_monitor
//
// self-checking Testbench
//
// 검증 항목
//   1. Reset 후 모든 상태와 출력 초기화
//   2. 최초 프레임 기준 등록
//   3. 정상 증가
//   4. Duplicate 검출 및 폐기
//   5. Gap 검출, 후보 허용, 기준값 재동기화
//   6. Gap 다음 정상 프레임으로 복구
//   7. Old/Late 검출 및 폐기
//   8. 거리 127은 Gap, 거리 128은 Old로 분류
//   9. seq_valid=0이면 입력값이 변해도 상태 유지
//  10. 8'hFE -> 8'hFF -> 8'h00 -> 8'h01 순환 증가
//  11. 판정 결과가 모두 1클럭 펄스인지 확인
//////////////////////////////////////////////////////////////////////////////////

module tb_seq_monitor;

    localparam integer CLK_PERIOD_NS = 10;

    reg        clk;
    reg        reset_p;
    reg        clear;
    reg        seq_valid;
    reg  [7:0] tb_seq_value;

    wire       seq_accept;
    wire       seq_ok;
    wire       seq_duplicate;
    wire       seq_gap;
    wire       seq_old;
    wire [7:0] last_rx_seq;
    wire       seq_initialized;

    integer pass_count;
    integer fail_count;
    integer test_number;

    seq_monitor dut (
        .clk             (clk),
        .reset_p         (reset_p),
        .clear           (clear),
        .seq_valid       (seq_valid),
        .seq_value       (tb_seq_value),
        .seq_accept      (seq_accept),
        .seq_ok          (seq_ok),
        .seq_duplicate   (seq_duplicate),
        .seq_gap         (seq_gap),
        .seq_old         (seq_old),
        .last_rx_seq     (last_rx_seq),
        .seq_initialized (seq_initialized)
    );

    // Basys3의 100MHz와 같은 10ns 주기 클럭
    initial begin
        clk = 1'b0;
        forever #(CLK_PERIOD_NS / 2) clk = ~clk;
    end

    // -------------------------------------------------------------------------
    // SEQ 한 개를 입력하고 판정 결과를 자동 검사하는 Task
    //
    // 입력은 negedge에서 바꿔 다음 posedge 전에 안정시킨다.
    // DUT는 그 posedge에서 판정하고, 다음 negedge에서 결과를 검사한다.
    // Task는 Testbench 반복을 줄이기 위한 문법이며 FPGA에 합성되지 않는다.
    // -------------------------------------------------------------------------
    task send_and_check;
        input [7:0] seq_value;
        input       expected_accept;
        input       expected_ok;
        input       expected_duplicate;
        input       expected_gap;
        input       expected_old;
        input [7:0] expected_last;
        input       expected_initialized;

        reg         test_pass;
        begin
            test_number = test_number + 1;
            test_pass   = 1'b1;

            // seq_valid를 정확히 1클럭 동안 발생시킨다.
            @(negedge clk);
            tb_seq_value = seq_value;
            seq_valid = 1'b1;

            // 중간 posedge에서 DUT가 현재 SEQ를 판정한다.
            @(negedge clk);
            #1;

            if (seq_accept !== expected_accept) begin
                $display("[FAIL][TEST %0d] seq_accept: expected=%b, actual=%b",
                         test_number, expected_accept, seq_accept);
                test_pass = 1'b0;
            end

            if (seq_ok !== expected_ok) begin
                $display("[FAIL][TEST %0d] seq_ok: expected=%b, actual=%b",
                         test_number, expected_ok, seq_ok);
                test_pass = 1'b0;
            end

            if (seq_duplicate !== expected_duplicate) begin
                $display("[FAIL][TEST %0d] seq_duplicate: expected=%b, actual=%b",
                         test_number, expected_duplicate, seq_duplicate);
                test_pass = 1'b0;
            end

            if (seq_gap !== expected_gap) begin
                $display("[FAIL][TEST %0d] seq_gap: expected=%b, actual=%b",
                         test_number, expected_gap, seq_gap);
                test_pass = 1'b0;
            end

            if (seq_old !== expected_old) begin
                $display("[FAIL][TEST %0d] seq_old: expected=%b, actual=%b",
                         test_number, expected_old, seq_old);
                test_pass = 1'b0;
            end

            if (last_rx_seq !== expected_last) begin
                $display("[FAIL][TEST %0d] last_rx_seq: expected=%h, actual=%h",
                         test_number, expected_last, last_rx_seq);
                test_pass = 1'b0;
            end

            if (seq_initialized !== expected_initialized) begin
                $display("[FAIL][TEST %0d] seq_initialized: expected=%b, actual=%b",
                         test_number, expected_initialized, seq_initialized);
                test_pass = 1'b0;
            end

            // 입력 펄스를 내린다.
            seq_valid = 1'b0;

            // 다음 클럭에는 모든 판정 펄스가 다시 0이어야 한다.
            @(negedge clk);
            #1;

            if ((seq_accept    !== 1'b0) ||
                (seq_ok        !== 1'b0) ||
                (seq_duplicate !== 1'b0) ||
                (seq_gap       !== 1'b0) ||
                (seq_old       !== 1'b0)) begin
                $display("[FAIL][TEST %0d] 판정 출력이 1클럭 펄스가 아님",
                         test_number);
                test_pass = 1'b0;
            end

            if (test_pass) begin
                pass_count = pass_count + 1;
                $display("[PASS][TEST %0d] SEQ=%h, last=%h",
                         test_number, seq_value, expected_last);
            end
            else begin
                fail_count = fail_count + 1;
            end
        end
    endtask

    // seq_valid=0일 때 입력 변화가 내부 상태에 영향을 주지 않는지 검사한다.
    task check_idle_hold;
        input [7:0] changed_seq_value;
        input [7:0] expected_last;

        reg         test_pass;
        begin
            test_number = test_number + 1;
            test_pass   = 1'b1;

            @(negedge clk);
            tb_seq_value = changed_seq_value;
            seq_valid = 1'b0;

            @(negedge clk);
            #1;

            if ((last_rx_seq     !== expected_last) ||
                (seq_initialized !== 1'b1)          ||
                (seq_accept      !== 1'b0)          ||
                (seq_ok          !== 1'b0)          ||
                (seq_duplicate   !== 1'b0)          ||
                (seq_gap         !== 1'b0)          ||
                (seq_old         !== 1'b0)) begin
                $display("[FAIL][TEST %0d] seq_valid=0 상태 유지 실패",
                         test_number);
                test_pass = 1'b0;
            end

            if (test_pass) begin
                pass_count = pass_count + 1;
                $display("[PASS][TEST %0d] seq_valid=0 상태 유지",
                         test_number);
            end
            else begin
                fail_count = fail_count + 1;
            end
        end
    endtask

    // 비동기 Active-High Reset 동작을 검사한다.
    task reset_and_check;
        reg test_pass;
        begin
            test_number = test_number + 1;
            test_pass   = 1'b1;

            @(negedge clk);
            reset_p = 1'b1;
            #1;

            if ((last_rx_seq     !== 8'h00) ||
                (seq_initialized !== 1'b0)  ||
                (seq_accept      !== 1'b0)  ||
                (seq_ok          !== 1'b0)  ||
                (seq_duplicate   !== 1'b0)  ||
                (seq_gap         !== 1'b0)  ||
                (seq_old         !== 1'b0)) begin
                $display("[FAIL][TEST %0d] Reset 출력 확인 필요", test_number);
                test_pass = 1'b0;
            end

            repeat (2) @(negedge clk);
            reset_p = 1'b0;
            repeat (2) @(negedge clk);

            if (test_pass) begin
                pass_count = pass_count + 1;
                $display("[PASS][TEST %0d] Reset 출력 정상", test_number);
            end
            else begin
                fail_count = fail_count + 1;
            end
        end
    endtask

    initial begin
        reset_p     = 1'b1;
        clear       = 1'b0;
        seq_valid   = 1'b0;
        tb_seq_value = 8'h00;
        pass_count  = 0;
        fail_count  = 0;
        test_number = 0;

        // ---------------------------------------------------------------------
        // TEST 1: 초기 Reset
        // ---------------------------------------------------------------------
        reset_and_check();

        // ---------------------------------------------------------------------
        // TEST 2: 최초 프레임
        // 비교 기준이 없으므로 SEQ=10을 기준으로 등록하고 허용한다.
        // ---------------------------------------------------------------------
        send_and_check(8'h10, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 8'h10, 1'b1);

        // ---------------------------------------------------------------------
        // TEST 3: 정상 증가 10 -> 11
        // ---------------------------------------------------------------------
        send_and_check(8'h11, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 8'h11, 1'b1);

        // ---------------------------------------------------------------------
        // TEST 4: Duplicate 11 -> 11
        // 폐기하며 last_rx_seq는 11을 유지한다.
        // ---------------------------------------------------------------------
        send_and_check(8'h11, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 8'h11, 1'b1);

        // ---------------------------------------------------------------------
        // TEST 5: Gap 11 -> 14
        // 12, 13이 빠졌지만 현재 프레임은 후보로 허용하고 14로 재동기화한다.
        // ---------------------------------------------------------------------
        send_and_check(8'h14, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 8'h14, 1'b1);

        // ---------------------------------------------------------------------
        // TEST 6: Gap 다음 정상 복구 14 -> 15
        // ---------------------------------------------------------------------
        send_and_check(8'h15, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 8'h15, 1'b1);

        // ---------------------------------------------------------------------
        // TEST 7: Old/Late 15 -> 14
        // 늦게 도착한 과거 프레임이므로 폐기하며 기준값은 15를 유지한다.
        // ---------------------------------------------------------------------
        send_and_check(8'h14, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 8'h15, 1'b1);

        // ---------------------------------------------------------------------
        // TEST 8: forward_distance=127 경계값
        // 8'h94 - 8'h15 = 8'h7F이므로 Gap으로 허용한다.
        // ---------------------------------------------------------------------
        send_and_check(8'h94, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 8'h94, 1'b1);

        // ---------------------------------------------------------------------
        // TEST 9: forward_distance=128 경계값
        // 8'h14 - 8'h94 = 8'h80이므로 Old로 폐기한다.
        // ---------------------------------------------------------------------
        send_and_check(8'h14, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 8'h94, 1'b1);

        // ---------------------------------------------------------------------
        // TEST 10: seq_valid=0
        // seq_value가 바뀌어도 last_rx_seq와 판정 출력은 변하면 안 된다.
        // ---------------------------------------------------------------------
        check_idle_hold(8'hAA, 8'h94);

        // ---------------------------------------------------------------------
        // TEST 11: 순환 증가 검증을 위한 기준값 초기화
        // ---------------------------------------------------------------------
        reset_and_check();

        // ---------------------------------------------------------------------
        // TEST 12~15: FE -> FF -> 00 -> 01
        // 8비트 SEQ가 255 이후 0으로 돌아가도 모두 정상 증가이다.
        // ---------------------------------------------------------------------
        send_and_check(8'hFE, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 8'hFE, 1'b1);
        send_and_check(8'hFF, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 8'hFF, 1'b1);
        send_and_check(8'h00, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 8'h00, 1'b1);
        send_and_check(8'h01, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 8'h01, 1'b1);

        // ---------------------------------------------------------------------
        // 최종 결과
        // ---------------------------------------------------------------------
        $display("\n==================================================");
        $display("SEQ MONITOR TB RESULT: PASS=%0d, FAIL=%0d",
                 pass_count, fail_count);

        if (fail_count == 0)
            $display("ALL TESTS PASSED");
        else
            $display("TEST FAILED");

        $display("==================================================\n");
        $finish;
    end

    // Testbench가 예상치 못하게 멈췄을 때 무한 시뮬레이션을 방지한다.
    initial begin
        #(100_000);
        $display("[FAIL] 전체 시뮬레이션 Timeout");
        $finish;
    end

endmodule
