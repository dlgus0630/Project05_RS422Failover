`timescale 1ns / 1ps

module tb_uart_rx;

    // 실제 Basys3 설정과 동일하게 검증한다.
    localparam integer CLK_FREQ_HZ = 100_000_000;
    localparam integer BAUD_RATE   = 115_200;
    localparam integer CLKS_PER_BIT = CLK_FREQ_HZ / BAUD_RATE;
    localparam integer CLK_PERIOD_NS = 10;

    reg        clk;
    reg        reset_p;
    reg        clear;
    reg        rx;
    wire [7:0] rx_data;
    wire       rx_valid;
    wire       rx_frame_error;

    integer pass_count;
    integer fail_count;
    integer valid_count;
    integer frame_error_count;
    reg [7:0] last_rx_data;
    reg       prev_rx_valid;
    reg       prev_frame_error;

    uart_rx #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .BAUD_RATE  (BAUD_RATE)
    ) dut (
        .clk           (clk),
        .reset_p       (reset_p),
        .clear         (clear),
        .rx            (rx),
        .rx_data       (rx_data),
        .rx_valid      (rx_valid),
        .rx_frame_error(rx_frame_error)
    );

    // 100 MHz 시스템 클럭
    initial begin
        clk = 1'b0;
        forever #(CLK_PERIOD_NS / 2) clk = ~clk;
    end

    // 수신 이벤트 기록 및 1클럭 펄스 검사
    always @(negedge clk or posedge reset_p) begin
        if (reset_p) begin
            valid_count       <= 0;
            frame_error_count <= 0;
            last_rx_data      <= 8'h00;
            prev_rx_valid     <= 1'b0;
            prev_frame_error  <= 1'b0;
        end
        else begin
            if (rx_valid) begin
                valid_count  <= valid_count + 1;
                last_rx_data <= rx_data;
            end

            if (rx_frame_error)
                frame_error_count <= frame_error_count + 1;

            if (rx_valid && prev_rx_valid) begin
                $display("[FAIL] rx_valid가 1클럭보다 길게 유지됨");
                fail_count = fail_count + 1;
            end

            if (rx_frame_error && prev_frame_error) begin
                $display("[FAIL] rx_frame_error가 1클럭보다 길게 유지됨");
                fail_count = fail_count + 1;
            end

            prev_rx_valid    <= rx_valid;
            prev_frame_error <= rx_frame_error;
        end
    end

    // UART 8N1 정상 바이트 송신: Start(0), Data LSB first, Stop(1)
    task uart_send_byte;
        input [7:0] data;
        integer i;
        begin
            rx = 1'b0;
            repeat (CLKS_PER_BIT) @(negedge clk);

            for (i = 0; i < 8; i = i + 1) begin
                rx = data[i];
                repeat (CLKS_PER_BIT) @(negedge clk);
            end

            rx = 1'b1;
            repeat (CLKS_PER_BIT) @(negedge clk);
        end
    endtask

    // Stop Bit를 Low로 보내 Framing Error 발생
    task uart_send_bad_stop;
        input [7:0] data;
        integer i;
        begin
            rx = 1'b0;
            repeat (CLKS_PER_BIT) @(negedge clk);

            for (i = 0; i < 8; i = i + 1) begin
                rx = data[i];
                repeat (CLKS_PER_BIT) @(negedge clk);
            end

            rx = 1'b0;
            repeat (CLKS_PER_BIT) @(negedge clk);
            rx = 1'b1;
            repeat (CLKS_PER_BIT) @(negedge clk);
        end
    endtask

    // rx_valid가 발생할 때까지 기다린 뒤 데이터 자동 비교
    task expect_byte;
        input [7:0] expected;
        integer timeout_count;
        begin
            timeout_count = 0;
            while ((rx_valid !== 1'b1) &&
                   (timeout_count < CLKS_PER_BIT * 12)) begin
                @(negedge clk);
                timeout_count = timeout_count + 1;
            end

            if (rx_valid !== 1'b1) begin
                $display("[FAIL] 0x%02h 수신 Timeout", expected);
                fail_count = fail_count + 1;
            end
            else if (rx_data !== expected) begin
                $display("[FAIL] 예상=0x%02h, 실제=0x%02h", expected, rx_data);
                fail_count = fail_count + 1;
            end
            else begin
                $display("[PASS] 정상 수신 0x%02h", expected);
                pass_count = pass_count + 1;
            end

            // 연속 바이트 검사에서 같은 rx_valid 펄스를 두 번 읽지 않도록
            // 현재 1클럭 펄스가 끝난 뒤 다음 기대값으로 넘어간다.
            while (rx_valid === 1'b1)
                @(negedge clk);
        end
    endtask

    // 송신과 수신 확인을 동시에 수행한다.
    task send_and_expect;
        input [7:0] data;
        begin
            fork
                uart_send_byte(data);
                expect_byte(data);
            join
        end
    endtask

    initial begin
        pass_count        = 0;
        fail_count        = 0;
        valid_count       = 0;
        frame_error_count = 0;
        last_rx_data      = 8'h00;
        prev_rx_valid     = 1'b0;
        prev_frame_error  = 1'b0;
        reset_p           = 1'b1;
        clear             = 1'b0;
        rx                = 1'b1;

        repeat (5) @(negedge clk);
        reset_p = 1'b0;
        repeat (5) @(negedge clk);

        // -------------------------------------------------------------
        // TEST 1: 대표 데이터 패턴 정상 수신
        // -------------------------------------------------------------
        $display("\n[TEST 1] 대표 패턴 정상 수신");
        send_and_expect(8'h00);
        send_and_expect(8'hFF);
        send_and_expect(8'h55);
        send_and_expect(8'hAA);
        send_and_expect(8'hA5);

        // -------------------------------------------------------------
        // TEST 2: 바이트 사이 추가 Idle 없이 연속 수신
        // -------------------------------------------------------------
        $display("\n[TEST 2] 연속 바이트 수신");
        fork
            begin
                uart_send_byte(8'h12);
                uart_send_byte(8'h34);
                uart_send_byte(8'h56);
            end
            begin
                expect_byte(8'h12);
                expect_byte(8'h34);
                expect_byte(8'h56);
            end
        join

        // -------------------------------------------------------------
        // TEST 3: 짧은 Start glitch 무시
        // Start 중앙보다 짧은 Low이므로 바이트로 인식하면 안 된다.
        // -------------------------------------------------------------
        $display("\n[TEST 3] 짧은 Start glitch 무시");
        begin : glitch_test
            integer valid_before;
            valid_before = valid_count;
            rx = 1'b0;
            repeat (CLKS_PER_BIT / 4) @(negedge clk);
            rx = 1'b1;
            repeat (CLKS_PER_BIT * 2) @(negedge clk);

            if (valid_count == valid_before) begin
                $display("[PASS] Start glitch를 무시함");
                pass_count = pass_count + 1;
            end
            else begin
                $display("[FAIL] Start glitch를 정상 바이트로 잘못 인식함");
                fail_count = fail_count + 1;
            end
        end

        // -------------------------------------------------------------
        // TEST 4: 잘못된 Stop Bit 검출 및 이후 정상 복구
        // -------------------------------------------------------------
        $display("\n[TEST 4] Framing Error 검출 및 복구");
        begin : framing_error_test
            integer valid_before;
            integer error_before;
            valid_before = valid_count;
            error_before = frame_error_count;

            uart_send_bad_stop(8'h3C);
            repeat (5) @(negedge clk);

            if ((frame_error_count == error_before + 1) &&
                (valid_count == valid_before)) begin
                $display("[PASS] 잘못된 Stop Bit를 검출하고 데이터는 폐기함");
                pass_count = pass_count + 1;
            end
            else begin
                $display("[FAIL] Framing Error 처리 오류: valid 증가=%0d, error 증가=%0d",
                         valid_count - valid_before,
                         frame_error_count - error_before);
                fail_count = fail_count + 1;
            end
        end
        send_and_expect(8'hC3);

        // -------------------------------------------------------------
        // TEST 5: 수신 도중 Reset 시 현재 바이트 폐기 후 정상 복귀
        // -------------------------------------------------------------
        $display("\n[TEST 5] 수신 도중 Reset");
        begin : reset_test
            integer valid_before;
            valid_before = valid_count;

            rx = 1'b0;
            repeat (CLKS_PER_BIT) @(negedge clk);
            rx = 1'b1; // D0
            repeat (CLKS_PER_BIT) @(negedge clk);
            rx = 1'b0; // D1 전송 도중 Reset
            repeat (CLKS_PER_BIT / 3) @(negedge clk);
            reset_p = 1'b1;
            repeat (5) @(negedge clk);
            rx = 1'b1;
            reset_p = 1'b0;
            repeat (CLKS_PER_BIT * 2) @(negedge clk);

            if ((rx_valid == 1'b0) && (valid_count == 0)) begin
                // monitor 카운터도 Reset되므로 0이 정상이다.
                $display("[PASS] 수신 중이던 바이트를 폐기하고 Idle로 복귀함");
                pass_count = pass_count + 1;
            end
            else begin
                $display("[FAIL] Reset 중 잘못된 rx_valid 발생");
                fail_count = fail_count + 1;
            end
        end
        send_and_expect(8'h5A);

        // -------------------------------------------------------------
        // 최종 결과
        // -------------------------------------------------------------
        repeat (CLKS_PER_BIT) @(negedge clk);
        $display("\n========================================");
        $display("UART RX TEST RESULT: PASS=%0d, FAIL=%0d",
                 pass_count, fail_count);
        $display("========================================");

        if (fail_count == 0)
            $display("ALL TESTS PASSED");
        else
            $display("TEST FAILED");

        $finish;
    end

    // DUT가 응답하지 않아 시뮬레이션이 무한 실행되는 상황 방지
    initial begin
        #(2_000_000);
        $display("[FAIL] 전체 시뮬레이션 Timeout");
        $finish;
    end

endmodule
