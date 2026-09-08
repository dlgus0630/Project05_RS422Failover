`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// uart_tx Self-checking Testbench
//
// 검증 항목
//   1. Reset/Idle 상태: TX=1, ready=1, busy=0
//   2. UART 8-N-1 및 LSB-first 순서
//   3. 한 Bit가 정확히 CLKS_PER_BIT 동안 유지되는지 확인
//   4. Handshake 뒤 tx_data가 바뀌어도 송신 데이터가 유지되는지 확인
//   5. Busy 중 제시한 다음 Byte가 ready까지 기다렸다가 연속 송신되는지 확인
//   6. 송신 중 Reset 시 즉시 Idle로 복귀하는지 확인
//   7. tx_done이 Byte당 정확히 1클럭 Pulse인지 확인
//////////////////////////////////////////////////////////////////////////////////

module tb_uart_tx;

    localparam integer CLK_FREQ_HZ      = 40;
    localparam integer BAUD_RATE        = 10;
    localparam integer CLKS_PER_BIT     = 4;
    localparam integer CLOCK_PERIOD_NS  = 10;
    localparam integer HALF_BIT_TIME_NS =
        (CLKS_PER_BIT * CLOCK_PERIOD_NS) / 2;
    localparam integer BIT_TIME_NS =
        CLKS_PER_BIT * CLOCK_PERIOD_NS;

    reg        clk;
    reg        reset_p;
    reg        clear;
    reg        tx_valid;
    wire       tx_ready;
    reg  [7:0] tx_data;

    wire       uart_txd;
    wire       tx_busy;
    wire       tx_done;

    integer error_count;
    integer done_count;
    integer accepted_count;
    integer cycle_count;
    integer start_cycle;
    integer done_cycle;

    uart_tx #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE)
    ) dut (
        .clk      (clk),
        .reset_p  (reset_p),
        .clear    (clear),
        .tx_valid (tx_valid),
        .tx_ready (tx_ready),
        .tx_data  (tx_data),
        .uart_txd (uart_txd),
        .tx_busy  (tx_busy),
        .tx_done  (tx_done)
    );

    initial clk = 1'b0;
    always #(CLOCK_PERIOD_NS / 2) clk = ~clk;

    // Handshake와 전체 Clock 수는 입력이 수락되는 상승 에지에서 센다.
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            cycle_count    = 0;
            accepted_count = 0;
        end
        else begin
            cycle_count = cycle_count + 1;

            if (tx_valid && tx_ready) begin
                accepted_count = accepted_count + 1;
                start_cycle = cycle_count;
            end
        end
    end

    // tx_done은 DUT의 상승 에지 Nonblocking 할당 뒤에 갱신된다.
    // 따라서 안정된 하강 에지에서 완료 횟수와 완료 Clock을 기록한다.
    always @(negedge clk or posedge reset_p) begin
        if (reset_p) begin
            done_count = 0;
            done_cycle = 0;
        end
        else begin
            if (tx_done) begin
                done_count = done_count + 1;
                done_cycle = cycle_count;
            end
        end
    end

    task apply_reset;
        begin
            reset_p = 1'b1;
            repeat (2) @(negedge clk);
            reset_p = 1'b0;
            #1;

            if ((uart_txd !== 1'b1) ||
                (tx_ready !== 1'b1) ||
                (tx_busy  !== 1'b0) ||
                (tx_done  !== 1'b0)) begin
                error_count = error_count + 1;
                $display(
                    "[FAIL] time=%0t invalid idle state TX=%b ready=%b busy=%b done=%b",
                    $time, uart_txd, tx_ready, tx_busy, tx_done
                );
            end
        end
    endtask

    // tx_ready 상태에서 한 클럭 Pulse로 Byte를 전달한다.
    // change_after_accept=1이면 Handshake 직후 외부 tx_data를 바꾼다.
    task send_pulse;
        input [7:0] data;
        input       change_after_accept;
        begin
            wait (tx_ready === 1'b1);
            @(negedge clk);
            tx_data  = data;
            tx_valid = 1'b1;

            @(posedge clk);
            #1;

            if ((tx_busy !== 1'b1) ||
                (tx_ready !== 1'b0) ||
                (uart_txd !== 1'b0)) begin
                error_count = error_count + 1;
                $display(
                    "[FAIL] time=%0t byte was not accepted correctly",
                    $time
                );
            end

            @(negedge clk);
            tx_valid = 1'b0;

            if (change_after_accept)
                tx_data = ~data;
        end
    endtask

    // Busy 중에 다음 Byte를 제시하고 ready가 될 때까지 유지한다.
    // raw_frame_buffer의 valid/ready 동작과 같은 조건이다.
    task hold_byte_until_accepted;
        input [7:0] data;
        begin
            @(negedge clk);
            tx_data  = data;
            tx_valid = 1'b1;

            while (tx_ready !== 1'b1)
                @(negedge clk);

            // ready=1인 다음 상승 에지에서 Handshake가 일어난다.
            @(posedge clk);
            #1;

            if ((tx_busy !== 1'b1) || (uart_txd !== 1'b0)) begin
                error_count = error_count + 1;
                $display(
                    "[FAIL] time=%0t held byte was not accepted",
                    $time
                );
            end

            @(negedge clk);
            tx_valid = 1'b0;
        end
    endtask

    // 외부 UART 선만 보고 8-N-1 Byte를 검사한다.
    task check_uart_byte;
        input [7:0] expected_data;
        integer bit_number;
        begin
            // 새 Byte의 Start Bit를 기다린다.
            wait ((uart_txd === 1'b0) && (tx_busy === 1'b1));

            // 각 Bit의 중앙에서 샘플링한다.
            #(HALF_BIT_TIME_NS);

            if (uart_txd !== 1'b0) begin
                error_count = error_count + 1;
                $display("[FAIL] time=%0t Start bit is not 0", $time);
            end

            for (bit_number = 0;
                 bit_number < 8;
                 bit_number = bit_number + 1) begin
                #(BIT_TIME_NS);

                if (uart_txd !== expected_data[bit_number]) begin
                    error_count = error_count + 1;
                    $display(
                        "[FAIL] time=%0t Data[%0d]=%b expected=%b",
                        $time,
                        bit_number,
                        uart_txd,
                        expected_data[bit_number]
                    );
                end
            end

            #(BIT_TIME_NS);

            if (uart_txd !== 1'b1) begin
                error_count = error_count + 1;
                $display("[FAIL] time=%0t Stop bit is not 1", $time);
            end
        end
    endtask

    initial begin
        reset_p       = 1'b1;
        clear         = 1'b0;
        tx_valid      = 1'b0;
        tx_data       = 8'd0;
        error_count   = 0;
        done_count    = 0;
        accepted_count = 0;
        cycle_count   = 0;
        start_cycle   = 0;
        done_cycle    = 0;

        apply_reset;

        // ---------------------------------------------------------------------
        // 1. 8'hA5: Start -> 1,0,1,0,0,1,0,1 -> Stop
        //    Handshake 후 입력을 바꿔도 A5가 유지되어야 한다.
        // ---------------------------------------------------------------------
        fork
            check_uart_byte(8'hA5);
            send_pulse(8'hA5, 1'b1);
        join

        wait (tx_done === 1'b1);
        @(negedge clk);
        #1;

        if ((done_cycle - start_cycle) !== (10 * CLKS_PER_BIT)) begin
            error_count = error_count + 1;
            $display(
                "[FAIL] UART frame clocks=%0d expected=%0d",
                done_cycle - start_cycle,
                10 * CLKS_PER_BIT
            );
        end

        @(posedge clk);
        #1;

        if (tx_done !== 1'b0) begin
            error_count = error_count + 1;
            $display("[FAIL] tx_done is longer than one clock");
        end

        // ---------------------------------------------------------------------
        // 2. 첫 Byte 송신 중 두 번째 Byte를 valid로 유지한다.
        //    첫 Byte 완료 뒤 ready가 열리면 두 번째 Byte가 수락되어야 한다.
        // ---------------------------------------------------------------------
        fork
            begin
                check_uart_byte(8'h3C);
                check_uart_byte(8'hC3);
            end
            begin
                send_pulse(8'h3C, 1'b0);

                // 첫 Byte가 Busy인 동안 다음 Byte를 제시한다.
                wait (tx_busy === 1'b1);
                hold_byte_until_accepted(8'hC3);
            end
        join

        wait (done_count == 3);
        @(posedge clk);
        #1;

        if ((accepted_count !== 3) || (done_count !== 3)) begin
            error_count = error_count + 1;
            $display(
                "[FAIL] accepted=%0d done=%0d expected=3/3",
                accepted_count,
                done_count
            );
        end

        // ---------------------------------------------------------------------
        // 3. 송신 중 Reset을 넣으면 진행 중 Byte를 버리고 즉시 Idle 복귀
        // ---------------------------------------------------------------------
        wait (tx_ready === 1'b1);
        @(negedge clk);
        tx_data  = 8'hFF;
        tx_valid = 1'b1;

        @(posedge clk);
        @(negedge clk);
        tx_valid = 1'b0;

        repeat (CLKS_PER_BIT + 1) @(posedge clk);
        #1 reset_p = 1'b1;
        #1;

        if ((uart_txd !== 1'b1) ||
            (tx_busy  !== 1'b0) ||
            (tx_ready !== 1'b0) ||
            (tx_done  !== 1'b0)) begin
            error_count = error_count + 1;
            $display(
                "[FAIL] time=%0t reset did not abort TX safely",
                $time
            );
        end

        @(negedge clk);
        reset_p = 1'b0;
        #1;

        if ((uart_txd !== 1'b1) ||
            (tx_busy  !== 1'b0) ||
            (tx_ready !== 1'b1)) begin
            error_count = error_count + 1;
            $display("[FAIL] UART did not return to idle after reset");
        end

        if (error_count == 0)
            $display("[PASS] All uart_tx core tests passed.");
        else
            $display("[FAIL] error_count=%0d", error_count);

        $finish;
    end

    // 무한 시뮬레이션 방지
    initial begin
        #10000;
        $display("[FAIL] Simulation timeout");
        $finish;
    end

endmodule
