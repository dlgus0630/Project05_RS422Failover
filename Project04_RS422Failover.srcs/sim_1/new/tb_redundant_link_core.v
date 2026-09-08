`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// tb_redundant_link_core
//
// End-to-End Self-checking Testbench
//   1. AXI 설정 반영 및 System Enable
//   2. 정상 A/B Pair -> 1개 출력, ID/CMD 변환, CRC 재계산
//   3. 같은 SEQ의 A/B Data Mismatch -> 출력 폐기, Event/IRQ
//   4. Single A Timeout -> Fallback 출력
//   5. 늦게 온 동일 B Frame -> Duplicate 차단
//   6. A CRC Error + 정상 B -> B Fallback 출력
//   7. Event FIFO AXI Read/Pop/Clear
//   8. 런타임 Channel Timeout/Fail Threshold 반영
//////////////////////////////////////////////////////////////////////////////////

module tb_redundant_link_core;

    localparam integer CLK_FREQ_HZ = 1_000_000;
    localparam integer BAUD_RATE   = 100_000;

    reg clk;
    reg reset_p;

    wire source_a_txd;
    wire source_b_txd;
    reg  source_a_start;
    reg  source_b_start;
    reg  [7:0] source_a_byte_count;
    reg  [7:0] source_b_byte_count;
    wire source_a_busy;
    wire source_b_busy;
    wire source_a_done;
    wire source_b_done;

    wire rs422_tx_out;

    reg  [6:0]  s_axil_awaddr;
    reg         s_axil_awvalid;
    wire        s_axil_awready;
    reg  [31:0] s_axil_wdata;
    reg  [3:0]  s_axil_wstrb;
    reg         s_axil_wvalid;
    wire        s_axil_wready;
    wire [1:0]  s_axil_bresp;
    wire        s_axil_bvalid;
    reg         s_axil_bready;

    reg  [6:0]  s_axil_araddr;
    reg         s_axil_arvalid;
    wire        s_axil_arready;
    wire [31:0] s_axil_rdata;
    wire [1:0]  s_axil_rresp;
    wire        s_axil_rvalid;
    reg         s_axil_rready;

    wire        irq;
    wire [15:0] led;
    wire [6:0]  seg;
    wire        dp;
    wire [3:0]  an;

    wire [7:0] output_rx_data;
    wire       output_rx_valid;
    wire       output_rx_frame_error;

    reg [7:0] output_byte_mem [0:255];
    reg [7:0] expected_byte_mem [0:31];
    integer   output_byte_count;
    integer   expected_read_index;
    integer   error_count;
    integer   check_index;
    integer   saved_output_count;
    reg [31:0] axi_read_value;

    redundant_link_core #(
        .CLK_FREQ_HZ            (CLK_FREQ_HZ),
        .BAUD_RATE              (BAUD_RATE),
        .INTERBYTE_TIMEOUT_CLKS (200),
        .FRAME_TIMEOUT_CLKS     (2_000),
        .PAIR_TIMEOUT_CYCLES    (50),
        .CHANNEL_TIMEOUT_CYCLES (20_000),
        .EVENT_FIFO_DEPTH       (16),
        .HISTORY_DEPTH          (4),
        .SCAN_TICK_CYCLES       (4),
        .HEARTBEAT_CYCLES       (16),
        .ALERT_HOLD_CYCLES      (32)
    ) dut (
        .clk             (clk),
        .reset_p         (reset_p),
        .rs422_rx_a      (source_a_txd),
        .rs422_rx_b      (source_b_txd),
        .rs422_tx_out    (rs422_tx_out),
        .s_axil_awaddr   (s_axil_awaddr),
        .s_axil_awvalid  (s_axil_awvalid),
        .s_axil_awready  (s_axil_awready),
        .s_axil_wdata    (s_axil_wdata),
        .s_axil_wstrb    (s_axil_wstrb),
        .s_axil_wvalid   (s_axil_wvalid),
        .s_axil_wready   (s_axil_wready),
        .s_axil_bresp    (s_axil_bresp),
        .s_axil_bvalid   (s_axil_bvalid),
        .s_axil_bready   (s_axil_bready),
        .s_axil_araddr   (s_axil_araddr),
        .s_axil_arvalid  (s_axil_arvalid),
        .s_axil_arready  (s_axil_arready),
        .s_axil_rdata    (s_axil_rdata),
        .s_axil_rresp    (s_axil_rresp),
        .s_axil_rvalid   (s_axil_rvalid),
        .s_axil_rready   (s_axil_rready),
        .irq             (irq),
        .led             (led),
        .seg             (seg),
        .dp              (dp),
        .an              (an)
    );

    tb_uart_frame_source #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE)
    ) source_a (
        .clk        (clk),
        .reset_p    (reset_p),
        .start      (source_a_start),
        .byte_count (source_a_byte_count),
        .uart_txd   (source_a_txd),
        .busy       (source_a_busy),
        .done       (source_a_done)
    );

    tb_uart_frame_source #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE)
    ) source_b (
        .clk        (clk),
        .reset_p    (reset_p),
        .start      (source_b_start),
        .byte_count (source_b_byte_count),
        .uart_txd   (source_b_txd),
        .busy       (source_b_busy),
        .done       (source_b_done)
    );

    uart_rx #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE)
    ) output_monitor (
        .clk            (clk),
        .reset_p        (reset_p),
        .clear          (1'b0),
        .rx             (rs422_tx_out),
        .rx_data        (output_rx_data),
        .rx_valid       (output_rx_valid),
        .rx_frame_error (output_rx_frame_error)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    always @(posedge clk) begin
        if (reset_p) begin
            output_byte_count = 0;
        end
        else begin
            if (output_rx_frame_error) begin
                $display("[FAIL] Output UART framing error at %0t", $time);
                error_count = error_count + 1;
            end

            if (output_rx_valid) begin
                output_byte_mem[output_byte_count] = output_rx_data;
                output_byte_count = output_byte_count + 1;
            end
        end
    end

`ifdef DEBUG_CORE
    always @(posedge clk) begin
        #1;
        if (dut.matcher_result_valid)
            $display(
                "[DBG] matcher kind=%b equal=%b timeout=%b seq=%h",
                dut.matcher_result_kind, dut.matcher_pair_equal,
                dut.matcher_timeout, dut.matcher_sequence
            );
        if (dut.decision_valid)
            $display(
                "[DBG] decision accept=%b degraded=%b mismatch=%b both=%b seq=%h reserved=%b",
                dut.decision_accept, dut.decision_degraded,
                dut.decision_mismatch_drop, dut.decision_both_invalid,
                dut.decision_sequence, dut.translation_reserved
            );
        if (dut.duplicate_out_valid || dut.duplicate_drop)
            $display(
                "[DBG] duplicate out=%b drop=%b seq=%h",
                dut.duplicate_out_valid, dut.duplicate_drop,
                dut.duplicate_sequence
            );
        if (dut.translated_frame_fire)
            $display(
                "[DBG] translated fire seq=%h crc=%h",
                dut.translation_sequence, dut.translation_crc_reg
            );
        if (dut.arbiter_event_valid && dut.arbiter_event_ready)
            $display(
                "[DBG] event code=%h ch=%b seq=%h",
                dut.arbiter_event_data[31:24],
                dut.arbiter_event_data[23:22],
                dut.arbiter_event_data[21:14]
            );
        if (output_rx_valid)
            $display("[DBG] output byte=%h count=%0d",
                     output_rx_data, output_byte_count);
    end
`endif

    task axi_write;
        input [6:0]  address;
        input [31:0] data;
        begin
            @(negedge clk);
            s_axil_awaddr  = address;
            s_axil_wdata   = data;
            s_axil_wstrb   = 4'hF;
            s_axil_awvalid = 1'b1;
            s_axil_wvalid  = 1'b1;

            @(posedge clk);
            #1;

            @(negedge clk);
            s_axil_awvalid = 1'b0;
            s_axil_wvalid  = 1'b0;

            while (!s_axil_bvalid)
                @(posedge clk);

            #1;
            if (s_axil_bresp !== 2'b00) begin
                $display(
                    "[FAIL] AXI write addr=%h BRESP=%b",
                    address, s_axil_bresp
                );
                error_count = error_count + 1;
            end

            @(posedge clk);
            #1;
        end
    endtask

    task axi_read;
        input [6:0] address;
        begin
            @(negedge clk);
            s_axil_araddr  = address;
            s_axil_arvalid = 1'b1;

            @(posedge clk);
            #1;

            @(negedge clk);
            s_axil_arvalid = 1'b0;

            while (!s_axil_rvalid)
                @(posedge clk);

            #1;
            axi_read_value = s_axil_rdata;

            if (s_axil_rresp !== 2'b00) begin
                $display(
                    "[FAIL] AXI read addr=%h RRESP=%b",
                    address, s_axil_rresp
                );
                error_count = error_count + 1;
            end

            @(posedge clk);
            #1;
        end
    endtask

    task start_both_sources;
        begin
            @(negedge clk);
            source_a_start = 1'b1;
            source_b_start = 1'b1;

            @(negedge clk);
            source_a_start = 1'b0;
            source_b_start = 1'b0;

            while (!source_a_done || !source_b_done)
                @(posedge clk);
        end
    endtask

    task start_a_source;
        begin
            @(negedge clk);
            source_a_start = 1'b1;

            @(negedge clk);
            source_a_start = 1'b0;

            while (!source_a_done)
                @(posedge clk);
        end
    endtask

    task start_b_source;
        begin
            @(negedge clk);
            source_b_start = 1'b1;

            @(negedge clk);
            source_b_start = 1'b0;

            while (!source_b_done)
                @(posedge clk);
        end
    endtask

    task wait_and_check_ten_output_bytes;
        begin
            while (output_byte_count < (expected_read_index + 10))
                @(posedge clk);

            #1;
            for (check_index = 0;
                 check_index < 10;
                 check_index = check_index + 1) begin
                if (output_byte_mem[expected_read_index + check_index]
                    !== expected_byte_mem[check_index]) begin
                    $display(
                        "[FAIL] Output byte[%0d] expected=%h actual=%h",
                        check_index,
                        expected_byte_mem[check_index],
                        output_byte_mem[expected_read_index + check_index]
                    );
                    error_count = error_count + 1;
                end
            end

            expected_read_index = expected_read_index + 10;
        end
    endtask

    task expect_no_new_output;
        input integer cycle_count;
        begin
            saved_output_count = output_byte_count;
            repeat (cycle_count)
                @(posedge clk);

            #1;
            if (output_byte_count != saved_output_count) begin
                $display(
                    "[FAIL] Unexpected output: before=%0d after=%0d",
                    saved_output_count, output_byte_count
                );
                error_count = error_count + 1;
            end
        end
    endtask

    initial begin
        error_count         = 0;
        expected_read_index = 0;
        axi_read_value      = 32'd0;

        reset_p             = 1'b1;
        source_a_start      = 1'b0;
        source_b_start      = 1'b0;
        source_a_byte_count = 8'd0;
        source_b_byte_count = 8'd0;

        s_axil_awaddr       = 7'd0;
        s_axil_awvalid      = 1'b0;
        s_axil_wdata        = 32'd0;
        s_axil_wstrb        = 4'hF;
        s_axil_wvalid       = 1'b0;
        s_axil_bready       = 1'b1;
        s_axil_araddr       = 7'd0;
        s_axil_arvalid      = 1'b0;
        s_axil_rready       = 1'b1;

        repeat (8) @(posedge clk);
        @(negedge clk);
        reset_p = 1'b0;
        repeat (4) @(posedge clk);

        // -------------------------------------------------------------
        // AXI Configuration
        // FAIL=3, RECOVERY=3, Preferred=A
        // -------------------------------------------------------------
        axi_write(7'h08, 32'h0003_0300);
        axi_write(7'h0C, 32'd50);
        axi_write(7'h10, 32'd20_000);
        axi_write(7'h14, 32'h0000_0055);
        axi_write(7'h18, 32'h0000_00A0);
        axi_write(7'h1C, 32'h0000_00A1);
        axi_write(7'h20, 32'h0000_00A2);
        axi_write(7'h24, 32'h0000_00A3);
        axi_write(7'h28, 32'h0000_003F);
        axi_write(7'h00, 32'h0000_0001);
        repeat (5) @(posedge clk);

        // -------------------------------------------------------------
        // Test 1: 정상 Pair
        // Input CRC EDEB
        // Output: A5 5A 05 55 A0 10 DE AD 80 0E
        // -------------------------------------------------------------
        source_a.mem[0] = 8'hA5;
        source_a.mem[1] = 8'h5A;
        source_a.mem[2] = 8'h05;
        source_a.mem[3] = 8'h01;
        source_a.mem[4] = 8'h10;
        source_a.mem[5] = 8'h10;
        source_a.mem[6] = 8'hDE;
        source_a.mem[7] = 8'hAD;
        source_a.mem[8] = 8'hED;
        source_a.mem[9] = 8'hEB;

        for (check_index = 0;
             check_index < 10;
             check_index = check_index + 1)
            source_b.mem[check_index] = source_a.mem[check_index];

        source_a_byte_count = 8'd10;
        source_b_byte_count = 8'd10;

        expected_byte_mem[0] = 8'hA5;
        expected_byte_mem[1] = 8'h5A;
        expected_byte_mem[2] = 8'h05;
        expected_byte_mem[3] = 8'h55;
        expected_byte_mem[4] = 8'hA0;
        expected_byte_mem[5] = 8'h10;
        expected_byte_mem[6] = 8'hDE;
        expected_byte_mem[7] = 8'hAD;
        expected_byte_mem[8] = 8'h80;
        expected_byte_mem[9] = 8'h0E;

        start_both_sources;
        wait_and_check_ten_output_bytes;

        axi_read(7'h04);
        if (axi_read_value[2:0] !== 3'b111) begin
            $display(
                "[FAIL] Normal STATUS enable/alive expected=111 actual=%b",
                axi_read_value[2:0]
            );
            error_count = error_count + 1;
        end

        // -------------------------------------------------------------
        // Test 2: 같은 SEQ, 다른 Payload -> 무조건 폐기
        // -------------------------------------------------------------
        source_a.mem[0] = 8'hA5;
        source_a.mem[1] = 8'h5A;
        source_a.mem[2] = 8'h05;
        source_a.mem[3] = 8'h01;
        source_a.mem[4] = 8'h10;
        source_a.mem[5] = 8'h11;
        source_a.mem[6] = 8'h11;
        source_a.mem[7] = 8'h11;
        source_a.mem[8] = 8'hBA;
        source_a.mem[9] = 8'hE6;

        source_b.mem[0] = 8'hA5;
        source_b.mem[1] = 8'h5A;
        source_b.mem[2] = 8'h05;
        source_b.mem[3] = 8'h01;
        source_b.mem[4] = 8'h10;
        source_b.mem[5] = 8'h11;
        source_b.mem[6] = 8'h22;
        source_b.mem[7] = 8'h22;
        source_b.mem[8] = 8'hEC;
        source_b.mem[9] = 8'h10;

        start_both_sources;
        expect_no_new_output(300);

        axi_read(7'h04);
        if (!axi_read_value[8] || !axi_read_value[7]) begin
            $display(
                "[FAIL] Mismatch STATUS latch/fifo expected 1/1 actual=%b/%b",
                axi_read_value[8], axi_read_value[7]
            );
            error_count = error_count + 1;
        end

        if (!irq) begin
            $display("[FAIL] IRQ did not assert for mismatch event");
            error_count = error_count + 1;
        end

        axi_read(7'h34);
        if (axi_read_value[31:24] !== 8'h0B) begin
            $display(
                "[FAIL] First Event code expected=0B actual=%h",
                axi_read_value[31:24]
            );
            error_count = error_count + 1;
        end
        axi_write(7'h3C, 32'h0000_0001);

        // -------------------------------------------------------------
        // Test 3: Single A -> Timeout 뒤 Fallback 출력
        // Input CRC 501E, Output CRC 3DFB
        // -------------------------------------------------------------
        source_a.mem[0] = 8'hA5;
        source_a.mem[1] = 8'h5A;
        source_a.mem[2] = 8'h05;
        source_a.mem[3] = 8'h01;
        source_a.mem[4] = 8'h11;
        source_a.mem[5] = 8'h12;
        source_a.mem[6] = 8'hCA;
        source_a.mem[7] = 8'hFE;
        source_a.mem[8] = 8'h50;
        source_a.mem[9] = 8'h1E;

        expected_byte_mem[0] = 8'hA5;
        expected_byte_mem[1] = 8'h5A;
        expected_byte_mem[2] = 8'h05;
        expected_byte_mem[3] = 8'h55;
        expected_byte_mem[4] = 8'hA1;
        expected_byte_mem[5] = 8'h12;
        expected_byte_mem[6] = 8'hCA;
        expected_byte_mem[7] = 8'hFE;
        expected_byte_mem[8] = 8'h3D;
        expected_byte_mem[9] = 8'hFB;

        start_a_source;
        wait_and_check_ten_output_bytes;

        // -------------------------------------------------------------
        // Test 4: 늦게 도착한 동일 B Frame -> Duplicate Drop
        // -------------------------------------------------------------
        for (check_index = 0;
             check_index < 10;
             check_index = check_index + 1)
            source_b.mem[check_index] = source_a.mem[check_index];

        start_b_source;
        expect_no_new_output(300);

        // -------------------------------------------------------------
        // Test 5: A CRC Error + 정상 B -> B Fallback 출력
        // A의 정상 CRC F359 대신 F358 주입
        // B Input CRC 3A7F, Output CRC 579A
        // -------------------------------------------------------------
        source_a.mem[0] = 8'hA5;
        source_a.mem[1] = 8'h5A;
        source_a.mem[2] = 8'h05;
        source_a.mem[3] = 8'h01;
        source_a.mem[4] = 8'h12;
        source_a.mem[5] = 8'h13;
        source_a.mem[6] = 8'hAA;
        source_a.mem[7] = 8'h55;
        source_a.mem[8] = 8'hF3;
        source_a.mem[9] = 8'h58;

        source_b.mem[0] = 8'hA5;
        source_b.mem[1] = 8'h5A;
        source_b.mem[2] = 8'h05;
        source_b.mem[3] = 8'h01;
        source_b.mem[4] = 8'h12;
        source_b.mem[5] = 8'h13;
        source_b.mem[6] = 8'hBE;
        source_b.mem[7] = 8'hEF;
        source_b.mem[8] = 8'h3A;
        source_b.mem[9] = 8'h7F;

        expected_byte_mem[0] = 8'hA5;
        expected_byte_mem[1] = 8'h5A;
        expected_byte_mem[2] = 8'h05;
        expected_byte_mem[3] = 8'h55;
        expected_byte_mem[4] = 8'hA2;
        expected_byte_mem[5] = 8'h13;
        expected_byte_mem[6] = 8'hBE;
        expected_byte_mem[7] = 8'hEF;
        expected_byte_mem[8] = 8'h57;
        expected_byte_mem[9] = 8'h9A;

        start_both_sources;
        wait_and_check_ten_output_bytes;

        // FND가 같은 Frame의 선택 채널/SEQ를 저장해야 한다.
        // 이전 구현은 last_selected_b의 이전 Frame 값을 캡처했다.
        @(negedge clk);
        if ((dut.u_status_display.stored_selected_b !== 1'b1) ||
            (dut.u_status_display.stored_sequence !== 8'h13)) begin
            $display(
                "[FAIL] FND selection expected B/13 actual=%b/%h",
                dut.u_status_display.stored_selected_b,
                dut.u_status_display.stored_sequence
            );
            error_count = error_count + 1;
        end

        // -------------------------------------------------------------
        // Test 6: Event/Statistics Clear 실제 반영
        // -------------------------------------------------------------
        axi_write(7'h00, 32'h0000_0007);
        repeat (8) @(posedge clk);
        axi_read(7'h30);

        if (!axi_read_value[0] || (axi_read_value[15:8] != 8'd0)) begin
            $display(
                "[FAIL] Event FIFO clear failed status=%h",
                axi_read_value
            );
            error_count = error_count + 1;
        end

        // -------------------------------------------------------------
        // Test 7: 런타임 Timeout/Fail Threshold가 실제 RTL에 반영
        // -------------------------------------------------------------
        axi_write(7'h08, 32'h0003_0100);
        axi_write(7'h10, 32'd200);
        repeat (260) @(posedge clk);

        axi_read(7'h04);
        if (axi_read_value[2:1] !== 2'b00) begin
            $display(
                "[FAIL] Channel timeout did not clear Alive bits: %b",
                axi_read_value[2:1]
            );
            error_count = error_count + 1;
        end

        repeat (5) @(posedge clk);
        if ((led[1] !== 1'b1) ||
            (led[15:2] !== 14'd0)) begin
            $display(
                "[FAIL] Aggregate Alert expected LED1=1, LED15:2=0 actual=%b/%h",
                led[1],
                led[15:2]
            );
            error_count = error_count + 1;
        end

        if (error_count == 0)
            $display(
                "[PASS] All redundant_link_core end-to-end tests passed."
            );
        else
            $display(
                "[FAIL] redundant_link_core tests failed: %0d error(s).",
                error_count
            );

        $finish;
    end

    initial begin
        #20_000_000;
        $display("[FAIL] Global timeout");
        $finish;
    end

endmodule

//////////////////////////////////////////////////////////////////////////////////
// Testbench-only UART Frame Source
//////////////////////////////////////////////////////////////////////////////////

module tb_uart_frame_source #(
    parameter integer CLK_FREQ_HZ = 1_000_000,
    parameter integer BAUD_RATE   = 100_000
)(
    input  wire       clk,
    input  wire       reset_p,
    input  wire       start,
    input  wire [7:0] byte_count,
    output wire       uart_txd,
    output reg        busy,
    output reg        done
);

    reg [7:0] mem [0:31];
    reg [7:0] index;
    reg       tx_valid;
    reg [7:0] tx_data;
    wire      tx_ready;
    wire      tx_busy_unused;
    wire      tx_done_unused;

    uart_tx #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE)
    ) transmitter (
        .clk      (clk),
        .reset_p  (reset_p),
        .clear    (1'b0),
        .tx_valid (tx_valid),
        .tx_ready (tx_ready),
        .tx_data  (tx_data),
        .uart_txd (uart_txd),
        .tx_busy  (tx_busy_unused),
        .tx_done  (tx_done_unused)
    );

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            index    <= 8'd0;
            tx_valid <= 1'b0;
            tx_data  <= 8'd0;
            busy     <= 1'b0;
            done     <= 1'b0;
        end
        else begin
            done <= 1'b0;

            if (start && !busy && (byte_count != 0)) begin
                index    <= 8'd0;
                tx_data  <= mem[0];
                tx_valid <= 1'b1;
                busy     <= 1'b1;
            end
            else if (busy && tx_valid && tx_ready) begin
                if (index == (byte_count - 1'b1)) begin
                    tx_valid <= 1'b0;
                    busy     <= 1'b0;
                    done     <= 1'b1;
                end
                else begin
                    index   <= index + 1'b1;
                    tx_data <= mem[index + 1'b1];
                end
            end
        end
    end

endmodule
