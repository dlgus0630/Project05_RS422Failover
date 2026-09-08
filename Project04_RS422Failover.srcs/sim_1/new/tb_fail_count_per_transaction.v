`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// tb_fail_count_per_transaction
//
// 목적
//   한 Sequence의 A/B 입력 중 한 채널만 CRC 오류이고 반대 채널은 정상일 때,
//   CRC 오류와 Pair 미도착이 각각 Fail Event로 계산되는 현재 정책을 확인한다.
//
// 확인 순서
//   1. 정상 Pair로 A/B Alive 및 Sequence 기준을 만든다.
//   2. A CRC 오류 + B 정상 프레임 1건을 넣는다.
//      - CRC 오류 직후 Count
//      - Pair Timeout의 Single B 결과 뒤 Count
//   3. 같은 조건의 두 번째 CRC 오류에서 Threshold 3으로 Fault에
//      진입하는지 확인한다.
//   4. Reset 후 A/B 역할을 바꿔 대칭 동작도 확인한다.
//
// 구현 기준
//   CRC 오류는 Local Fail Event 1회, 이후 Pair Timeout의 상대 채널
//   미도착은 별도 Fail Event 1회로 계산한다.
//////////////////////////////////////////////////////////////////////////////////

module tb_fail_count_per_transaction;

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

    integer error_count;
    integer check_index;

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

    initial clk = 1'b0;
    always #5 clk = ~clk;

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

    task reset_and_configure;
        begin
            reset_p = 1'b1;
            source_a_start = 1'b0;
            source_b_start = 1'b0;

            repeat (8) @(posedge clk);
            @(negedge clk);
            reset_p = 1'b0;
            repeat (4) @(posedge clk);

            // FAIL=3, RECOVERY=3, Preferred=A
            axi_write(7'h08, 32'h0003_0300);
            axi_write(7'h0C, 32'd50);
            axi_write(7'h10, 32'd20_000);
            axi_write(7'h00, 32'h0000_0001);
            repeat (5) @(posedge clk);
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

    task load_normal_pair_seq10;
        begin
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
        end
    endtask

    task load_a_bad_b_good_seq11;
        begin
            // A: 정상 CRC BAE6의 LSB만 반전해 BAE7을 송신한다.
            source_a.mem[0] = 8'hA5;
            source_a.mem[1] = 8'h5A;
            source_a.mem[2] = 8'h05;
            source_a.mem[3] = 8'h01;
            source_a.mem[4] = 8'h10;
            source_a.mem[5] = 8'h11;
            source_a.mem[6] = 8'h11;
            source_a.mem[7] = 8'h11;
            source_a.mem[8] = 8'hBA;
            source_a.mem[9] = 8'hE7;

            // B: 같은 SEQ의 정상 프레임
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
        end
    endtask

    task load_a_bad_b_good_seq12;
        begin
            // 동일한 정상 프레임에서 A의 CRC_L만 반전한다.
            source_a.mem[0] = 8'hA5;
            source_a.mem[1] = 8'h5A;
            source_a.mem[2] = 8'h05;
            source_a.mem[3] = 8'h01;
            source_a.mem[4] = 8'h11;
            source_a.mem[5] = 8'h12;
            source_a.mem[6] = 8'hCA;
            source_a.mem[7] = 8'hFE;
            source_a.mem[8] = 8'h50;
            source_a.mem[9] = 8'h1F;

            for (check_index = 0;
                 check_index < 10;
                 check_index = check_index + 1)
                source_b.mem[check_index] = source_a.mem[check_index];

            source_b.mem[9] = 8'h1E;
        end
    endtask

    task load_a_good_b_bad_seq11;
        begin
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

            for (check_index = 0;
                 check_index < 10;
                 check_index = check_index + 1)
                source_b.mem[check_index] = source_a.mem[check_index];

            source_b.mem[9] = 8'hE7;
        end
    endtask

    task load_a_good_b_bad_seq12;
        begin
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

            for (check_index = 0;
                 check_index < 10;
                 check_index = check_index + 1)
                source_b.mem[check_index] = source_a.mem[check_index];

            source_b.mem[9] = 8'h1F;
        end
    endtask

    task wait_normal_pair;
        begin
            @(posedge dut.matcher_result_valid);
            if ((dut.matcher_result_kind !== 2'b01) ||
                !dut.matcher_pair_equal) begin
                $display(
                    "[FAIL] Baseline expected equal Pair, kind=%b equal=%b",
                    dut.matcher_result_kind, dut.matcher_pair_equal
                );
                error_count = error_count + 1;
            end

            @(posedge clk);
            #1;
        end
    endtask

    initial begin
        error_count         = 0;
        reset_p             = 1'b1;
        source_a_start      = 1'b0;
        source_b_start      = 1'b0;
        source_a_byte_count = 8'd10;
        source_b_byte_count = 8'd10;

        s_axil_awaddr       = 7'd0;
        s_axil_awvalid      = 1'b0;
        s_axil_wdata        = 32'd0;
        s_axil_wstrb        = 4'hF;
        s_axil_wvalid       = 1'b0;
        s_axil_bready       = 1'b1;
        s_axil_araddr       = 7'd0;
        s_axil_arvalid      = 1'b0;
        s_axil_rready       = 1'b1;

        // -------------------------------------------------------------
        // Channel A: CRC 오류 + 정상 B
        // -------------------------------------------------------------
        reset_and_configure;

        load_normal_pair_seq10;
        start_both_sources;
        wait_normal_pair;

        if ((dut.a_fail_count !== 8'd0) ||
            (dut.b_fail_count !== 8'd0) ||
            dut.a_fault || dut.b_fault) begin
            $display(
                "[FAIL] Baseline health A=%0d/%b B=%0d/%b",
                dut.a_fail_count, dut.a_fault,
                dut.b_fail_count, dut.b_fault
            );
            error_count = error_count + 1;
        end

        load_a_bad_b_good_seq11;
        start_both_sources;

        @(posedge dut.a_crc_error);
        @(posedge clk);
        #1;
        if ((dut.a_fail_count !== 8'd1) || dut.a_fault) begin
            $display(
                "[FAIL] A first CRC event expected count=1 fault=0, actual=%0d/%b",
                dut.a_fail_count, dut.a_fault
            );
            error_count = error_count + 1;
        end
        else begin
            $display(
                "[PASS] A first CRC event count=1 fault=0"
            );
        end

        @(posedge dut.matcher_result_valid);
        if (dut.matcher_result_kind !== 2'b11) begin
            $display(
                "[FAIL] A-bad transaction expected Single B, actual kind=%b",
                dut.matcher_result_kind
            );
            error_count = error_count + 1;
        end

        @(posedge clk);
        #1;
        if ((dut.a_fail_count !== 8'd2) || dut.a_fault) begin
            $display(
                "[FAIL] A first transaction expected count=2 fault=0 after Pair Timeout, actual=%0d/%b",
                dut.a_fail_count, dut.a_fault
            );
            error_count = error_count + 1;
        end
        else begin
            $display(
                "[PASS] A CRC event + Pair-missing event count=2"
            );
        end

        load_a_bad_b_good_seq12;
        start_both_sources;

        @(posedge dut.a_crc_error);
        @(posedge clk);
        #1;
        if ((dut.a_fail_count !== 8'd3) || !dut.a_fault) begin
            $display(
                "[FAIL] A second CRC event expected count=3 fault=1, actual=%0d/%b",
                dut.a_fail_count, dut.a_fault
            );
            error_count = error_count + 1;
        end
        else begin
            $display(
                "[PASS] A second CRC event reaches threshold count=3 fault=1"
            );
        end

        // -------------------------------------------------------------
        // Channel B: CRC 오류 + 정상 A
        // -------------------------------------------------------------
        reset_and_configure;

        load_normal_pair_seq10;
        start_both_sources;
        wait_normal_pair;

        load_a_good_b_bad_seq11;
        start_both_sources;

        @(posedge dut.b_crc_error);
        @(posedge clk);
        #1;
        if ((dut.b_fail_count !== 8'd1) || dut.b_fault) begin
            $display(
                "[FAIL] B first CRC event expected count=1 fault=0, actual=%0d/%b",
                dut.b_fail_count, dut.b_fault
            );
            error_count = error_count + 1;
        end
        else begin
            $display(
                "[PASS] B first CRC event count=1 fault=0"
            );
        end

        @(posedge dut.matcher_result_valid);
        if (dut.matcher_result_kind !== 2'b10) begin
            $display(
                "[FAIL] B-bad transaction expected Single A, actual kind=%b",
                dut.matcher_result_kind
            );
            error_count = error_count + 1;
        end

        @(posedge clk);
        #1;
        if ((dut.b_fail_count !== 8'd2) || dut.b_fault) begin
            $display(
                "[FAIL] B first transaction expected count=2 fault=0 after Pair Timeout, actual=%0d/%b",
                dut.b_fail_count, dut.b_fault
            );
            error_count = error_count + 1;
        end
        else begin
            $display(
                "[PASS] B CRC event + Pair-missing event count=2"
            );
        end

        load_a_good_b_bad_seq12;
        start_both_sources;

        @(posedge dut.b_crc_error);
        @(posedge clk);
        #1;
        if ((dut.b_fail_count !== 8'd3) || !dut.b_fault) begin
            $display(
                "[FAIL] B second CRC event expected count=3 fault=1, actual=%0d/%b",
                dut.b_fail_count, dut.b_fault
            );
            error_count = error_count + 1;
        end
        else begin
            $display(
                "[PASS] B second CRC event reaches threshold count=3 fault=1"
            );
        end

        if (error_count == 0)
            $display(
                "[PASS] Fail Event counting policy matches the current RTL."
            );
        else
            $display(
                "[FAIL] Fail Event count test found %0d error(s).",
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
