`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// tb_axi_lite_regs
//
// Self-checking 항목
//   1. Reset Default와 AXI Idle 출력
//   2. 모든 RW Register의 Default Readback
//   3. 일반 AXI Write/Read와 Byte Strobe
//   4. AW 먼저 도착, W 먼저 도착하는 독립 Channel 처리
//   5. BVALID/RVALID Backpressure와 출력 안정성
//   6. CONTROL 및 EVENT_FIFO_CONTROL의 1클럭 Pulse
//   7. STATUS와 EVENT_FIFO_STATUS Bit 배치
//   8. Event FIFO LOW/HIGH 원자적 Read 후 명시적 Pop
//   9. IRQ Enable, Level IRQ, Sticky IRQ, W1C, Set 우선순위
//  10. Read-only Write, 비정렬/미지원 주소의 SLVERR
//  11. 응답 대기 중 Reset 복귀
//////////////////////////////////////////////////////////////////////////////////

module tb_axi_lite_regs;

    localparam integer ADDR_WIDTH = 7;
    localparam integer DATA_WIDTH = 32;

    localparam [6:0] ADDR_CONTROL            = 7'h00;
    localparam [6:0] ADDR_STATUS             = 7'h04;
    localparam [6:0] ADDR_FAILOVER_CONFIG    = 7'h08;
    localparam [6:0] ADDR_PAIR_WAIT_TIMEOUT  = 7'h0C;
    localparam [6:0] ADDR_CHANNEL_TIMEOUT    = 7'h10;
    localparam [6:0] ADDR_OUTPUT_DEVICE_ID   = 7'h14;
    localparam [6:0] ADDR_COMMAND_MAP_0      = 7'h18;
    localparam [6:0] ADDR_COMMAND_MAP_1      = 7'h1C;
    localparam [6:0] ADDR_COMMAND_MAP_2      = 7'h20;
    localparam [6:0] ADDR_COMMAND_MAP_3      = 7'h24;
    localparam [6:0] ADDR_IRQ_ENABLE         = 7'h28;
    localparam [6:0] ADDR_IRQ_STATUS         = 7'h2C;
    localparam [6:0] ADDR_EVENT_FIFO_STATUS  = 7'h30;
    localparam [6:0] ADDR_EVENT_DATA_LOW     = 7'h34;
    localparam [6:0] ADDR_EVENT_DATA_HIGH    = 7'h38;
    localparam [6:0] ADDR_EVENT_FIFO_CONTROL = 7'h3C;
    localparam [6:0] ADDR_EVENT_LOST_COUNT   = 7'h40;

    localparam [1:0] AXI_RESP_OKAY   = 2'b00;
    localparam [1:0] AXI_RESP_SLVERR = 2'b10;

    reg clk;
    reg reset_p;

    reg  [ADDR_WIDTH-1:0] s_axil_awaddr;
    reg                   s_axil_awvalid;
    wire                  s_axil_awready;

    reg  [DATA_WIDTH-1:0] s_axil_wdata;
    reg  [3:0]            s_axil_wstrb;
    reg                   s_axil_wvalid;
    wire                  s_axil_wready;

    wire [1:0]            s_axil_bresp;
    wire                  s_axil_bvalid;
    reg                   s_axil_bready;

    reg  [ADDR_WIDTH-1:0] s_axil_araddr;
    reg                   s_axil_arvalid;
    wire                  s_axil_arready;

    wire [DATA_WIDTH-1:0] s_axil_rdata;
    wire [1:0]            s_axil_rresp;
    wire                  s_axil_rvalid;
    reg                   s_axil_rready;

    wire                  system_enable;
    wire                  preferred_channel_b;
    wire [7:0]            fail_threshold;
    wire [7:0]            recovery_count;
    wire [31:0]           pair_wait_timeout_cycles;
    wire [31:0]           channel_timeout_cycles;
    wire [7:0]            output_device_id;
    wire [7:0]            command_map_0;
    wire [7:0]            command_map_1;
    wire [7:0]            command_map_2;
    wire [7:0]            command_map_3;

    wire                  statistics_clear_pulse;
    wire                  fifo_clear_pulse;
    wire                  fifo_pop_request;

    reg                   channel_a_alive;
    reg                   channel_b_alive;
    reg                   last_selected_b;
    reg                   pair_wait_active;
    reg                   output_busy;
    reg                   frame_mismatch_latched;
    reg                   both_invalid_latched;

    reg                   event_front_valid;
    reg  [63:0]           event_front_data;
    reg                   event_fifo_empty;
    reg                   event_fifo_full;
    reg  [7:0]            event_fifo_count;
    reg  [15:0]           event_fifo_underflow_count;
    reg  [15:0]           event_lost_count;

    reg                   event_lost_pulse;
    reg                   frame_mismatch_event;
    reg                   both_invalid_event;
    reg                   channel_fault_event;
    reg                   fifo_underflow_pulse;

    wire                  irq;

    integer error_count;

    axi_lite_regs #(
        .C_S_AXI_ADDR_WIDTH (ADDR_WIDTH),
        .C_S_AXI_DATA_WIDTH (DATA_WIDTH)
    ) dut (
        .clk                        (clk),
        .reset_p                    (reset_p),
        .s_axil_awaddr              (s_axil_awaddr),
        .s_axil_awvalid             (s_axil_awvalid),
        .s_axil_awready             (s_axil_awready),
        .s_axil_wdata               (s_axil_wdata),
        .s_axil_wstrb               (s_axil_wstrb),
        .s_axil_wvalid              (s_axil_wvalid),
        .s_axil_wready              (s_axil_wready),
        .s_axil_bresp               (s_axil_bresp),
        .s_axil_bvalid              (s_axil_bvalid),
        .s_axil_bready              (s_axil_bready),
        .s_axil_araddr              (s_axil_araddr),
        .s_axil_arvalid             (s_axil_arvalid),
        .s_axil_arready             (s_axil_arready),
        .s_axil_rdata               (s_axil_rdata),
        .s_axil_rresp               (s_axil_rresp),
        .s_axil_rvalid              (s_axil_rvalid),
        .s_axil_rready              (s_axil_rready),
        .system_enable              (system_enable),
        .preferred_channel_b        (preferred_channel_b),
        .fail_threshold             (fail_threshold),
        .recovery_count             (recovery_count),
        .pair_wait_timeout_cycles   (pair_wait_timeout_cycles),
        .channel_timeout_cycles     (channel_timeout_cycles),
        .output_device_id           (output_device_id),
        .command_map_0              (command_map_0),
        .command_map_1              (command_map_1),
        .command_map_2              (command_map_2),
        .command_map_3              (command_map_3),
        .statistics_clear_pulse     (statistics_clear_pulse),
        .fifo_clear_pulse           (fifo_clear_pulse),
        .fifo_pop_request           (fifo_pop_request),
        .channel_a_alive            (channel_a_alive),
        .channel_b_alive            (channel_b_alive),
        .last_selected_b            (last_selected_b),
        .pair_wait_active           (pair_wait_active),
        .output_busy                (output_busy),
        .frame_mismatch_latched     (frame_mismatch_latched),
        .both_invalid_latched       (both_invalid_latched),
        .event_front_valid          (event_front_valid),
        .event_front_data           (event_front_data),
        .event_fifo_empty           (event_fifo_empty),
        .event_fifo_full            (event_fifo_full),
        .event_fifo_count           (event_fifo_count),
        .event_fifo_underflow_count (event_fifo_underflow_count),
        .event_lost_count           (event_lost_count),
        .event_lost_pulse           (event_lost_pulse),
        .frame_mismatch_event       (frame_mismatch_event),
        .both_invalid_event         (both_invalid_event),
        .channel_fault_event        (channel_fault_event),
        .fifo_underflow_pulse       (fifo_underflow_pulse),
        .irq                        (irq)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    task check_bit;
        input actual;
        input expected;
        input [8*100-1:0] check_name;
        begin
            if (actual !== expected) begin
                $display("[FAIL] %0s: expected=%b actual=%b",
                         check_name, expected, actual);
                error_count = error_count + 1;
            end
        end
    endtask

    task check_u8;
        input [7:0] actual;
        input [7:0] expected;
        input [8*100-1:0] check_name;
        begin
            if (actual !== expected) begin
                $display("[FAIL] %0s: expected=%02h actual=%02h",
                         check_name, expected, actual);
                error_count = error_count + 1;
            end
        end
    endtask

    task check_u32;
        input [31:0] actual;
        input [31:0] expected;
        input [8*100-1:0] check_name;
        begin
            if (actual !== expected) begin
                $display("[FAIL] %0s: expected=%08h actual=%08h",
                         check_name, expected, actual);
                error_count = error_count + 1;
            end
        end
    endtask

    task check_resp;
        input [1:0] actual;
        input [1:0] expected;
        input [8*100-1:0] check_name;
        begin
            if (actual !== expected) begin
                $display("[FAIL] %0s: expected=%b actual=%b",
                         check_name, expected, actual);
                error_count = error_count + 1;
            end
        end
    endtask

    task apply_reset;
        begin
            reset_p = 1'b1;
            repeat (2) @(negedge clk);
            reset_p = 1'b0;
            @(negedge clk);
        end
    endtask

    // Address와 Data를 같은 Transaction에서 전달한다.
    task axi_write;
        input [ADDR_WIDTH-1:0] address;
        input [31:0] data;
        input [3:0] strobe;
        input [1:0] expected_response;
        begin
            @(negedge clk);
            s_axil_awaddr  = address;
            s_axil_awvalid = 1'b1;
            s_axil_wdata   = data;
            s_axil_wstrb   = strobe;
            s_axil_wvalid  = 1'b1;

            check_bit(s_axil_awready, 1'b1,
                      "axi_write AWREADY before handshake");
            check_bit(s_axil_wready, 1'b1,
                      "axi_write WREADY before handshake");

            @(posedge clk);
            #1;

            check_bit(s_axil_bvalid, 1'b1,
                      "axi_write BVALID after commit");
            check_resp(s_axil_bresp, expected_response,
                       "axi_write BRESP");

            @(negedge clk);
            s_axil_awvalid = 1'b0;
            s_axil_wvalid  = 1'b0;
            s_axil_bready  = 1'b1;

            @(posedge clk);
            #1;

            @(negedge clk);
            s_axil_bready = 1'b0;

            check_bit(s_axil_bvalid, 1'b0,
                      "axi_write BVALID cleared");
        end
    endtask

    // Address를 전달하고 Read Response를 검사한다.
    task axi_read;
        input [ADDR_WIDTH-1:0] address;
        input [31:0] expected_data;
        input [1:0] expected_response;
        begin
            @(negedge clk);
            s_axil_araddr  = address;
            s_axil_arvalid = 1'b1;

            check_bit(s_axil_arready, 1'b1,
                      "axi_read ARREADY before handshake");

            @(posedge clk);
            #1;

            check_bit(s_axil_rvalid, 1'b1,
                      "axi_read RVALID after address");
            check_resp(s_axil_rresp, expected_response,
                       "axi_read RRESP");
            check_u32(s_axil_rdata, expected_data,
                      "axi_read RDATA");

            @(negedge clk);
            s_axil_arvalid = 1'b0;
            s_axil_rready  = 1'b1;

            @(posedge clk);
            #1;

            @(negedge clk);
            s_axil_rready = 1'b0;

            check_bit(s_axil_rvalid, 1'b0,
                      "axi_read RVALID cleared");
        end
    endtask

    initial begin
        reset_p        = 1'b1;

        s_axil_awaddr  = {ADDR_WIDTH{1'b0}};
        s_axil_awvalid = 1'b0;
        s_axil_wdata   = 32'd0;
        s_axil_wstrb   = 4'd0;
        s_axil_wvalid  = 1'b0;
        s_axil_bready  = 1'b0;
        s_axil_araddr  = {ADDR_WIDTH{1'b0}};
        s_axil_arvalid = 1'b0;
        s_axil_rready  = 1'b0;

        channel_a_alive            = 1'b0;
        channel_b_alive            = 1'b0;
        last_selected_b            = 1'b0;
        pair_wait_active           = 1'b0;
        output_busy                = 1'b0;
        frame_mismatch_latched     = 1'b0;
        both_invalid_latched       = 1'b0;

        event_front_valid          = 1'b0;
        event_front_data           = 64'd0;
        event_fifo_empty           = 1'b1;
        event_fifo_full            = 1'b0;
        event_fifo_count           = 8'd0;
        event_fifo_underflow_count = 16'd0;
        event_lost_count           = 16'd0;

        event_lost_pulse           = 1'b0;
        frame_mismatch_event       = 1'b0;
        both_invalid_event         = 1'b0;
        channel_fault_event        = 1'b0;
        fifo_underflow_pulse       = 1'b0;

        error_count = 0;

        apply_reset;

        // ---------------------------------------------------------------------
        // TEST 1: Reset Default 및 AXI Idle
        // ---------------------------------------------------------------------
        check_bit(s_axil_awready, 1'b1, "reset AWREADY");
        check_bit(s_axil_wready, 1'b1, "reset WREADY");
        check_bit(s_axil_bvalid, 1'b0, "reset BVALID");
        check_bit(s_axil_arready, 1'b1, "reset ARREADY");
        check_bit(s_axil_rvalid, 1'b0, "reset RVALID");

        check_bit(system_enable, 1'b0, "reset system_enable");
        check_bit(preferred_channel_b, 1'b0,
                  "reset preferred channel A");
        check_u8(fail_threshold, 8'd3, "reset fail threshold");
        check_u8(recovery_count, 8'd5, "reset recovery count");
        check_u32(pair_wait_timeout_cycles, 32'd1_000_000,
                  "reset pair timeout");
        check_u32(channel_timeout_cycles, 32'd30_000_000,
                  "reset channel timeout");
        check_u8(output_device_id, 8'h00, "reset output device id");
        check_u8(command_map_0, 8'h10, "reset command map 0");
        check_u8(command_map_1, 8'h11, "reset command map 1");
        check_u8(command_map_2, 8'h12, "reset command map 2");
        check_u8(command_map_3, 8'h13, "reset command map 3");
        check_bit(statistics_clear_pulse, 1'b0,
                  "reset statistics pulse");
        check_bit(fifo_clear_pulse, 1'b0, "reset fifo clear pulse");
        check_bit(fifo_pop_request, 1'b0, "reset fifo pop pulse");
        check_bit(irq, 1'b0, "reset irq");

        // ---------------------------------------------------------------------
        // TEST 2: Default Register Readback
        // ---------------------------------------------------------------------
        axi_read(ADDR_CONTROL, 32'h0000_0000, AXI_RESP_OKAY);
        axi_read(ADDR_FAILOVER_CONFIG, 32'h0005_0300,
                 AXI_RESP_OKAY);
        axi_read(ADDR_PAIR_WAIT_TIMEOUT, 32'd1_000_000,
                 AXI_RESP_OKAY);
        axi_read(ADDR_CHANNEL_TIMEOUT, 32'd30_000_000,
                 AXI_RESP_OKAY);
        axi_read(ADDR_OUTPUT_DEVICE_ID, 32'h0000_0000,
                 AXI_RESP_OKAY);
        axi_read(ADDR_COMMAND_MAP_0, 32'h0000_0010,
                 AXI_RESP_OKAY);
        axi_read(ADDR_COMMAND_MAP_1, 32'h0000_0011,
                 AXI_RESP_OKAY);
        axi_read(ADDR_COMMAND_MAP_2, 32'h0000_0012,
                 AXI_RESP_OKAY);
        axi_read(ADDR_COMMAND_MAP_3, 32'h0000_0013,
                 AXI_RESP_OKAY);

        // ---------------------------------------------------------------------
        // TEST 3: 일반 Write/Read 및 Byte Strobe
        // ---------------------------------------------------------------------
        axi_write(ADDR_FAILOVER_CONFIG, 32'h0007_0401,
                  4'b1111, AXI_RESP_OKAY);

        check_bit(preferred_channel_b, 1'b1,
                  "failover write preferred B");
        check_u8(fail_threshold, 8'd4,
                 "failover write fail threshold");
        check_u8(recovery_count, 8'd7,
                 "failover write recovery count");
        axi_read(ADDR_FAILOVER_CONFIG, 32'h0007_0401,
                 AXI_RESP_OKAY);

        axi_write(ADDR_PAIR_WAIT_TIMEOUT, 32'h1122_3344,
                  4'b1111, AXI_RESP_OKAY);
        axi_write(ADDR_PAIR_WAIT_TIMEOUT, 32'hAABB_CCDD,
                  4'b0101, AXI_RESP_OKAY);
        check_u32(pair_wait_timeout_cycles, 32'h11BB_33DD,
                  "pair timeout byte strobe merge");
        axi_read(ADDR_PAIR_WAIT_TIMEOUT, 32'h11BB_33DD,
                 AXI_RESP_OKAY);

        axi_write(ADDR_CHANNEL_TIMEOUT, 32'hCAFE_BABE,
                  4'b1111, AXI_RESP_OKAY);
        axi_write(ADDR_OUTPUT_DEVICE_ID, 32'h0000_005A,
                  4'b0001, AXI_RESP_OKAY);
        axi_write(ADDR_COMMAND_MAP_0, 32'h0000_00A0,
                  4'b0001, AXI_RESP_OKAY);
        axi_write(ADDR_COMMAND_MAP_1, 32'h0000_00A1,
                  4'b0001, AXI_RESP_OKAY);
        axi_write(ADDR_COMMAND_MAP_2, 32'h0000_00A2,
                  4'b0001, AXI_RESP_OKAY);
        axi_write(ADDR_COMMAND_MAP_3, 32'h0000_00A3,
                  4'b0001, AXI_RESP_OKAY);

        check_u32(channel_timeout_cycles, 32'hCAFE_BABE,
                  "channel timeout write");
        check_u8(output_device_id, 8'h5A, "output device id write");
        check_u8(command_map_0, 8'hA0, "command map 0 write");
        check_u8(command_map_1, 8'hA1, "command map 1 write");
        check_u8(command_map_2, 8'hA2, "command map 2 write");
        check_u8(command_map_3, 8'hA3, "command map 3 write");

        // ---------------------------------------------------------------------
        // TEST 4: CONTROL Pulse는 Commit 클럭에만 1
        // ---------------------------------------------------------------------
        @(negedge clk);
        s_axil_awaddr  = ADDR_CONTROL;
        s_axil_awvalid = 1'b1;
        s_axil_wdata   = 32'h0000_0007;
        s_axil_wstrb   = 4'b0001;
        s_axil_wvalid  = 1'b1;

        @(posedge clk);
        #1;

        check_bit(system_enable, 1'b1, "control enable set");
        check_bit(statistics_clear_pulse, 1'b1,
                  "control statistics pulse asserted");
        check_bit(fifo_clear_pulse, 1'b1,
                  "control fifo clear pulse asserted");
        check_bit(fifo_pop_request, 1'b0,
                  "control does not pop fifo");
        check_bit(s_axil_bvalid, 1'b1,
                  "control write response valid");

        @(negedge clk);
        s_axil_awvalid = 1'b0;
        s_axil_wvalid  = 1'b0;
        s_axil_bready  = 1'b1;

        @(posedge clk);
        #1;

        check_bit(statistics_clear_pulse, 1'b0,
                  "statistics pulse one clock");
        check_bit(fifo_clear_pulse, 1'b0,
                  "fifo clear pulse one clock");

        @(negedge clk);
        s_axil_bready = 1'b0;

        // Strobe가 없으면 Enable/Pulse 모두 바뀌지 않는다.
        axi_write(ADDR_CONTROL, 32'h0000_0000,
                  4'b0000, AXI_RESP_OKAY);
        check_bit(system_enable, 1'b1,
                  "control zero strobe preserves enable");

        // ---------------------------------------------------------------------
        // TEST 5: AW가 W보다 먼저 도착
        // ---------------------------------------------------------------------
        @(negedge clk);
        s_axil_awaddr  = ADDR_OUTPUT_DEVICE_ID;
        s_axil_awvalid = 1'b1;

        @(posedge clk);
        #1;

        @(negedge clk);
        s_axil_awvalid = 1'b0;

        repeat (2) begin
            @(posedge clk);
            #1;
            check_bit(s_axil_bvalid, 1'b0,
                      "AW first no response without W");
            check_u8(output_device_id, 8'h5A,
                     "AW first no early register update");
        end

        @(negedge clk);
        s_axil_wdata  = 32'h0000_0066;
        s_axil_wstrb  = 4'b0001;
        s_axil_wvalid = 1'b1;

        @(posedge clk);
        #1;

        check_bit(s_axil_bvalid, 1'b1,
                  "AW first response after W");
        check_resp(s_axil_bresp, AXI_RESP_OKAY,
                   "AW first response OKAY");
        check_u8(output_device_id, 8'h66,
                 "AW first register update");

        @(negedge clk);
        s_axil_wvalid = 1'b0;
        s_axil_bready = 1'b1;

        @(posedge clk);
        #1;

        @(negedge clk);
        s_axil_bready = 1'b0;

        // ---------------------------------------------------------------------
        // TEST 6: W가 AW보다 먼저 도착
        // ---------------------------------------------------------------------
        @(negedge clk);
        s_axil_wdata  = 32'h0000_0077;
        s_axil_wstrb  = 4'b0001;
        s_axil_wvalid = 1'b1;

        @(posedge clk);
        #1;

        @(negedge clk);
        s_axil_wvalid = 1'b0;

        repeat (2) begin
            @(posedge clk);
            #1;
            check_bit(s_axil_bvalid, 1'b0,
                      "W first no response without AW");
            check_u8(output_device_id, 8'h66,
                     "W first no early register update");
        end

        @(negedge clk);
        s_axil_awaddr  = ADDR_OUTPUT_DEVICE_ID;
        s_axil_awvalid = 1'b1;

        @(posedge clk);
        #1;

        check_bit(s_axil_bvalid, 1'b1,
                  "W first response after AW");
        check_resp(s_axil_bresp, AXI_RESP_OKAY,
                   "W first response OKAY");
        check_u8(output_device_id, 8'h77,
                 "W first register update");

        @(negedge clk);
        s_axil_awvalid = 1'b0;
        s_axil_bready  = 1'b1;

        @(posedge clk);
        #1;

        @(negedge clk);
        s_axil_bready = 1'b0;

        // ---------------------------------------------------------------------
        // TEST 7: BVALID Backpressure
        // ---------------------------------------------------------------------
        @(negedge clk);
        s_axil_awaddr  = ADDR_OUTPUT_DEVICE_ID;
        s_axil_awvalid = 1'b1;
        s_axil_wdata   = 32'h0000_0088;
        s_axil_wstrb   = 4'b0001;
        s_axil_wvalid  = 1'b1;

        @(posedge clk);
        #1;

        @(negedge clk);
        s_axil_awvalid = 1'b0;
        s_axil_wvalid  = 1'b0;

        repeat (3) begin
            @(posedge clk);
            #1;
            check_bit(s_axil_bvalid, 1'b1,
                      "BVALID held under backpressure");
            check_resp(s_axil_bresp, AXI_RESP_OKAY,
                       "BRESP held under backpressure");
            check_bit(s_axil_awready, 1'b0,
                      "AWREADY blocked by pending response");
            check_bit(s_axil_wready, 1'b0,
                      "WREADY blocked by pending response");
        end

        @(negedge clk);
        s_axil_bready = 1'b1;

        @(posedge clk);
        #1;

        @(negedge clk);
        s_axil_bready = 1'b0;

        check_bit(s_axil_bvalid, 1'b0,
                  "BVALID clears after BREADY");
        check_bit(s_axil_awready, 1'b1,
                  "AWREADY returns after response");
        check_bit(s_axil_wready, 1'b1,
                  "WREADY returns after response");

        // ---------------------------------------------------------------------
        // TEST 8: STATUS/FIFO STATUS/Data Read
        // ---------------------------------------------------------------------
        channel_a_alive            = 1'b1;
        channel_b_alive            = 1'b0;
        last_selected_b            = 1'b1;
        pair_wait_active           = 1'b1;
        output_busy                = 1'b0;
        frame_mismatch_latched     = 1'b1;
        both_invalid_latched       = 1'b0;

        event_front_valid          = 1'b1;
        event_front_data           = 64'h1122_3344_5566_7788;
        event_fifo_empty           = 1'b0;
        event_fifo_full            = 1'b0;
        event_fifo_count           = 8'd3;
        event_fifo_underflow_count = 16'd2;
        event_lost_count           = 16'h1234;

        axi_read(ADDR_STATUS, 32'h0000_01BB, AXI_RESP_OKAY);
        axi_read(ADDR_EVENT_FIFO_STATUS, 32'h0002_0304,
                 AXI_RESP_OKAY);
        axi_read(ADDR_EVENT_DATA_LOW, 32'h5566_7788,
                 AXI_RESP_OKAY);
        axi_read(ADDR_EVENT_DATA_HIGH, 32'h1122_3344,
                 AXI_RESP_OKAY);
        axi_read(ADDR_EVENT_LOST_COUNT, 32'h0000_1234,
                 AXI_RESP_OKAY);

        // LOW/HIGH Read만으로는 Pop Pulse가 발생하면 안 된다.
        check_bit(fifo_pop_request, 1'b0,
                  "event data reads do not pop");

        // EVENT FIFO Pop Pulse
        @(negedge clk);
        s_axil_awaddr  = ADDR_EVENT_FIFO_CONTROL;
        s_axil_awvalid = 1'b1;
        s_axil_wdata   = 32'h0000_0001;
        s_axil_wstrb   = 4'b0001;
        s_axil_wvalid  = 1'b1;

        @(posedge clk);
        #1;

        check_bit(fifo_pop_request, 1'b1,
                  "event fifo pop pulse asserted");
        check_bit(fifo_clear_pulse, 1'b0,
                  "event fifo pop does not clear");

        @(negedge clk);
        s_axil_awvalid = 1'b0;
        s_axil_wvalid  = 1'b0;
        s_axil_bready  = 1'b1;

        @(posedge clk);
        #1;

        check_bit(fifo_pop_request, 1'b0,
                  "event fifo pop pulse one clock");

        @(negedge clk);
        s_axil_bready = 1'b0;

        // Empty이면 Event Data는 0을 반환한다.
        event_front_valid = 1'b0;
        event_fifo_empty  = 1'b1;
        event_fifo_count  = 8'd0;

        axi_read(ADDR_EVENT_DATA_LOW, 32'd0, AXI_RESP_OKAY);
        axi_read(ADDR_EVENT_DATA_HIGH, 32'd0, AXI_RESP_OKAY);

        // ---------------------------------------------------------------------
        // TEST 9: RVALID Backpressure 및 Read Data 안정성
        // ---------------------------------------------------------------------
        @(negedge clk);
        s_axil_araddr  = ADDR_CHANNEL_TIMEOUT;
        s_axil_arvalid = 1'b1;

        @(posedge clk);
        #1;

        @(negedge clk);
        s_axil_arvalid = 1'b0;

        repeat (3) begin
            @(posedge clk);
            #1;
            check_bit(s_axil_rvalid, 1'b1,
                      "RVALID held under backpressure");
            check_resp(s_axil_rresp, AXI_RESP_OKAY,
                       "RRESP held under backpressure");
            check_u32(s_axil_rdata, 32'hCAFE_BABE,
                      "RDATA held under backpressure");
            check_bit(s_axil_arready, 1'b0,
                      "ARREADY blocked by pending read");
        end

        @(negedge clk);
        s_axil_rready = 1'b1;

        @(posedge clk);
        #1;

        @(negedge clk);
        s_axil_rready = 1'b0;

        check_bit(s_axil_rvalid, 1'b0,
                  "RVALID clears after RREADY");

        // ---------------------------------------------------------------------
        // TEST 10: IRQ Level/Sticky/W1C/Set 우선순위
        // ---------------------------------------------------------------------
        axi_write(ADDR_IRQ_ENABLE, 32'h0000_003F,
                  4'b0001, AXI_RESP_OKAY);
        axi_read(ADDR_IRQ_ENABLE, 32'h0000_003F,
                 AXI_RESP_OKAY);

        // FIFO Not Empty는 Level IRQ이다.
        event_fifo_empty = 1'b0;
        #1;
        check_bit(irq, 1'b1, "fifo not empty level irq");
        axi_read(ADDR_IRQ_STATUS, 32'h0000_0001,
                 AXI_RESP_OKAY);

        event_fifo_empty = 1'b1;
        #1;
        check_bit(irq, 1'b0, "fifo empty clears level irq");
        axi_read(ADDR_IRQ_STATUS, 32'h0000_0000,
                 AXI_RESP_OKAY);

        // Sticky Event 5종 동시 발생
        @(negedge clk);
        event_lost_pulse     = 1'b1;
        frame_mismatch_event = 1'b1;
        both_invalid_event   = 1'b1;
        channel_fault_event  = 1'b1;
        fifo_underflow_pulse = 1'b1;

        @(posedge clk);
        #1;

        @(negedge clk);
        event_lost_pulse     = 1'b0;
        frame_mismatch_event = 1'b0;
        both_invalid_event   = 1'b0;
        channel_fault_event  = 1'b0;
        fifo_underflow_pulse = 1'b0;

        check_bit(irq, 1'b1, "sticky event irq asserted");
        axi_read(ADDR_IRQ_STATUS, 32'h0000_003E,
                 AXI_RESP_OKAY);

        // 모든 Sticky Bit Clear와 동시에 EVENT_LOST 재발생:
        // Clear보다 새 Event Set이 우선하므로 Bit 1만 남아야 한다.
        @(negedge clk);
        s_axil_awaddr    = ADDR_IRQ_STATUS;
        s_axil_awvalid   = 1'b1;
        s_axil_wdata     = 32'h0000_003E;
        s_axil_wstrb     = 4'b0001;
        s_axil_wvalid    = 1'b1;
        event_lost_pulse = 1'b1;

        @(posedge clk);
        #1;

        @(negedge clk);
        s_axil_awvalid   = 1'b0;
        s_axil_wvalid    = 1'b0;
        s_axil_bready    = 1'b1;
        event_lost_pulse = 1'b0;

        @(posedge clk);
        #1;

        @(negedge clk);
        s_axil_bready = 1'b0;

        axi_read(ADDR_IRQ_STATUS, 32'h0000_0002,
                 AXI_RESP_OKAY);

        // 남은 Bit 1 W1C
        axi_write(ADDR_IRQ_STATUS, 32'h0000_0002,
                  4'b0001, AXI_RESP_OKAY);
        axi_read(ADDR_IRQ_STATUS, 32'h0000_0000,
                 AXI_RESP_OKAY);
        check_bit(irq, 1'b0, "irq clears after W1C");

        // Byte Strobe가 없으면 W1C가 적용되지 않는다.
        @(negedge clk);
        channel_fault_event = 1'b1;

        @(posedge clk);
        #1;

        @(negedge clk);
        channel_fault_event = 1'b0;

        axi_write(ADDR_IRQ_STATUS, 32'h0000_0010,
                  4'b0000, AXI_RESP_OKAY);
        axi_read(ADDR_IRQ_STATUS, 32'h0000_0010,
                 AXI_RESP_OKAY);
        axi_write(ADDR_IRQ_STATUS, 32'h0000_0010,
                  4'b0001, AXI_RESP_OKAY);

        // ---------------------------------------------------------------------
        // TEST 11: SLVERR와 Register 무변경
        // ---------------------------------------------------------------------
        axi_write(ADDR_STATUS, 32'hFFFF_FFFF,
                  4'b1111, AXI_RESP_SLVERR);
        check_bit(system_enable, 1'b1,
                  "RO write does not change control");

        axi_write(7'h42, 32'h1234_5678,
                  4'b1111, AXI_RESP_SLVERR);
        axi_write(7'h44, 32'h1234_5678,
                  4'b1111, AXI_RESP_SLVERR);

        axi_read(7'h01, 32'd0, AXI_RESP_SLVERR);
        axi_read(7'h44, 32'd0, AXI_RESP_SLVERR);

        // ---------------------------------------------------------------------
        // TEST 12: 응답 대기 중 Reset
        // ---------------------------------------------------------------------
        @(negedge clk);
        s_axil_awaddr  = ADDR_CONTROL;
        s_axil_awvalid = 1'b1;
        s_axil_wdata   = 32'h0000_0001;
        s_axil_wstrb   = 4'b0001;
        s_axil_wvalid  = 1'b1;
        s_axil_araddr  = ADDR_STATUS;
        s_axil_arvalid = 1'b1;

        @(posedge clk);
        #1;

        check_bit(s_axil_bvalid, 1'b1,
                  "pre-reset write response pending");
        check_bit(s_axil_rvalid, 1'b1,
                  "pre-reset read response pending");

        reset_p = 1'b1;
        #1;

        check_bit(s_axil_bvalid, 1'b0,
                  "async reset clears BVALID");
        check_bit(s_axil_rvalid, 1'b0,
                  "async reset clears RVALID");
        check_bit(system_enable, 1'b0,
                  "async reset restores system enable");
        check_bit(irq, 1'b0, "async reset clears irq");

        @(negedge clk);
        s_axil_awvalid = 1'b0;
        s_axil_wvalid  = 1'b0;
        s_axil_arvalid = 1'b0;
        reset_p        = 1'b0;

        @(negedge clk);

        check_u8(fail_threshold, 8'd3,
                 "final reset fail threshold");
        check_u8(recovery_count, 8'd5,
                 "final reset recovery count");
        check_u32(pair_wait_timeout_cycles, 32'd1_000_000,
                  "final reset pair timeout");
        check_u32(channel_timeout_cycles, 32'd30_000_000,
                  "final reset channel timeout");

        // ---------------------------------------------------------------------
        // 결과
        // ---------------------------------------------------------------------
        if (error_count == 0)
            $display("[PASS] All axi_lite_regs core tests passed.");
        else
            $display("[FAIL] axi_lite_regs tests failed: %0d error(s).",
                     error_count);

        #20;
        $finish;
    end

    initial begin
        #100000;
        $display("[FAIL] Simulation timeout.");
        $finish;
    end

endmodule