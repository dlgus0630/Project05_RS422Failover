`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// axi_lite_regs
//
// 역할
//   MicroBlaze의 AXI4-Lite 접근을 이중화 통신 Core의 설정, 상태,
//   Interrupt, Event FIFO 제어 신호로 변환한다.
//
// AXI4-Lite 규칙
//   - 32-bit Data, Byte Address 방식이다.
//   - Write Address와 Write Data는 서로 독립적으로 도착할 수 있다.
//   - AW/W를 각각 1개씩 보관한 뒤 둘 다 확보된 시점에 Write를 수행한다.
//   - BVALID/RVALID은 Master가 READY로 수락할 때까지 유지한다.
//   - 지원하지 않는 주소, 비정렬 주소, Read-only Register Write는
//     SLVERR(2'b10)를 반환한다.
//
// Register Map
//   0x00 CONTROL
//        [0] SYSTEM_ENABLE       RW
//        [1] STATISTICS_CLEAR   W1P
//        [2] EVENT_FIFO_CLEAR   W1P
//
//   0x04 STATUS                 RO
//        [0] SYSTEM_ENABLE
//        [1] CHANNEL_A_ALIVE
//        [2] CHANNEL_B_ALIVE
//        [3] PREFERRED_CHANNEL  0:A, 1:B
//        [4] LAST_SELECTED      0:A, 1:B
//        [5] PAIR_WAIT_ACTIVE
//        [6] OUTPUT_BUSY
//        [7] EVENT_FIFO_NOT_EMPTY
//        [8] FRAME_MISMATCH_LATCHED
//        [9] BOTH_INVALID_LATCHED
//
//   0x08 FAILOVER_CONFIG        RW
//        [0]     PREFERRED_CHANNEL  0:A, 1:B
//        [15:8]  FAIL_THRESHOLD
//        [23:16] RECOVERY_COUNT
//
//   0x0C PAIR_WAIT_TIMEOUT      RW, 100 MHz Clock Cycle 단위
//   0x10 CHANNEL_TIMEOUT        RW, 100 MHz Clock Cycle 단위
//   0x14 OUTPUT_DEVICE_ID       RW, 하위 8-bit
//   0x18 COMMAND_MAP_0          RW, 하위 8-bit
//   0x1C COMMAND_MAP_1          RW, 하위 8-bit
//   0x20 COMMAND_MAP_2          RW, 하위 8-bit
//   0x24 COMMAND_MAP_3          RW, 하위 8-bit
//
//   0x28 IRQ_ENABLE             RW
//        [0] EVENT_FIFO_NOT_EMPTY
//        [1] EVENT_LOST
//        [2] FRAME_MISMATCH
//        [3] BOTH_INVALID
//        [4] CHANNEL_FAULT
//        [5] EVENT_FIFO_UNDERFLOW
//
//   0x2C IRQ_STATUS             RO / W1C
//        [0] EVENT_FIFO_NOT_EMPTY: Level 상태, FIFO Empty 시 자동 Clear
//        [5:1] Event Pending: Sticky, 해당 Bit에 1을 쓰면 Clear
//        같은 클럭에 Event Set과 W1C가 겹치면 Event Set을 우선한다.
//
//   0x30 EVENT_FIFO_STATUS      RO
//        [0]     EMPTY
//        [1]     FULL
//        [2]     FRONT_VALID
//        [15:8]  EVENT_COUNT
//        [31:16] UNDERFLOW_COUNT
//
//   0x34 EVENT_DATA_LOW         RO, Front Event[31:0]
//   0x38 EVENT_DATA_HIGH        RO, Front Event[63:32]
//
//   0x3C EVENT_FIFO_CONTROL     WO
//        [0] POP                W1P
//        [1] CLEAR              W1P
//
//   0x40 EVENT_LOST_COUNT       RO, 하위 16-bit
//
// Reset Default
//   SYSTEM_ENABLE       = 0
//   PREFERRED_CHANNEL   = A
//   FAIL_THRESHOLD      = 3
//   RECOVERY_COUNT      = 5
//   PAIR_WAIT_TIMEOUT   = 1,000,000 Cycle  = 10 ms @ 100 MHz
//   CHANNEL_TIMEOUT     = 30,000,000 Cycle = 300 ms @ 100 MHz
//   OUTPUT_DEVICE_ID    = 0
//   COMMAND_MAP_0..3    = 0x10, 0x11, 0x12, 0x13
//   IRQ_ENABLE/STATUS   = 0
//
// 중요
//   EVENT_DATA_LOW/HIGH Read 자체는 FIFO를 Pop하지 않는다.
//   CPU는 LOW와 HIGH를 모두 읽은 뒤 EVENT_FIFO_CONTROL.POP에 1을 써야 한다.
//   statistics_clear_pulse와 fifo_clear_pulse는 외부 Counter/FIFO가 처리할
//   1클럭 Pulse이며, 이 Register Block이 외부 상태를 직접 Reset하지 않는다.
//////////////////////////////////////////////////////////////////////////////////

module axi_lite_regs #(
    parameter integer C_S_AXI_ADDR_WIDTH = 7,
    parameter integer C_S_AXI_DATA_WIDTH = 32
)(
    input  wire                              clk,
    input  wire                              reset_p,

    // AXI4-Lite Slave Write Address Channel
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]     s_axil_awaddr,
    input  wire                              s_axil_awvalid,
    output wire                              s_axil_awready,

    // AXI4-Lite Slave Write Data Channel
    input  wire [C_S_AXI_DATA_WIDTH-1:0]     s_axil_wdata,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0] s_axil_wstrb,
    input  wire                              s_axil_wvalid,
    output wire                              s_axil_wready,

    // AXI4-Lite Slave Write Response Channel
    output reg  [1:0]                        s_axil_bresp,
    output reg                               s_axil_bvalid,
    input  wire                              s_axil_bready,

    // AXI4-Lite Slave Read Address Channel
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]     s_axil_araddr,
    input  wire                              s_axil_arvalid,
    output wire                              s_axil_arready,

    // AXI4-Lite Slave Read Data Channel
    output reg  [C_S_AXI_DATA_WIDTH-1:0]     s_axil_rdata,
    output reg  [1:0]                        s_axil_rresp,
    output reg                               s_axil_rvalid,
    input  wire                              s_axil_rready,

    // Core 설정 출력
    output reg                               system_enable,
    output reg                               preferred_channel_b,
    output reg  [7:0]                        fail_threshold,
    output reg  [7:0]                        recovery_count,
    output reg  [31:0]                       pair_wait_timeout_cycles,
    output reg  [31:0]                       channel_timeout_cycles,
    output reg  [7:0]                        output_device_id,
    output reg  [7:0]                        command_map_0,
    output reg  [7:0]                        command_map_1,
    output reg  [7:0]                        command_map_2,
    output reg  [7:0]                        command_map_3,

    // Write-One-Pulse 제어 출력
    output reg                               statistics_clear_pulse,
    output reg                               fifo_clear_pulse,
    output reg                               fifo_pop_request,

    // Core 상태 입력
    input  wire                              channel_a_alive,
    input  wire                              channel_b_alive,
    input  wire                              last_selected_b,
    input  wire                              pair_wait_active,
    input  wire                              output_busy,
    input  wire                              frame_mismatch_latched,
    input  wire                              both_invalid_latched,

    // Event FIFO 상태/데이터 입력
    input  wire                              event_front_valid,
    input  wire [63:0]                       event_front_data,
    input  wire                              event_fifo_empty,
    input  wire                              event_fifo_full,
    input  wire [7:0]                        event_fifo_count,
    input  wire [15:0]                       event_fifo_underflow_count,
    input  wire [15:0]                       event_lost_count,

    // Interrupt Event 입력: 모두 1클럭 Pulse
    input  wire                              event_lost_pulse,
    input  wire                              frame_mismatch_event,
    input  wire                              both_invalid_event,
    input  wire                              channel_fault_event,
    input  wire                              fifo_underflow_pulse,

    output wire                              irq
);

    localparam [1:0] AXI_RESP_OKAY   = 2'b00;
    localparam [1:0] AXI_RESP_SLVERR = 2'b10;

    localparam integer ADDR_CONTROL            = 7'h00;
    localparam integer ADDR_STATUS             = 7'h04;
    localparam integer ADDR_FAILOVER_CONFIG    = 7'h08;
    localparam integer ADDR_PAIR_WAIT_TIMEOUT  = 7'h0C;
    localparam integer ADDR_CHANNEL_TIMEOUT    = 7'h10;
    localparam integer ADDR_OUTPUT_DEVICE_ID   = 7'h14;
    localparam integer ADDR_COMMAND_MAP_0      = 7'h18;
    localparam integer ADDR_COMMAND_MAP_1      = 7'h1C;
    localparam integer ADDR_COMMAND_MAP_2      = 7'h20;
    localparam integer ADDR_COMMAND_MAP_3      = 7'h24;
    localparam integer ADDR_IRQ_ENABLE         = 7'h28;
    localparam integer ADDR_IRQ_STATUS         = 7'h2C;
    localparam integer ADDR_EVENT_FIFO_STATUS  = 7'h30;
    localparam integer ADDR_EVENT_DATA_LOW     = 7'h34;
    localparam integer ADDR_EVENT_DATA_HIGH    = 7'h38;
    localparam integer ADDR_EVENT_FIFO_CONTROL = 7'h3C;
    localparam integer ADDR_EVENT_LOST_COUNT   = 7'h40;

    localparam [31:0] DEFAULT_PAIR_WAIT_TIMEOUT =
        32'd1_000_000;

    localparam [31:0] DEFAULT_CHANNEL_TIMEOUT =
        32'd30_000_000;

    // -------------------------------------------------------------------------
    // AXI Write Address/Data 독립 보관
    // -------------------------------------------------------------------------
    reg                               aw_hold_valid;
    reg [C_S_AXI_ADDR_WIDTH-1:0]      aw_hold_addr;

    reg                               w_hold_valid;
    reg [C_S_AXI_DATA_WIDTH-1:0]      w_hold_data;
    reg [(C_S_AXI_DATA_WIDTH/8)-1:0]  w_hold_strb;

    wire aw_accept;
    wire w_accept;
    wire write_have_addr;
    wire write_have_data;
    wire write_commit;

    wire [C_S_AXI_ADDR_WIDTH-1:0]      write_addr;
    wire [C_S_AXI_DATA_WIDTH-1:0]      write_data;
    wire [(C_S_AXI_DATA_WIDTH/8)-1:0]  write_strb;

    assign s_axil_awready = !aw_hold_valid && !s_axil_bvalid;
    assign s_axil_wready  = !w_hold_valid  && !s_axil_bvalid;

    assign aw_accept = s_axil_awvalid && s_axil_awready;
    assign w_accept  = s_axil_wvalid  && s_axil_wready;

    assign write_have_addr = aw_hold_valid || aw_accept;
    assign write_have_data = w_hold_valid  || w_accept;

    assign write_commit =
        !s_axil_bvalid && write_have_addr && write_have_data;

    assign write_addr =
        aw_hold_valid ? aw_hold_addr : s_axil_awaddr;

    assign write_data =
        w_hold_valid ? w_hold_data : s_axil_wdata;

    assign write_strb =
        w_hold_valid ? w_hold_strb : s_axil_wstrb;

    // -------------------------------------------------------------------------
    // AXI Read Channel
    // -------------------------------------------------------------------------
    wire read_accept;

    assign s_axil_arready = !s_axil_rvalid;
    assign read_accept = s_axil_arvalid && s_axil_arready;

    // -------------------------------------------------------------------------
    // Register/Interrupt 내부 상태
    // -------------------------------------------------------------------------
    reg [5:0] irq_enable_reg;

    // Bit 0은 FIFO Not Empty Level이므로 Sticky Register에 저장하지 않는다.
    reg [5:1] irq_pending_sticky;

    wire [5:0] irq_status_value;
    wire [5:1] irq_event_set;
    wire [5:1] irq_w1c_mask;

    wire [31:0] control_current;
    wire [31:0] failover_current;
    wire [31:0] irq_enable_current;

    reg [31:0] merged_control;
    reg [31:0] merged_failover;
    reg [31:0] merged_pair_wait_timeout;
    reg [31:0] merged_channel_timeout;
    reg [31:0] merged_output_device_id;
    reg [31:0] merged_command_map_0;
    reg [31:0] merged_command_map_1;
    reg [31:0] merged_command_map_2;
    reg [31:0] merged_command_map_3;
    reg [31:0] merged_irq_enable;

    reg write_addr_supported;
    reg read_addr_supported;

    integer byte_index;

    assign control_current =
        {31'd0, system_enable};

    assign failover_current = {
        8'd0,
        recovery_count,
        fail_threshold,
        7'd0,
        preferred_channel_b
    };

    assign irq_enable_current =
        {26'd0, irq_enable_reg};

    // AXI WSTRB가 1인 Byte만 새 Write Data로 교체한다.
    always @(*) begin
        merged_control          = control_current;
        merged_failover         = failover_current;
        merged_pair_wait_timeout = pair_wait_timeout_cycles;
        merged_channel_timeout  = channel_timeout_cycles;
        merged_output_device_id = {24'd0, output_device_id};
        merged_command_map_0    = {24'd0, command_map_0};
        merged_command_map_1    = {24'd0, command_map_1};
        merged_command_map_2    = {24'd0, command_map_2};
        merged_command_map_3    = {24'd0, command_map_3};
        merged_irq_enable       = irq_enable_current;

        for (byte_index = 0;
             byte_index < 4;
             byte_index = byte_index + 1) begin

            if (write_strb[byte_index]) begin
                merged_control[(byte_index*8) +: 8] =
                    write_data[(byte_index*8) +: 8];
                merged_failover[(byte_index*8) +: 8] =
                    write_data[(byte_index*8) +: 8];
                merged_pair_wait_timeout[(byte_index*8) +: 8] =
                    write_data[(byte_index*8) +: 8];
                merged_channel_timeout[(byte_index*8) +: 8] =
                    write_data[(byte_index*8) +: 8];
                merged_output_device_id[(byte_index*8) +: 8] =
                    write_data[(byte_index*8) +: 8];
                merged_command_map_0[(byte_index*8) +: 8] =
                    write_data[(byte_index*8) +: 8];
                merged_command_map_1[(byte_index*8) +: 8] =
                    write_data[(byte_index*8) +: 8];
                merged_command_map_2[(byte_index*8) +: 8] =
                    write_data[(byte_index*8) +: 8];
                merged_command_map_3[(byte_index*8) +: 8] =
                    write_data[(byte_index*8) +: 8];
                merged_irq_enable[(byte_index*8) +: 8] =
                    write_data[(byte_index*8) +: 8];
            end
        end
    end

    assign irq_status_value = {
        irq_pending_sticky,
        !event_fifo_empty
    };

    assign irq_event_set[1] = event_lost_pulse;
    assign irq_event_set[2] = frame_mismatch_event;
    assign irq_event_set[3] = both_invalid_event;
    assign irq_event_set[4] = channel_fault_event;
    assign irq_event_set[5] = fifo_underflow_pulse;

    // IRQ_STATUS Write가 실제 Commit되는 클럭에만 W1C Mask를 만든다.
    assign irq_w1c_mask =
        (write_commit &&
         write_addr_supported &&
         (write_addr == ADDR_IRQ_STATUS)) ?
        (write_data[5:1] & {
            {1{write_strb[0]}},
            {1{write_strb[0]}},
            {1{write_strb[0]}},
            {1{write_strb[0]}},
            {1{write_strb[0]}}
        }) :
        5'd0;

    assign irq = |(irq_enable_reg & irq_status_value);

    // -------------------------------------------------------------------------
    // 주소 유효성 조합논리
    // -------------------------------------------------------------------------
    always @(*) begin
        write_addr_supported = 1'b0;

        if (write_addr[1:0] == 2'b00) begin
            case (write_addr)
                ADDR_CONTROL,
                ADDR_FAILOVER_CONFIG,
                ADDR_PAIR_WAIT_TIMEOUT,
                ADDR_CHANNEL_TIMEOUT,
                ADDR_OUTPUT_DEVICE_ID,
                ADDR_COMMAND_MAP_0,
                ADDR_COMMAND_MAP_1,
                ADDR_COMMAND_MAP_2,
                ADDR_COMMAND_MAP_3,
                ADDR_IRQ_ENABLE,
                ADDR_IRQ_STATUS,
                ADDR_EVENT_FIFO_CONTROL:
                    write_addr_supported = 1'b1;

                default:
                    write_addr_supported = 1'b0;
            endcase
        end
    end

    always @(*) begin
        read_addr_supported = 1'b0;

        if (s_axil_araddr[1:0] == 2'b00) begin
            case (s_axil_araddr)
                ADDR_CONTROL,
                ADDR_STATUS,
                ADDR_FAILOVER_CONFIG,
                ADDR_PAIR_WAIT_TIMEOUT,
                ADDR_CHANNEL_TIMEOUT,
                ADDR_OUTPUT_DEVICE_ID,
                ADDR_COMMAND_MAP_0,
                ADDR_COMMAND_MAP_1,
                ADDR_COMMAND_MAP_2,
                ADDR_COMMAND_MAP_3,
                ADDR_IRQ_ENABLE,
                ADDR_IRQ_STATUS,
                ADDR_EVENT_FIFO_STATUS,
                ADDR_EVENT_DATA_LOW,
                ADDR_EVENT_DATA_HIGH,
                ADDR_EVENT_FIFO_CONTROL,
                ADDR_EVENT_LOST_COUNT:
                    read_addr_supported = 1'b1;

                default:
                    read_addr_supported = 1'b0;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Read Data Mux
    // -------------------------------------------------------------------------
    reg [31:0] read_data_mux;

    always @(*) begin
        read_data_mux = 32'd0;

        case (s_axil_araddr)
            ADDR_CONTROL: begin
                read_data_mux = control_current;
            end

            ADDR_STATUS: begin
                read_data_mux = {
                    22'd0,
                    both_invalid_latched,
                    frame_mismatch_latched,
                    !event_fifo_empty,
                    output_busy,
                    pair_wait_active,
                    last_selected_b,
                    preferred_channel_b,
                    channel_b_alive,
                    channel_a_alive,
                    system_enable
                };
            end

            ADDR_FAILOVER_CONFIG: begin
                read_data_mux = failover_current;
            end

            ADDR_PAIR_WAIT_TIMEOUT: begin
                read_data_mux = pair_wait_timeout_cycles;
            end

            ADDR_CHANNEL_TIMEOUT: begin
                read_data_mux = channel_timeout_cycles;
            end

            ADDR_OUTPUT_DEVICE_ID: begin
                read_data_mux = {24'd0, output_device_id};
            end

            ADDR_COMMAND_MAP_0: begin
                read_data_mux = {24'd0, command_map_0};
            end

            ADDR_COMMAND_MAP_1: begin
                read_data_mux = {24'd0, command_map_1};
            end

            ADDR_COMMAND_MAP_2: begin
                read_data_mux = {24'd0, command_map_2};
            end

            ADDR_COMMAND_MAP_3: begin
                read_data_mux = {24'd0, command_map_3};
            end

            ADDR_IRQ_ENABLE: begin
                read_data_mux = irq_enable_current;
            end

            ADDR_IRQ_STATUS: begin
                read_data_mux = {26'd0, irq_status_value};
            end

            ADDR_EVENT_FIFO_STATUS: begin
                read_data_mux = {
                    event_fifo_underflow_count,
                    event_fifo_count,
                    5'd0,
                    event_front_valid,
                    event_fifo_full,
                    event_fifo_empty
                };
            end

            ADDR_EVENT_DATA_LOW: begin
                read_data_mux =
                    event_front_valid ? event_front_data[31:0] : 32'd0;
            end

            ADDR_EVENT_DATA_HIGH: begin
                read_data_mux =
                    event_front_valid ? event_front_data[63:32] : 32'd0;
            end

            ADDR_EVENT_FIFO_CONTROL: begin
                // Write-One-Pulse Register이므로 Readback은 항상 0이다.
                read_data_mux = 32'd0;
            end

            ADDR_EVENT_LOST_COUNT: begin
                read_data_mux = {16'd0, event_lost_count};
            end

            default: begin
                read_data_mux = 32'd0;
            end
        endcase
    end

    // -------------------------------------------------------------------------
    // AXI Write, Register Update, Pulse 생성
    // -------------------------------------------------------------------------
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            aw_hold_valid            <= 1'b0;
            aw_hold_addr             <= {C_S_AXI_ADDR_WIDTH{1'b0}};
            w_hold_valid             <= 1'b0;
            w_hold_data              <= {C_S_AXI_DATA_WIDTH{1'b0}};
            w_hold_strb              <= {(C_S_AXI_DATA_WIDTH/8){1'b0}};

            s_axil_bresp             <= AXI_RESP_OKAY;
            s_axil_bvalid            <= 1'b0;

            system_enable            <= 1'b0;
            preferred_channel_b      <= 1'b0;
            fail_threshold           <= 8'd3;
            recovery_count           <= 8'd5;
            pair_wait_timeout_cycles <= DEFAULT_PAIR_WAIT_TIMEOUT;
            channel_timeout_cycles   <= DEFAULT_CHANNEL_TIMEOUT;
            output_device_id         <= 8'd0;
            command_map_0            <= 8'h10;
            command_map_1            <= 8'h11;
            command_map_2            <= 8'h12;
            command_map_3            <= 8'h13;

            statistics_clear_pulse   <= 1'b0;
            fifo_clear_pulse         <= 1'b0;
            fifo_pop_request         <= 1'b0;

            irq_enable_reg           <= 6'd0;
            irq_pending_sticky       <= 5'd0;
        end
        else begin
            // Write-One-Pulse 출력의 기본값
            statistics_clear_pulse <= 1'b0;
            fifo_clear_pulse       <= 1'b0;
            fifo_pop_request       <= 1'b0;

            // Master가 Write Response를 수락하면 Response를 종료한다.
            if (s_axil_bvalid && s_axil_bready)
                s_axil_bvalid <= 1'b0;

            // Address/Data가 먼저 도착하면 각각 보관한다.
            if (aw_accept) begin
                aw_hold_valid <= 1'b1;
                aw_hold_addr  <= s_axil_awaddr;
            end

            if (w_accept) begin
                w_hold_valid <= 1'b1;
                w_hold_data  <= s_axil_wdata;
                w_hold_strb  <= s_axil_wstrb;
            end

            // Address와 Data를 모두 확보한 시점에 Write를 1회 수행한다.
            if (write_commit) begin
                aw_hold_valid <= 1'b0;
                w_hold_valid  <= 1'b0;

                s_axil_bvalid <= 1'b1;

                if (write_addr_supported) begin
                    s_axil_bresp <= AXI_RESP_OKAY;

                    case (write_addr)
                        ADDR_CONTROL: begin
                            system_enable <= merged_control[0];

                            if (write_strb[0] && write_data[1])
                                statistics_clear_pulse <= 1'b1;

                            if (write_strb[0] && write_data[2])
                                fifo_clear_pulse <= 1'b1;
                        end

                        ADDR_FAILOVER_CONFIG: begin
                            preferred_channel_b <= merged_failover[0];
                            fail_threshold      <= merged_failover[15:8];
                            recovery_count      <= merged_failover[23:16];
                        end

                        ADDR_PAIR_WAIT_TIMEOUT: begin
                            pair_wait_timeout_cycles <=
                                merged_pair_wait_timeout;
                        end

                        ADDR_CHANNEL_TIMEOUT: begin
                            channel_timeout_cycles <=
                                merged_channel_timeout;
                        end

                        ADDR_OUTPUT_DEVICE_ID: begin
                            output_device_id <=
                                merged_output_device_id[7:0];
                        end

                        ADDR_COMMAND_MAP_0: begin
                            command_map_0 <= merged_command_map_0[7:0];
                        end

                        ADDR_COMMAND_MAP_1: begin
                            command_map_1 <= merged_command_map_1[7:0];
                        end

                        ADDR_COMMAND_MAP_2: begin
                            command_map_2 <= merged_command_map_2[7:0];
                        end

                        ADDR_COMMAND_MAP_3: begin
                            command_map_3 <= merged_command_map_3[7:0];
                        end

                        ADDR_IRQ_ENABLE: begin
                            irq_enable_reg <= merged_irq_enable[5:0];
                        end

                        ADDR_IRQ_STATUS: begin
                            // 실제 W1C 처리는 아래 공통 IRQ 갱신식에서 한다.
                        end

                        ADDR_EVENT_FIFO_CONTROL: begin
                            if (write_strb[0] && write_data[0])
                                fifo_pop_request <= 1'b1;

                            if (write_strb[0] && write_data[1])
                                fifo_clear_pulse <= 1'b1;
                        end

                        default: begin
                            // write_addr_supported에서 걸러지므로 미사용
                        end
                    endcase
                end
                else begin
                    s_axil_bresp <= AXI_RESP_SLVERR;
                end
            end

            // Sticky IRQ: W1C 후 같은 클럭의 새 Event를 OR하여 Set 우선
            irq_pending_sticky <=
                (irq_pending_sticky & ~irq_w1c_mask) | irq_event_set;
        end
    end

    // -------------------------------------------------------------------------
    // AXI Read
    // -------------------------------------------------------------------------
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            s_axil_rdata  <= {C_S_AXI_DATA_WIDTH{1'b0}};
            s_axil_rresp  <= AXI_RESP_OKAY;
            s_axil_rvalid <= 1'b0;
        end
        else begin
            if (s_axil_rvalid && s_axil_rready)
                s_axil_rvalid <= 1'b0;

            if (read_accept) begin
                s_axil_rvalid <= 1'b1;

                if (read_addr_supported) begin
                    s_axil_rdata <= read_data_mux;
                    s_axil_rresp <= AXI_RESP_OKAY;
                end
                else begin
                    s_axil_rdata <= {C_S_AXI_DATA_WIDTH{1'b0}};
                    s_axil_rresp <= AXI_RESP_SLVERR;
                end
            end
        end
    end

endmodule
