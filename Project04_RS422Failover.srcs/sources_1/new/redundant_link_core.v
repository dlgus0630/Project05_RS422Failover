`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// redundant_link_core
//
// Redundant RS-422 Fault-Tolerant Communication Gateway SoC의 최종 RTL Core.
//
// Data Path
//   UART RX A/B -> Parser -> CRC -> Sequence -> Frame FIFO
//   -> Pair Matcher -> Health/Decision -> Duplicate Guard
//   -> Output Translation/CRC -> Raw Frame Buffer -> UART TX
//
// Management Path
//   RTL Event Sources -> Event Arbiter -> Event FIFO -> AXI4-Lite -> MicroBlaze
//
// Event Record [63:0]
//   [63:32] Timestamp, 1 us 단위
//   [31:24] Event Code
//   [23:22] Channel: 00 System, 01 A, 10 B, 11 Both
//   [21:14] Sequence
//   [13:0]  Detail
//////////////////////////////////////////////////////////////////////////////////

module redundant_link_core #(
    parameter integer CLK_FREQ_HZ            = 100_000_000,
    parameter integer BAUD_RATE              = 115_200,
    parameter integer INTERBYTE_TIMEOUT_CLKS = 50_000,
    parameter integer FRAME_TIMEOUT_CLKS     = 500_000,
    parameter integer PAIR_TIMEOUT_CYCLES    = 1_000_000,
    parameter integer CHANNEL_TIMEOUT_CYCLES = 30_000_000,
    parameter integer EVENT_FIFO_DEPTH       = 16,
    parameter integer HISTORY_DEPTH          = 4,
    parameter integer SCAN_TICK_CYCLES       = 100_000,
    parameter integer HEARTBEAT_CYCLES       = 50_000_000,
    parameter integer ALERT_HOLD_CYCLES      = 50_000_000
)(
    input  wire        clk,
    input  wire        reset_p,

    input  wire        rs422_rx_a,
    input  wire        rs422_rx_b,
    output wire        rs422_tx_out,

    input  wire [6:0]  s_axil_awaddr,
    input  wire        s_axil_awvalid,
    output wire        s_axil_awready,
    input  wire [31:0] s_axil_wdata,
    input  wire [3:0]  s_axil_wstrb,
    input  wire        s_axil_wvalid,
    output wire        s_axil_wready,
    output wire [1:0]  s_axil_bresp,
    output wire        s_axil_bvalid,
    input  wire        s_axil_bready,

    input  wire [6:0]  s_axil_araddr,
    input  wire        s_axil_arvalid,
    output wire        s_axil_arready,
    output wire [31:0] s_axil_rdata,
    output wire [1:0]  s_axil_rresp,
    output wire        s_axil_rvalid,
    input  wire        s_axil_rready,

    output wire        irq,
    output wire [15:0] led,
    output wire [6:0]  seg,
    output wire        dp,
    output wire [3:0]  an
);

    localparam [1:0] RESULT_PAIR     = 2'b01;
    localparam [1:0] RESULT_SINGLE_A = 2'b10;
    localparam [1:0] RESULT_SINGLE_B = 2'b11;

    localparam [1:0] CHANNEL_SYSTEM = 2'b00;
    localparam [1:0] CHANNEL_A      = 2'b01;
    localparam [1:0] CHANNEL_B      = 2'b10;
    localparam [1:0] CHANNEL_BOTH   = 2'b11;

    localparam [7:0] EV_CRC_ERROR          = 8'h01;
    localparam [7:0] EV_LENGTH_ERROR       = 8'h02;
    localparam [7:0] EV_UART_FRAME_ERROR   = 8'h03;
    localparam [7:0] EV_SEQ_GAP            = 8'h04;
    localparam [7:0] EV_SEQ_DUP            = 8'h05;
    localparam [7:0] EV_SEQ_OLD            = 8'h06;
    localparam [7:0] EV_CHANNEL_TIMEOUT    = 8'h07;
    localparam [7:0] EV_PAIR_TIMEOUT       = 8'h08;
    localparam [7:0] EV_PAIR_SEQ_MISMATCH  = 8'h09;
    localparam [7:0] EV_PAIR_SEQ_AMBIGUOUS = 8'h0A;
    localparam [7:0] EV_DATA_MISMATCH      = 8'h0B;
    localparam [7:0] EV_BOTH_INVALID       = 8'h0C;
    localparam [7:0] EV_FRAME_FALLBACK     = 8'h0D;
    localparam [7:0] EV_FAILOVER_A_TO_B    = 8'h0E;
    localparam [7:0] EV_FAILOVER_B_TO_A    = 8'h0F;
    localparam [7:0] EV_RECOVERY_DEFAULT   = 8'h10;
    localparam [7:0] EV_OUTPUT_BLOCKED     = 8'h11;
    localparam [7:0] EV_UNSUPPORTED_CMD    = 8'h12;
    localparam [7:0] EV_FIFO_OVERFLOW      = 8'h13;
    localparam [7:0] EV_DUPLICATE_DROP     = 8'h14;
    localparam [7:0] EV_INTERBYTE_TIMEOUT  = 8'h15;
    localparam [7:0] EV_FRAME_TIMEOUT      = 8'h16;
    localparam [7:0] EV_CHANNEL_FAULT      = 8'h17;
    localparam [7:0] EV_CHANNEL_RECOVERED  = 8'h18;

    localparam integer EVENT_SOURCE_COUNT = 16;
    localparam integer US_TICK_CYCLES =
        (CLK_FREQ_HZ <= 1_000_000) ? 1 : (CLK_FREQ_HZ / 1_000_000);
    localparam integer US_COUNT_WIDTH =
        (US_TICK_CYCLES <= 1) ? 1 : $clog2(US_TICK_CYCLES);

    // -------------------------------------------------------------------------
    // AXI Configuration
    // -------------------------------------------------------------------------
    wire        system_enable;
    wire        preferred_channel_b;
    wire [7:0]  fail_threshold;
    wire [7:0]  recovery_count_cfg;
    wire [31:0] pair_wait_timeout_cycles;
    wire [31:0] channel_timeout_cycles;
    wire [7:0]  output_device_id;
    wire [7:0]  command_map_0;
    wire [7:0]  command_map_1;
    wire [7:0]  command_map_2;
    wire [7:0]  command_map_3;
    wire        statistics_clear_pulse;
    wire        fifo_clear_pulse;
    wire        fifo_pop_request;

    // reset_p만 비동기 Reset으로 사용한다. AXI의 system_enable은 각
    // 데이터 경로 블록에서 clk에 동기된 Clear로 처리한다.
    wire datapath_clear;
    assign datapath_clear = !system_enable;

    // -------------------------------------------------------------------------
    // UART RX A/B
    // -------------------------------------------------------------------------
    wire [7:0] a_rx_data;
    wire       a_rx_valid;
    wire       a_uart_frame_error;
    wire [7:0] b_rx_data;
    wire       b_rx_valid;
    wire       b_uart_frame_error;

    uart_rx #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE)
    ) u_uart_rx_a (
        .clk            (clk),
        .reset_p        (reset_p),
        .clear          (datapath_clear),
        .rx             (rs422_rx_a),
        .rx_data        (a_rx_data),
        .rx_valid       (a_rx_valid),
        .rx_frame_error (a_uart_frame_error)
    );

    uart_rx #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE)
    ) u_uart_rx_b (
        .clk            (clk),
        .reset_p        (reset_p),
        .clear          (datapath_clear),
        .rx             (rs422_rx_b),
        .rx_data        (b_rx_data),
        .rx_valid       (b_rx_valid),
        .rx_frame_error (b_uart_frame_error)
    );

    // -------------------------------------------------------------------------
    // Frame Parser A/B
    // -------------------------------------------------------------------------
    wire [7:0]   a_frame_length;
    wire [7:0]   a_device_id;
    wire [7:0]   a_command;
    wire [7:0]   a_sequence;
    wire [127:0] a_payload_data;
    wire [15:0]  a_received_crc;
    wire         a_packet_valid;
    wire [7:0]   a_crc_data;
    wire         a_crc_data_valid;
    wire         a_crc_start;
    wire         a_length_error;
    wire         a_interbyte_timeout;
    wire         a_frame_timeout;

    wire [7:0]   b_frame_length;
    wire [7:0]   b_device_id;
    wire [7:0]   b_command;
    wire [7:0]   b_sequence;
    wire [127:0] b_payload_data;
    wire [15:0]  b_received_crc;
    wire         b_packet_valid;
    wire [7:0]   b_crc_data;
    wire         b_crc_data_valid;
    wire         b_crc_start;
    wire         b_length_error;
    wire         b_interbyte_timeout;
    wire         b_frame_timeout;

    frame_parser #(
        .INTERBYTE_TIMEOUT_CLKS (INTERBYTE_TIMEOUT_CLKS),
        .FRAME_TIMEOUT_CLKS     (FRAME_TIMEOUT_CLKS)
    ) u_parser_a (
        .clk               (clk),
        .reset_p           (reset_p),
        .clear             (datapath_clear),
        .rx_data           (a_rx_data),
        .rx_valid          (a_rx_valid),
        .rx_frame_error    (a_uart_frame_error),
        .frame_length      (a_frame_length),
        .device_id         (a_device_id),
        .command           (a_command),
        .sequence          (a_sequence),
        .payload_data      (a_payload_data),
        .received_crc      (a_received_crc),
        .packet_valid      (a_packet_valid),
        .crc_data          (a_crc_data),
        .crc_data_valid    (a_crc_data_valid),
        .crc_start         (a_crc_start),
        .length_error      (a_length_error),
        .interbyte_timeout (a_interbyte_timeout),
        .frame_timeout     (a_frame_timeout)
    );

    frame_parser #(
        .INTERBYTE_TIMEOUT_CLKS (INTERBYTE_TIMEOUT_CLKS),
        .FRAME_TIMEOUT_CLKS     (FRAME_TIMEOUT_CLKS)
    ) u_parser_b (
        .clk               (clk),
        .reset_p           (reset_p),
        .clear             (datapath_clear),
        .rx_data           (b_rx_data),
        .rx_valid          (b_rx_valid),
        .rx_frame_error    (b_uart_frame_error),
        .frame_length      (b_frame_length),
        .device_id         (b_device_id),
        .command           (b_command),
        .sequence          (b_sequence),
        .payload_data      (b_payload_data),
        .received_crc      (b_received_crc),
        .packet_valid      (b_packet_valid),
        .crc_data          (b_crc_data),
        .crc_data_valid    (b_crc_data_valid),
        .crc_start         (b_crc_start),
        .length_error      (b_length_error),
        .interbyte_timeout (b_interbyte_timeout),
        .frame_timeout     (b_frame_timeout)
    );

    // -------------------------------------------------------------------------
    // CRC A/B
    // -------------------------------------------------------------------------
    wire [15:0] a_calculated_crc;
    wire        a_crc_done;
    wire        a_crc_ok;
    wire        a_crc_error;
    wire [15:0] b_calculated_crc;
    wire        b_crc_done;
    wire        b_crc_ok;
    wire        b_crc_error;

    crc16_ccitt u_crc_a (
        .clk            (clk),
        .reset_p        (reset_p),
        .clear          (datapath_clear),
        .crc_data       (a_crc_data),
        .crc_data_valid (a_crc_data_valid),
        .crc_start      (a_crc_start),
        .packet_valid   (a_packet_valid),
        .received_crc   (a_received_crc),
        .calculated_crc (a_calculated_crc),
        .crc_done       (a_crc_done),
        .crc_ok         (a_crc_ok),
        .crc_error      (a_crc_error)
    );

    crc16_ccitt u_crc_b (
        .clk            (clk),
        .reset_p        (reset_p),
        .clear          (datapath_clear),
        .crc_data       (b_crc_data),
        .crc_data_valid (b_crc_data_valid),
        .crc_start      (b_crc_start),
        .packet_valid   (b_packet_valid),
        .received_crc   (b_received_crc),
        .calculated_crc (b_calculated_crc),
        .crc_done       (b_crc_done),
        .crc_ok         (b_crc_ok),
        .crc_error      (b_crc_error)
    );

    // -------------------------------------------------------------------------
    // Channel Alive / Timeout
    // -------------------------------------------------------------------------
    reg        channel_a_alive;
    reg        channel_b_alive;
    reg        a_seen_valid;
    reg        b_seen_valid;
    reg [31:0] a_channel_count;
    reg [31:0] b_channel_count;
    reg        a_channel_timeout_pulse;
    reg        b_channel_timeout_pulse;
    reg [7:0]  a_timeout_sequence;
    reg [7:0]  b_timeout_sequence;
    wire [7:0] a_last_rx_seq;
    wire [7:0] b_last_rx_seq;

    wire [31:0] effective_channel_timeout;
    wire        a_channel_timeout_fire;
    wire        b_channel_timeout_fire;

    assign effective_channel_timeout =
        (channel_timeout_cycles == 32'd0) ?
        CHANNEL_TIMEOUT_CYCLES : channel_timeout_cycles;

    assign a_channel_timeout_fire =
        !datapath_clear &&
        !(a_crc_done && a_crc_ok) &&
        a_seen_valid && channel_a_alive &&
        (a_channel_count >= (effective_channel_timeout - 1'b1));

    assign b_channel_timeout_fire =
        !datapath_clear &&
        !(b_crc_done && b_crc_ok) &&
        b_seen_valid && channel_b_alive &&
        (b_channel_count >= (effective_channel_timeout - 1'b1));

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            channel_a_alive       <= 1'b0;
            a_seen_valid          <= 1'b0;
            a_channel_count       <= 32'd0;
            a_channel_timeout_pulse <= 1'b0;
            a_timeout_sequence      <= 8'd0;
        end
        else if (datapath_clear) begin
            channel_a_alive       <= 1'b0;
            a_seen_valid          <= 1'b0;
            a_channel_count       <= 32'd0;
            a_channel_timeout_pulse <= 1'b0;
            a_timeout_sequence      <= 8'd0;
        end
        else begin
            a_channel_timeout_pulse <= 1'b0;

            if (a_crc_done && a_crc_ok) begin
                channel_a_alive <= 1'b1;
                a_seen_valid    <= 1'b1;
                a_channel_count <= 32'd0;
            end
            else if (a_channel_timeout_fire) begin
                channel_a_alive         <= 1'b0;
                a_channel_timeout_pulse <= 1'b1;
                a_timeout_sequence      <= a_last_rx_seq;
            end
            else if (a_seen_valid && channel_a_alive) begin
                a_channel_count <= a_channel_count + 1'b1;
            end
        end
    end

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            channel_b_alive       <= 1'b0;
            b_seen_valid          <= 1'b0;
            b_channel_count       <= 32'd0;
            b_channel_timeout_pulse <= 1'b0;
            b_timeout_sequence      <= 8'd0;
        end
        else if (datapath_clear) begin
            channel_b_alive       <= 1'b0;
            b_seen_valid          <= 1'b0;
            b_channel_count       <= 32'd0;
            b_channel_timeout_pulse <= 1'b0;
            b_timeout_sequence      <= 8'd0;
        end
        else begin
            b_channel_timeout_pulse <= 1'b0;

            if (b_crc_done && b_crc_ok) begin
                channel_b_alive <= 1'b1;
                b_seen_valid    <= 1'b1;
                b_channel_count <= 32'd0;
            end
            else if (b_channel_timeout_fire) begin
                channel_b_alive         <= 1'b0;
                b_channel_timeout_pulse <= 1'b1;
                b_timeout_sequence      <= b_last_rx_seq;
            end
            else if (b_seen_valid && channel_b_alive) begin
                b_channel_count <= b_channel_count + 1'b1;
            end
        end
    end

    // Timeout 후 해당 채널의 Sequence 기준과 대기 Frame을 비운다.
    wire a_seq_clear;
    wire b_seq_clear;
    assign a_seq_clear = datapath_clear | a_channel_timeout_fire;
    assign b_seq_clear = datapath_clear | b_channel_timeout_fire;

    // -------------------------------------------------------------------------
    // Sequence Monitor A/B
    // -------------------------------------------------------------------------
    wire       a_seq_accept;
    wire       a_seq_ok;
    wire       a_seq_duplicate;
    wire       a_seq_gap;
    wire       a_seq_old;
    wire       a_seq_initialized;
    wire       b_seq_accept;
    wire       b_seq_ok;
    wire       b_seq_duplicate;
    wire       b_seq_gap;
    wire       b_seq_old;
    wire       b_seq_initialized;

    seq_monitor u_seq_a (
        .clk             (clk),
        .reset_p         (reset_p),
        .clear           (a_seq_clear),
        .seq_valid       (a_crc_done && a_crc_ok),
        .seq_value       (a_sequence),
        .seq_accept      (a_seq_accept),
        .seq_ok          (a_seq_ok),
        .seq_duplicate   (a_seq_duplicate),
        .seq_gap         (a_seq_gap),
        .seq_old         (a_seq_old),
        .last_rx_seq     (a_last_rx_seq),
        .seq_initialized (a_seq_initialized)
    );

    seq_monitor u_seq_b (
        .clk             (clk),
        .reset_p         (reset_p),
        .clear           (b_seq_clear),
        .seq_valid       (b_crc_done && b_crc_ok),
        .seq_value       (b_sequence),
        .seq_accept      (b_seq_accept),
        .seq_ok          (b_seq_ok),
        .seq_duplicate   (b_seq_duplicate),
        .seq_gap         (b_seq_gap),
        .seq_old         (b_seq_old),
        .last_rx_seq     (b_last_rx_seq),
        .seq_initialized (b_seq_initialized)
    );

    // -------------------------------------------------------------------------
    // Frame FIFO A/B
    // -------------------------------------------------------------------------
    wire       pop_a;
    wire       pop_b;
    wire [7:0] a_fifo_length;
    wire [7:0] a_fifo_device_id;
    wire [7:0] a_fifo_command;
    wire [7:0] a_fifo_sequence;
    wire [127:0] a_fifo_payload;
    wire [15:0] a_fifo_crc;
    wire       a_fifo_seq_gap;
    wire       a_fifo_empty;
    wire       a_fifo_full;
    wire [1:0] a_fifo_count;
    wire       a_fifo_overflow;

    wire [7:0] b_fifo_length;
    wire [7:0] b_fifo_device_id;
    wire [7:0] b_fifo_command;
    wire [7:0] b_fifo_sequence;
    wire [127:0] b_fifo_payload;
    wire [15:0] b_fifo_crc;
    wire       b_fifo_seq_gap;
    wire       b_fifo_empty;
    wire       b_fifo_full;
    wire [1:0] b_fifo_count;
    wire       b_fifo_overflow;

    frame_fifo u_fifo_a (
        .clk              (clk),
        .reset_p          (reset_p),
        .clear            (a_seq_clear),
        .push             (a_seq_accept),
        .in_frame_length  (a_frame_length),
        .in_device_id     (a_device_id),
        .in_command       (a_command),
        .in_sequence      (a_sequence),
        .in_payload_data  (a_payload_data),
        .in_received_crc  (a_received_crc),
        .in_seq_gap       (a_seq_gap),
        .pop              (pop_a),
        .out_frame_length (a_fifo_length),
        .out_device_id    (a_fifo_device_id),
        .out_command      (a_fifo_command),
        .out_sequence     (a_fifo_sequence),
        .out_payload_data (a_fifo_payload),
        .out_received_crc (a_fifo_crc),
        .out_seq_gap      (a_fifo_seq_gap),
        .empty            (a_fifo_empty),
        .full             (a_fifo_full),
        .count            (a_fifo_count),
        .overflow_pulse   (a_fifo_overflow)
    );

    frame_fifo u_fifo_b (
        .clk              (clk),
        .reset_p          (reset_p),
        .clear            (b_seq_clear),
        .push             (b_seq_accept),
        .in_frame_length  (b_frame_length),
        .in_device_id     (b_device_id),
        .in_command       (b_command),
        .in_sequence      (b_sequence),
        .in_payload_data  (b_payload_data),
        .in_received_crc  (b_received_crc),
        .in_seq_gap       (b_seq_gap),
        .pop              (pop_b),
        .out_frame_length (b_fifo_length),
        .out_device_id    (b_fifo_device_id),
        .out_command      (b_fifo_command),
        .out_sequence     (b_fifo_sequence),
        .out_payload_data (b_fifo_payload),
        .out_received_crc (b_fifo_crc),
        .out_seq_gap      (b_fifo_seq_gap),
        .empty            (b_fifo_empty),
        .full             (b_fifo_full),
        .count            (b_fifo_count),
        .overflow_pulse   (b_fifo_overflow)
    );

    // -------------------------------------------------------------------------
    // Pair Matcher
    // -------------------------------------------------------------------------
    wire        matcher_result_valid;
    wire [1:0]  matcher_result_kind;
    wire        matcher_pair_equal;
    wire [5:0]  matcher_mismatch_flags;
    wire        matcher_timeout;
    wire        matcher_seq_skew;
    wire        matcher_seq_ambiguous;
    wire        matcher_a_seq_gap;
    wire        matcher_b_seq_gap;
    wire        pair_wait_active;
    wire [7:0]  matcher_length;
    wire [7:0]  matcher_device_id;
    wire [7:0]  matcher_command;
    wire [7:0]  matcher_sequence;
    wire [127:0] matcher_payload;
    wire [15:0] matcher_crc;
    wire        matcher_seq_gap;

    pair_matcher #(
        .PAIR_TIMEOUT_CYCLES (PAIR_TIMEOUT_CYCLES)
    ) u_pair_matcher (
        .clk                  (clk),
        .reset_p              (reset_p),
        .clear                (datapath_clear),
        .pair_timeout_cycles  (pair_wait_timeout_cycles),
        .a_empty              (a_fifo_empty),
        .a_frame_length       (a_fifo_length),
        .a_device_id          (a_fifo_device_id),
        .a_command            (a_fifo_command),
        .a_sequence           (a_fifo_sequence),
        .a_payload_data       (a_fifo_payload),
        .a_received_crc       (a_fifo_crc),
        .a_seq_gap            (a_fifo_seq_gap),
        .pop_a                (pop_a),
        .b_empty              (b_fifo_empty),
        .b_frame_length       (b_fifo_length),
        .b_device_id          (b_fifo_device_id),
        .b_command            (b_fifo_command),
        .b_sequence           (b_fifo_sequence),
        .b_payload_data       (b_fifo_payload),
        .b_received_crc       (b_fifo_crc),
        .b_seq_gap            (b_fifo_seq_gap),
        .pop_b                (pop_b),
        .result_valid         (matcher_result_valid),
        .result_kind          (matcher_result_kind),
        .result_pair_equal    (matcher_pair_equal),
        .mismatch_flags       (matcher_mismatch_flags),
        .result_timeout       (matcher_timeout),
        .result_seq_skew      (matcher_seq_skew),
        .result_seq_ambiguous (matcher_seq_ambiguous),
        .result_a_seq_gap     (matcher_a_seq_gap),
        .result_b_seq_gap     (matcher_b_seq_gap),
        .pair_wait_active     (pair_wait_active),
        .out_frame_length     (matcher_length),
        .out_device_id        (matcher_device_id),
        .out_command          (matcher_command),
        .out_sequence         (matcher_sequence),
        .out_payload_data     (matcher_payload),
        .out_received_crc     (matcher_crc),
        .out_seq_gap          (matcher_seq_gap)
    );

    // -------------------------------------------------------------------------
    // Channel Health
    // -------------------------------------------------------------------------
    wire a_local_fail_event;
    wire b_local_fail_event;

    assign a_local_fail_event =
        a_uart_frame_error | a_length_error | a_interbyte_timeout |
        a_frame_timeout | a_crc_error | a_seq_duplicate | a_seq_gap |
        a_seq_old | a_fifo_overflow | a_channel_timeout_pulse;

    assign b_local_fail_event =
        b_uart_frame_error | b_length_error | b_interbyte_timeout |
        b_frame_timeout | b_crc_error | b_seq_duplicate | b_seq_gap |
        b_seq_old | b_fifo_overflow | b_channel_timeout_pulse;

    wire       a_fault;
    wire       b_fault;
    wire [7:0] a_fail_count;
    wire [7:0] b_fail_count;
    wire [7:0] a_recover_count;
    wire [7:0] b_recover_count;
    wire       a_fault_enter_pulse;
    wire       b_fault_enter_pulse;
    wire       a_recovered_pulse;
    wire       b_recovered_pulse;

    channel_health_mgr u_health (
        .clk                  (clk),
        .reset_p              (reset_p),
        .clear                (datapath_clear),
        .matcher_result_valid (matcher_result_valid),
        .matcher_result_kind  (matcher_result_kind),
        .matcher_pair_equal        (matcher_pair_equal),
        .matcher_result_a_seq_gap   (matcher_a_seq_gap),
        .matcher_result_b_seq_gap   (matcher_b_seq_gap),
        .a_local_fail_event          (a_local_fail_event),
        .b_local_fail_event   (b_local_fail_event),
        .fail_threshold_cfg   (fail_threshold),
        .recover_threshold_cfg(recovery_count_cfg),
        .a_fault              (a_fault),
        .b_fault              (b_fault),
        .a_fail_count         (a_fail_count),
        .b_fail_count         (b_fail_count),
        .a_recover_count      (a_recover_count),
        .b_recover_count      (b_recover_count),
        .a_fault_enter_pulse  (a_fault_enter_pulse),
        .b_fault_enter_pulse  (b_fault_enter_pulse),
        .a_recovered_pulse    (a_recovered_pulse),
        .b_recovered_pulse    (b_recovered_pulse)
    );

    wire channel_a_usable;
    wire channel_b_usable;
    assign channel_a_usable = channel_a_alive && !a_fault;
    assign channel_b_usable = channel_b_alive && !b_fault;

    // -------------------------------------------------------------------------
    // Decision
    // -------------------------------------------------------------------------
    wire        decision_valid;
    wire        decision_accept;
    wire        decision_degraded;
    wire        decision_mismatch_drop;
    wire        decision_both_invalid;
    wire        decision_selected_b;
    wire [7:0]  decision_length;
    wire [7:0]  decision_device_id;
    wire [7:0]  decision_command;
    wire [7:0]  decision_sequence;
    wire [127:0] decision_payload;
    wire [15:0] decision_crc;
    wire        decision_seq_gap;

    decision_unit u_decision (
        .clk                    (clk),
        .reset_p                (reset_p),
        .system_enable          (system_enable),
        .preferred_channel_b    (preferred_channel_b),
        .channel_a_usable       (channel_a_usable),
        .channel_b_usable       (channel_b_usable),
        .result_valid           (matcher_result_valid),
        .result_kind            (matcher_result_kind),
        .result_pair_equal      (matcher_pair_equal),
        .in_frame_length        (matcher_length),
        .in_device_id           (matcher_device_id),
        .in_command             (matcher_command),
        .in_sequence            (matcher_sequence),
        .in_payload_data        (matcher_payload),
        .in_received_crc        (matcher_crc),
        .in_seq_gap             (matcher_seq_gap),
        .decision_valid         (decision_valid),
        .decision_accept        (decision_accept),
        .decision_degraded      (decision_degraded),
        .decision_mismatch_drop (decision_mismatch_drop),
        .decision_both_invalid  (decision_both_invalid),
        .decision_selected_b    (decision_selected_b),
        .out_frame_length       (decision_length),
        .out_device_id          (decision_device_id),
        .out_command            (decision_command),
        .out_sequence           (decision_sequence),
        .out_payload_data       (decision_payload),
        .out_received_crc       (decision_crc),
        .out_seq_gap            (decision_seq_gap)
    );

    // -------------------------------------------------------------------------
    // Duplicate Guard와 Translation 예약
    // -------------------------------------------------------------------------
    wire decision_command_supported;
    assign decision_command_supported =
        (decision_command == 8'h10) ||
        (decision_command == 8'h11) ||
        (decision_command == 8'h12) ||
        (decision_command == 8'h13);

    reg translation_reserved;
    wire guard_accept;
    wire unsupported_command_pulse;
    wire output_blocked_pulse;

    assign guard_accept =
        decision_accept && decision_command_supported &&
        !translation_reserved;

    assign unsupported_command_pulse =
        decision_valid && decision_accept &&
        !decision_command_supported;

    assign output_blocked_pulse =
        decision_valid && decision_accept &&
        decision_command_supported && translation_reserved;

    wire        duplicate_out_valid;
    wire [7:0]  duplicate_length;
    wire [7:0]  duplicate_device_id;
    wire [7:0]  duplicate_command;
    wire [7:0]  duplicate_sequence;
    wire [127:0] duplicate_payload;
    wire [15:0] duplicate_crc;
    wire        duplicate_seq_gap;
    wire        duplicate_selected_b;
    wire        duplicate_drop;
    wire [15:0] duplicate_count;

    duplicate_guard #(
        .HISTORY_DEPTH (HISTORY_DEPTH)
    ) u_duplicate_guard (
        .clk              (clk),
        .reset_p          (reset_p),
        .clear            (datapath_clear),
        .decision_valid   (decision_valid),
        .decision_accept  (guard_accept),
        .statistics_clear (statistics_clear_pulse),
        .in_frame_length  (decision_length),
        .in_device_id     (decision_device_id),
        .in_command       (decision_command),
        .in_sequence      (decision_sequence),
        .in_payload_data  (decision_payload),
        .in_received_crc  (decision_crc),
        .in_seq_gap       (decision_seq_gap),
        .in_selected_b    (decision_selected_b),
        .out_valid        (duplicate_out_valid),
        .out_frame_length (duplicate_length),
        .out_device_id    (duplicate_device_id),
        .out_command      (duplicate_command),
        .out_sequence     (duplicate_sequence),
        .out_payload_data (duplicate_payload),
        .out_received_crc (duplicate_crc),
        .out_seq_gap      (duplicate_seq_gap),
        .out_selected_b   (duplicate_selected_b),
        .duplicate_drop   (duplicate_drop),
        .duplicate_count  (duplicate_count)
    );

    // -------------------------------------------------------------------------
    // Output Translation + CRC-16/CCITT-FALSE 재계산
    // -------------------------------------------------------------------------
    reg        translation_crc_active;
    reg        translation_valid;
    reg [5:0]  translation_byte_index;
    reg [15:0] translation_crc_reg;
    reg [7:0]  translation_length;
    reg [7:0]  translation_device_id;
    reg [7:0]  translation_command;
    reg [7:0]  translation_sequence;
    reg [127:0] translation_payload;
    reg        translation_seq_gap;
    reg        translation_selected_b;

    reg [7:0]  translation_crc_byte;
    reg [15:0] translation_crc_next;
    integer output_crc_bit_index;

    always @(*) begin
        translation_crc_byte = 8'd0;

        case (translation_byte_index)
            6'd0: translation_crc_byte = translation_length;
            6'd1: translation_crc_byte = translation_device_id;
            6'd2: translation_crc_byte = translation_command;
            6'd3: translation_crc_byte = translation_sequence;
            default: begin
                if ((translation_byte_index >= 6'd4) &&
                    (translation_byte_index <= translation_length)) begin
                    translation_crc_byte =
                        translation_payload >>
                        ((translation_length -
                          translation_byte_index) * 8);
                end
            end
        endcase

        translation_crc_next =
            translation_crc_reg ^ {translation_crc_byte, 8'h00};

        for (output_crc_bit_index = 0;
             output_crc_bit_index < 8;
             output_crc_bit_index = output_crc_bit_index + 1) begin
            if (translation_crc_next[15])
                translation_crc_next =
                    {translation_crc_next[14:0], 1'b0} ^ 16'h1021;
            else
                translation_crc_next =
                    {translation_crc_next[14:0], 1'b0};
        end
    end

    wire translated_frame_fire;
    wire raw_in_ready;

    assign translated_frame_fire = translation_valid && raw_in_ready;

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            translation_reserved   <= 1'b0;
            translation_crc_active <= 1'b0;
            translation_valid      <= 1'b0;
            translation_byte_index <= 6'd0;
            translation_crc_reg    <= 16'hFFFF;
            translation_length     <= 8'd0;
            translation_device_id  <= 8'd0;
            translation_command    <= 8'd0;
            translation_sequence   <= 8'd0;
            translation_payload    <= 128'd0;
            translation_seq_gap    <= 1'b0;
            translation_selected_b <= 1'b0;
        end
        else if (datapath_clear) begin
            translation_reserved   <= 1'b0;
            translation_crc_active <= 1'b0;
            translation_valid      <= 1'b0;
            translation_byte_index <= 6'd0;
            translation_crc_reg    <= 16'hFFFF;
            translation_length     <= 8'd0;
            translation_device_id  <= 8'd0;
            translation_command    <= 8'd0;
            translation_sequence   <= 8'd0;
            translation_payload    <= 128'd0;
            translation_seq_gap    <= 1'b0;
            translation_selected_b <= 1'b0;
        end
        else begin
            if (decision_valid && guard_accept)
                translation_reserved <= 1'b1;

            if (duplicate_drop)
                translation_reserved <= 1'b0;

            if (translated_frame_fire) begin
                translation_valid    <= 1'b0;
                translation_reserved <= 1'b0;
            end

            if (duplicate_out_valid) begin
                translation_crc_active <= 1'b1;
                translation_valid      <= 1'b0;
                translation_byte_index <= 6'd0;
                translation_crc_reg    <= 16'hFFFF;
                translation_length     <= duplicate_length;
                translation_device_id  <= output_device_id;
                translation_sequence   <= duplicate_sequence;
                translation_payload    <= duplicate_payload;
                translation_seq_gap    <= duplicate_seq_gap;
                translation_selected_b <= duplicate_selected_b;

                case (duplicate_command)
                    8'h10: translation_command <= command_map_0;
                    8'h11: translation_command <= command_map_1;
                    8'h12: translation_command <= command_map_2;
                    8'h13: translation_command <= command_map_3;
                    default: translation_command <= duplicate_command;
                endcase
            end
            else if (translation_crc_active) begin
                translation_crc_reg <= translation_crc_next;

                if (translation_byte_index == translation_length) begin
                    translation_crc_active <= 1'b0;
                    translation_valid      <= 1'b1;
                end
                else begin
                    translation_byte_index <=
                        translation_byte_index + 1'b1;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Raw Frame Buffer -> UART TX
    // -------------------------------------------------------------------------
    wire       raw_tx_valid;
    wire       raw_tx_ready;
    wire [7:0] raw_tx_data;
    wire       output_busy;
    wire       raw_buffer_full;
    wire [7:0] raw_buffered_count;
    wire       raw_frame_done;
    wire       raw_frame_done_seq_gap;
    wire       raw_overflow_pulse;
    wire [15:0] raw_overflow_count;
    wire       raw_length_error_pulse;
    wire       uart_tx_busy;
    wire       uart_tx_done;

    raw_frame_buffer u_raw_buffer (
        .clk                 (clk),
        .reset_p             (reset_p),
        .clear               (datapath_clear),
        .statistics_clear    (statistics_clear_pulse),
        .in_valid            (translation_valid),
        .in_ready            (raw_in_ready),
        .in_frame_length     (translation_length),
        .in_device_id        (translation_device_id),
        .in_command          (translation_command),
        .in_sequence         (translation_sequence),
        .in_payload_data     (translation_payload),
        .in_received_crc     (translation_crc_reg),
        .in_seq_gap          (translation_seq_gap),
        .tx_valid            (raw_tx_valid),
        .tx_ready            (raw_tx_ready),
        .tx_data             (raw_tx_data),
        .buffer_busy         (output_busy),
        .buffer_full         (raw_buffer_full),
        .buffered_count      (raw_buffered_count),
        .frame_done          (raw_frame_done),
        .frame_done_seq_gap  (raw_frame_done_seq_gap),
        .overflow_pulse      (raw_overflow_pulse),
        .overflow_count      (raw_overflow_count),
        .length_error_pulse  (raw_length_error_pulse)
    );

    uart_tx #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE)
    ) u_uart_tx (
        .clk      (clk),
        .reset_p  (reset_p),
        .clear    (datapath_clear),
        .tx_valid (raw_tx_valid),
        .tx_ready (raw_tx_ready),
        .tx_data  (raw_tx_data),
        .uart_txd (rs422_tx_out),
        .tx_busy  (uart_tx_busy),
        .tx_done  (uart_tx_done)
    );

    // -------------------------------------------------------------------------
    // 실제 출력 Frame 기준 선택 채널/Failover 추적
    // -------------------------------------------------------------------------
    reg last_selected_b;
    reg selection_initialized;
    reg failover_a_to_b_pulse;
    reg failover_b_to_a_pulse;
    reg recovery_default_pulse;

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            last_selected_b          <= 1'b0;
            selection_initialized    <= 1'b0;
            failover_a_to_b_pulse    <= 1'b0;
            failover_b_to_a_pulse    <= 1'b0;
            recovery_default_pulse   <= 1'b0;
        end
        else if (datapath_clear) begin
            last_selected_b          <= 1'b0;
            selection_initialized    <= 1'b0;
            failover_a_to_b_pulse    <= 1'b0;
            failover_b_to_a_pulse    <= 1'b0;
            recovery_default_pulse   <= 1'b0;
        end
        else begin
            failover_a_to_b_pulse  <= 1'b0;
            failover_b_to_a_pulse  <= 1'b0;
            recovery_default_pulse <= 1'b0;

            if (translated_frame_fire) begin
                if (selection_initialized &&
                    (last_selected_b != translation_selected_b)) begin
                    if (translation_selected_b)
                        failover_a_to_b_pulse <= 1'b1;
                    else
                        failover_b_to_a_pulse <= 1'b1;

                    if (translation_selected_b ==
                        preferred_channel_b)
                        recovery_default_pulse <= 1'b1;
                end

                last_selected_b       <= translation_selected_b;
                selection_initialized <= 1'b1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Sticky 상태
    // -------------------------------------------------------------------------
    reg frame_mismatch_latched;
    reg both_invalid_latched;
    reg [5:0] last_matcher_mismatch_flags;

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            frame_mismatch_latched <= 1'b0;
            both_invalid_latched   <= 1'b0;
            last_matcher_mismatch_flags <= 6'd0;
        end
        else begin
            if (statistics_clear_pulse) begin
                frame_mismatch_latched <= 1'b0;
                both_invalid_latched   <= 1'b0;
            end

            if (decision_mismatch_drop)
                frame_mismatch_latched <= 1'b1;

            if (decision_both_invalid)
                both_invalid_latched <= 1'b1;

            if (matcher_result_valid)
                last_matcher_mismatch_flags <= matcher_mismatch_flags;
        end
    end

    // -------------------------------------------------------------------------
    // 1 us Timestamp
    // -------------------------------------------------------------------------
    reg [US_COUNT_WIDTH-1:0] us_div_count;
    reg [31:0] timestamp_us;

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            us_div_count <= {US_COUNT_WIDTH{1'b0}};
            timestamp_us <= 32'd0;
        end
        else if (US_TICK_CYCLES <= 1) begin
            us_div_count <= {US_COUNT_WIDTH{1'b0}};
            timestamp_us <= timestamp_us + 1'b1;
        end
        else if (us_div_count >= (US_TICK_CYCLES - 1)) begin
            us_div_count <= {US_COUNT_WIDTH{1'b0}};
            timestamp_us <= timestamp_us + 1'b1;
        end
        else begin
            us_div_count <= us_div_count + 1'b1;
        end
    end

    // -------------------------------------------------------------------------
    // Event Source Record 생성
    // -------------------------------------------------------------------------
    reg [EVENT_SOURCE_COUNT-1:0] event_source_valid;
    reg [(EVENT_SOURCE_COUNT*64)-1:0] event_source_data;
    wire [15:0] a_crc_difference;
    wire [15:0] b_crc_difference;

    assign a_crc_difference = a_calculated_crc ^ a_received_crc;
    assign b_crc_difference = b_calculated_crc ^ b_received_crc;

    always @(*) begin
        event_source_valid = {EVENT_SOURCE_COUNT{1'b0}};
        event_source_data  = {(EVENT_SOURCE_COUNT*64){1'b0}};

        // Source 0: Channel A Byte/Frame/CRC 오류
        if (a_crc_error) begin
            event_source_valid[0] = 1'b1;
            event_source_data[0*64 +: 64] = {
                timestamp_us, EV_CRC_ERROR, CHANNEL_A,
                a_sequence, a_crc_difference[13:0]
            };
        end
        else if (a_length_error) begin
            event_source_valid[0] = 1'b1;
            event_source_data[0*64 +: 64] = {
                timestamp_us, EV_LENGTH_ERROR, CHANNEL_A,
                a_sequence, {6'd0, a_rx_data}
            };
        end
        else if (a_uart_frame_error) begin
            event_source_valid[0] = 1'b1;
            event_source_data[0*64 +: 64] = {
                timestamp_us, EV_UART_FRAME_ERROR, CHANNEL_A,
                a_sequence, 14'd0
            };
        end
        else if (a_interbyte_timeout) begin
            event_source_valid[0] = 1'b1;
            event_source_data[0*64 +: 64] = {
                timestamp_us, EV_INTERBYTE_TIMEOUT, CHANNEL_A,
                a_sequence, 14'd0
            };
        end
        else if (a_frame_timeout) begin
            event_source_valid[0] = 1'b1;
            event_source_data[0*64 +: 64] = {
                timestamp_us, EV_FRAME_TIMEOUT, CHANNEL_A,
                a_sequence, 14'd0
            };
        end

        // Source 1: Channel B Byte/Frame/CRC 오류
        if (b_crc_error) begin
            event_source_valid[1] = 1'b1;
            event_source_data[1*64 +: 64] = {
                timestamp_us, EV_CRC_ERROR, CHANNEL_B,
                b_sequence, b_crc_difference[13:0]
            };
        end
        else if (b_length_error) begin
            event_source_valid[1] = 1'b1;
            event_source_data[1*64 +: 64] = {
                timestamp_us, EV_LENGTH_ERROR, CHANNEL_B,
                b_sequence, {6'd0, b_rx_data}
            };
        end
        else if (b_uart_frame_error) begin
            event_source_valid[1] = 1'b1;
            event_source_data[1*64 +: 64] = {
                timestamp_us, EV_UART_FRAME_ERROR, CHANNEL_B,
                b_sequence, 14'd0
            };
        end
        else if (b_interbyte_timeout) begin
            event_source_valid[1] = 1'b1;
            event_source_data[1*64 +: 64] = {
                timestamp_us, EV_INTERBYTE_TIMEOUT, CHANNEL_B,
                b_sequence, 14'd0
            };
        end
        else if (b_frame_timeout) begin
            event_source_valid[1] = 1'b1;
            event_source_data[1*64 +: 64] = {
                timestamp_us, EV_FRAME_TIMEOUT, CHANNEL_B,
                b_sequence, 14'd0
            };
        end

        // Source 2/3: Sequence 오류
        if (a_seq_gap || a_seq_duplicate || a_seq_old) begin
            event_source_valid[2] = 1'b1;
            event_source_data[2*64 +: 64] = {
                timestamp_us,
                a_seq_gap ? EV_SEQ_GAP :
                (a_seq_duplicate ? EV_SEQ_DUP : EV_SEQ_OLD),
                CHANNEL_A, a_sequence, {6'd0, a_last_rx_seq}
            };
        end

        if (b_seq_gap || b_seq_duplicate || b_seq_old) begin
            event_source_valid[3] = 1'b1;
            event_source_data[3*64 +: 64] = {
                timestamp_us,
                b_seq_gap ? EV_SEQ_GAP :
                (b_seq_duplicate ? EV_SEQ_DUP : EV_SEQ_OLD),
                CHANNEL_B, b_sequence, {6'd0, b_last_rx_seq}
            };
        end

        if (a_channel_timeout_pulse) begin
            event_source_valid[4] = 1'b1;
            event_source_data[4*64 +: 64] = {
                timestamp_us, EV_CHANNEL_TIMEOUT, CHANNEL_A,
                a_timeout_sequence, 14'd0
            };
        end

        if (b_channel_timeout_pulse) begin
            event_source_valid[5] = 1'b1;
            event_source_data[5*64 +: 64] = {
                timestamp_us, EV_CHANNEL_TIMEOUT, CHANNEL_B,
                b_timeout_sequence, 14'd0
            };
        end

        if (matcher_timeout) begin
            event_source_valid[6] = 1'b1;
            event_source_data[6*64 +: 64] = {
                timestamp_us, EV_PAIR_TIMEOUT,
                (matcher_result_kind == RESULT_SINGLE_A) ?
                CHANNEL_A : CHANNEL_B,
                matcher_sequence, 14'd0
            };
        end

        if (matcher_seq_skew || matcher_seq_ambiguous) begin
            event_source_valid[7] = 1'b1;
            event_source_data[7*64 +: 64] = {
                timestamp_us,
                matcher_seq_ambiguous ?
                EV_PAIR_SEQ_AMBIGUOUS : EV_PAIR_SEQ_MISMATCH,
                CHANNEL_BOTH, matcher_sequence,
                {8'd0, matcher_mismatch_flags}
            };
        end

        if (decision_mismatch_drop) begin
            event_source_valid[8] = 1'b1;
            event_source_data[8*64 +: 64] = {
                timestamp_us, EV_DATA_MISMATCH, CHANNEL_BOTH,
                matcher_sequence,
                {8'd0, last_matcher_mismatch_flags}
            };
        end

        if (decision_both_invalid) begin
            event_source_valid[9] = 1'b1;
            event_source_data[9*64 +: 64] = {
                timestamp_us, EV_BOTH_INVALID, CHANNEL_BOTH,
                matcher_sequence, 14'd0
            };
        end

        if (decision_valid && decision_accept && decision_degraded) begin
            event_source_valid[10] = 1'b1;
            event_source_data[10*64 +: 64] = {
                timestamp_us, EV_FRAME_FALLBACK,
                decision_selected_b ? CHANNEL_B : CHANNEL_A,
                decision_sequence, 14'd0
            };
        end

        if (a_fault_enter_pulse || a_recovered_pulse) begin
            event_source_valid[11] = 1'b1;
            event_source_data[11*64 +: 64] = {
                timestamp_us,
                a_fault_enter_pulse ?
                EV_CHANNEL_FAULT : EV_CHANNEL_RECOVERED,
                CHANNEL_A, a_last_rx_seq,
                {6'd0, a_fail_count}
            };
        end

        if (b_fault_enter_pulse || b_recovered_pulse) begin
            event_source_valid[12] = 1'b1;
            event_source_data[12*64 +: 64] = {
                timestamp_us,
                b_fault_enter_pulse ?
                EV_CHANNEL_FAULT : EV_CHANNEL_RECOVERED,
                CHANNEL_B, b_last_rx_seq,
                {6'd0, b_fail_count}
            };
        end

        if (failover_a_to_b_pulse || failover_b_to_a_pulse ||
            recovery_default_pulse) begin
            event_source_valid[13] = 1'b1;
            event_source_data[13*64 +: 64] = {
                timestamp_us,
                recovery_default_pulse ? EV_RECOVERY_DEFAULT :
                (failover_a_to_b_pulse ?
                 EV_FAILOVER_A_TO_B : EV_FAILOVER_B_TO_A),
                CHANNEL_SYSTEM, translation_sequence, 14'd0
            };
        end

        if (a_fifo_overflow || b_fifo_overflow ||
            raw_overflow_pulse || raw_length_error_pulse) begin
            event_source_valid[14] = 1'b1;
            event_source_data[14*64 +: 64] = {
                timestamp_us, EV_FIFO_OVERFLOW,
                a_fifo_overflow ? CHANNEL_A :
                (b_fifo_overflow ? CHANNEL_B : CHANNEL_SYSTEM),
                translation_sequence,
                {12'd0, raw_buffer_full,
                 raw_length_error_pulse}
            };
        end

        if (unsupported_command_pulse || output_blocked_pulse ||
            duplicate_drop) begin
            event_source_valid[15] = 1'b1;
            event_source_data[15*64 +: 64] = {
                timestamp_us,
                unsupported_command_pulse ? EV_UNSUPPORTED_CMD :
                (output_blocked_pulse ?
                 EV_OUTPUT_BLOCKED : EV_DUPLICATE_DROP),
                duplicate_drop ?
                (duplicate_selected_b ? CHANNEL_B : CHANNEL_A) :
                (decision_selected_b ? CHANNEL_B : CHANNEL_A),
                duplicate_drop ? duplicate_sequence : decision_sequence,
                {6'd0,
                 duplicate_drop ? duplicate_command : decision_command}
            };
        end
    end

    // -------------------------------------------------------------------------
    // Event Arbiter / FIFO
    // -------------------------------------------------------------------------
    wire        arbiter_event_valid;
    wire        arbiter_event_ready;
    wire [63:0] arbiter_event_data;
    wire        arbiter_pending_any;
    wire [7:0]  arbiter_pending_count;
    wire        event_lost_pulse;
    wire [15:0] event_lost_count;

    event_arbiter #(
        .SOURCE_COUNT (EVENT_SOURCE_COUNT),
        .EVENT_WIDTH  (64)
    ) u_event_arbiter (
        .clk               (clk),
        .reset_p           (reset_p),
        .statistics_clear  (statistics_clear_pulse),
        .source_valid      (event_source_valid),
        .source_data       (event_source_data),
        .event_valid       (arbiter_event_valid),
        .event_ready       (arbiter_event_ready),
        .event_data        (arbiter_event_data),
        .pending_any       (arbiter_pending_any),
        .pending_count     (arbiter_pending_count),
        .event_lost_pulse  (event_lost_pulse),
        .event_lost_count  (event_lost_count)
    );

    wire        event_front_valid;
    wire [63:0] event_front_data;
    wire        event_fifo_empty;
    wire        event_fifo_full;
    wire [7:0]  event_fifo_count;
    wire        event_fifo_underflow_pulse;
    wire [15:0] event_fifo_underflow_count;

    event_fifo #(
        .EVENT_WIDTH (64),
        .FIFO_DEPTH  (EVENT_FIFO_DEPTH)
    ) u_event_fifo (
        .clk              (clk),
        .reset_p          (reset_p),
        .clear_fifo       (fifo_clear_pulse),
        .statistics_clear (statistics_clear_pulse),
        .event_valid      (arbiter_event_valid),
        .event_ready      (arbiter_event_ready),
        .event_data       (arbiter_event_data),
        .front_valid      (event_front_valid),
        .front_data       (event_front_data),
        .pop_request      (fifo_pop_request),
        .fifo_empty       (event_fifo_empty),
        .fifo_full        (event_fifo_full),
        .event_count      (event_fifo_count),
        .underflow_pulse  (event_fifo_underflow_pulse),
        .underflow_count  (event_fifo_underflow_count)
    );

    // -------------------------------------------------------------------------
    // AXI4-Lite Register Bank
    // -------------------------------------------------------------------------
    axi_lite_regs u_axi_regs (
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
        .recovery_count             (recovery_count_cfg),
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
        .frame_mismatch_event       (decision_mismatch_drop),
        .both_invalid_event         (decision_both_invalid),
        .channel_fault_event        (a_fault_enter_pulse |
                                     b_fault_enter_pulse),
        .fifo_underflow_pulse       (event_fifo_underflow_pulse),
        .irq                        (irq)
    );

    // -------------------------------------------------------------------------
    // Basys3 Status Display
    // -------------------------------------------------------------------------
    status_display #(
        .SCAN_TICK_CYCLES        (SCAN_TICK_CYCLES),
        .HEARTBEAT_TOGGLE_CYCLES (HEARTBEAT_CYCLES),
        .ALERT_HOLD_CYCLES        (ALERT_HOLD_CYCLES)
    ) u_status_display (
        .clk                     (clk),
        .reset_p                 (reset_p),
        .system_enable           (system_enable),
        .channel_a_alive         (channel_a_alive),
        .channel_b_alive         (channel_b_alive),
        .a_fault                 (a_fault),
        .b_fault                 (b_fault),
        .pair_wait_active        (pair_wait_active),
        .output_busy             (output_busy),
        .event_fifo_not_empty    (!event_fifo_empty),
        .frame_mismatch_latched  (frame_mismatch_latched),
        .both_invalid_latched    (both_invalid_latched),
        .event_lost_latched      (event_lost_count != 16'd0),
        .irq                     (irq),
        .output_frame_valid      (translated_frame_fire),
        .output_sequence         (translation_sequence),
        // 같은 Frame의 valid와 선택 채널을 함께 전달해 FND가 이전
        // Frame의 채널을 저장하지 않도록 한다.
        .last_selected_b         (translation_selected_b),
        .duplicate_drop_pulse    (duplicate_drop),
        .led                     (led),
        .seg                     (seg),
        .dp                      (dp),
        .an                      (an)
    );

endmodule
