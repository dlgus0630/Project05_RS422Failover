`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// status_display
//
// 역할
//   이중화 통신 Core의 운용 상태를 Basys3 LED 16개와 4자리 FND에 표시한다.
//   CPU나 AXI-Lite 소프트웨어를 거치지 않는 순수 RTL 표시 경로이므로
//   MicroBlaze가 Halt되어도 현재 상태를 계속 확인할 수 있다.
//
// LED Mapping
//   LED[0]    : Heartbeat, System Enable일 때만 점멸
//   LED[1]    : 종합 Alert
//               A/B Fault, Frame Mismatch, Both Invalid,
//               Duplicate Drop 또는 Event Lost 중 하나 이상일 때 점등
//   LED[15:2] : 항상 꺼짐
//
// FND 표시
//   비활성 상태 : "0FF "  (7-Segment에서 0을 O처럼 사용)
//   활성 상태   : [Mode][Last Channel][Last Output Sequence(HEX 2자리)]
//
//   Mode
//     d : A/B 모두 Healthy
//     A : A만 사용 가능
//     b : B만 사용 가능
//     F : A/B 모두 Fault
//     - : 아직 동기화 전이거나 사용 가능한 채널 없음
//
//   Last Channel
//     A 또는 b. 아직 출력 프레임이 없으면 '-'.
//
// Basys3 FND는 Segment와 Anode가 모두 Active-Low이다.
//   seg[6:0] = {g, f, e, d, c, b, a}
//   an[3]은 왼쪽 자리, an[0]은 오른쪽 자리이다.
//////////////////////////////////////////////////////////////////////////////////

module status_display #(
    // 각 FND 자리를 유지하는 Clock Cycle 수
    // 100 MHz에서 100,000 Cycle = 자리당 1 ms, 전체 Refresh 250 Hz
    parameter integer SCAN_TICK_CYCLES       = 100_000,

    // Heartbeat LED가 한 번 Toggle되는 Clock Cycle 수
    // 100 MHz에서 50,000,000 Cycle = 0.5초마다 Toggle
    parameter integer HEARTBEAT_TOGGLE_CYCLES = 50_000_000,

    // 1클럭 Duplicate Pulse를 눈으로 볼 수 있게 유지하는 Clock Cycle 수
    // 100 MHz에서 50,000,000 Cycle = 0.5초
    parameter integer ALERT_HOLD_CYCLES       = 50_000_000
)(
    input  wire        clk,
    input  wire        reset_p,

    // 현재 운용 상태
    input  wire        system_enable,
    input  wire        channel_a_alive,
    input  wire        channel_b_alive,
    input  wire        a_fault,
    input  wire        b_fault,
    input  wire        pair_wait_active,
    input  wire        output_busy,
    input  wire        event_fifo_not_empty,
    input  wire        frame_mismatch_latched,
    input  wire        both_invalid_latched,
    input  wire        event_lost_latched,
    input  wire        irq,

    // duplicate_guard.out_valid/out_sequence와 연결
    input  wire        output_frame_valid,
    input  wire [7:0]  output_sequence,

    // 최종 선택 채널의 현재 상태값
    input  wire        last_selected_b,

    // duplicate_guard.duplicate_drop과 연결되는 1클럭 Pulse
    input  wire        duplicate_drop_pulse,

    // Basys3 출력
    output reg  [15:0] led,
    output reg  [6:0]  seg,
    output reg         dp,
    output reg  [3:0]  an
);

    localparam integer SCAN_COUNTER_WIDTH =
        (SCAN_TICK_CYCLES <= 1) ? 1 : $clog2(SCAN_TICK_CYCLES);

    localparam integer HEARTBEAT_COUNTER_WIDTH =
        (HEARTBEAT_TOGGLE_CYCLES <= 1) ?
        1 : $clog2(HEARTBEAT_TOGGLE_CYCLES);

    localparam integer ALERT_COUNTER_WIDTH =
        (ALERT_HOLD_CYCLES <= 0) ?
        1 : $clog2(ALERT_HOLD_CYCLES + 1);

    // FND Symbol Code
    localparam [4:0] SYMBOL_0     = 5'd0;
    localparam [4:0] SYMBOL_A     = 5'd10;
    localparam [4:0] SYMBOL_B     = 5'd11;
    localparam [4:0] SYMBOL_D     = 5'd13;
    localparam [4:0] SYMBOL_F     = 5'd15;
    localparam [4:0] SYMBOL_BLANK = 5'd16;
    localparam [4:0] SYMBOL_DASH  = 5'd17;

    reg [SCAN_COUNTER_WIDTH-1:0]      scan_counter;
    reg [1:0]                         scan_index;

    reg [HEARTBEAT_COUNTER_WIDTH-1:0] heartbeat_counter;
    reg                               heartbeat_state;

    reg [ALERT_COUNTER_WIDTH-1:0]     duplicate_hold_count;

    reg                               last_frame_valid;
    reg                               stored_selected_b;
    reg [7:0]                         stored_sequence;

    reg [4:0] digit_3_symbol;
    reg [4:0] digit_2_symbol;
    reg [4:0] digit_1_symbol;
    reg [4:0] digit_0_symbol;
    reg [4:0] active_symbol;

    wire duplicate_indicator;
    wire channel_a_usable;
    wire channel_b_usable;

    assign duplicate_indicator = (duplicate_hold_count != 0);
    assign channel_a_usable = channel_a_alive && !a_fault;
    assign channel_b_usable = channel_b_alive && !b_fault;

    // -------------------------------------------------------------------------
    // FND Scan Tick
    // -------------------------------------------------------------------------
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            scan_counter <= {SCAN_COUNTER_WIDTH{1'b0}};
            scan_index   <= 2'd0;
        end
        else begin
            if (SCAN_TICK_CYCLES <= 1) begin
                scan_counter <= {SCAN_COUNTER_WIDTH{1'b0}};
                scan_index   <= scan_index + 1'b1;
            end
            else if (scan_counter >= (SCAN_TICK_CYCLES - 1)) begin
                scan_counter <= {SCAN_COUNTER_WIDTH{1'b0}};
                scan_index   <= scan_index + 1'b1;
            end
            else begin
                scan_counter <= scan_counter + 1'b1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // System Heartbeat
    // -------------------------------------------------------------------------
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            heartbeat_counter <= {HEARTBEAT_COUNTER_WIDTH{1'b0}};
            heartbeat_state   <= 1'b0;
        end
        else if (!system_enable) begin
            heartbeat_counter <= {HEARTBEAT_COUNTER_WIDTH{1'b0}};
            heartbeat_state   <= 1'b0;
        end
        else begin
            if (HEARTBEAT_TOGGLE_CYCLES <= 1) begin
                heartbeat_counter <= {HEARTBEAT_COUNTER_WIDTH{1'b0}};
                heartbeat_state   <= ~heartbeat_state;
            end
            else if (heartbeat_counter >=
                     (HEARTBEAT_TOGGLE_CYCLES - 1)) begin
                heartbeat_counter <= {HEARTBEAT_COUNTER_WIDTH{1'b0}};
                heartbeat_state   <= ~heartbeat_state;
            end
            else begin
                heartbeat_counter <= heartbeat_counter + 1'b1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Duplicate Drop Pulse Stretch
    // -------------------------------------------------------------------------
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            duplicate_hold_count <= {ALERT_COUNTER_WIDTH{1'b0}};
        end
        else if (duplicate_drop_pulse && (ALERT_HOLD_CYCLES > 0)) begin
            // 유지 중 다시 Pulse가 오면 처음부터 다시 유지한다.
            duplicate_hold_count <= ALERT_HOLD_CYCLES;
        end
        else if (duplicate_hold_count != 0) begin
            duplicate_hold_count <= duplicate_hold_count - 1'b1;
        end
    end

    // -------------------------------------------------------------------------
    // 마지막으로 실제 출력된 Frame 정보 보관
    // -------------------------------------------------------------------------
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            last_frame_valid  <= 1'b0;
            stored_selected_b <= 1'b0;
            stored_sequence   <= 8'd0;
        end
        else if (!system_enable) begin
            // 재활성화 후 이전 운용의 Frame을 현재 Frame처럼 표시하지 않는다.
            last_frame_valid  <= 1'b0;
            stored_selected_b <= 1'b0;
            stored_sequence   <= 8'd0;
        end
        else if (output_frame_valid) begin
            last_frame_valid  <= 1'b1;
            stored_selected_b <= last_selected_b;
            stored_sequence   <= output_sequence;
        end
    end

    // -------------------------------------------------------------------------
    // LED 상태 조합
    // -------------------------------------------------------------------------
    always @(*) begin
        led = 16'd0;

        if (!reset_p) begin
            led[0] = heartbeat_state;
            led[1] = system_enable &&
                     (
                         a_fault                ||
                         b_fault                ||
                         frame_mismatch_latched ||
                         both_invalid_latched   ||
                         duplicate_indicator   ||
                         event_lost_latched
                     );
        end
    end

    // -------------------------------------------------------------------------
    // 표시할 4자리 Symbol 결정
    // -------------------------------------------------------------------------
    always @(*) begin
        digit_3_symbol = SYMBOL_DASH;
        digit_2_symbol = SYMBOL_DASH;
        digit_1_symbol = SYMBOL_DASH;
        digit_0_symbol = SYMBOL_DASH;

        if (!system_enable) begin
            // "0FF " : 7-Segment의 0을 O처럼 사용한다.
            digit_3_symbol = SYMBOL_0;
            digit_2_symbol = SYMBOL_F;
            digit_1_symbol = SYMBOL_F;
            digit_0_symbol = SYMBOL_BLANK;
        end
        else begin
            if (a_fault && b_fault)
                digit_3_symbol = SYMBOL_F;
            else if (channel_a_usable && channel_b_usable)
                digit_3_symbol = SYMBOL_D;
            else if (channel_a_usable)
                digit_3_symbol = SYMBOL_A;
            else if (channel_b_usable)
                digit_3_symbol = SYMBOL_B;
            else
                digit_3_symbol = SYMBOL_DASH;

            if (last_frame_valid) begin
                digit_2_symbol =
                    stored_selected_b ? SYMBOL_B : SYMBOL_A;
                digit_1_symbol = {1'b0, stored_sequence[7:4]};
                digit_0_symbol = {1'b0, stored_sequence[3:0]};
            end
        end
    end

    // -------------------------------------------------------------------------
    // Active-Low FND Multiplexing
    // -------------------------------------------------------------------------
    always @(*) begin
        an            = 4'b1111;
        seg           = 7'b1111111;
        dp            = 1'b1;
        active_symbol = SYMBOL_BLANK;

        if (!reset_p) begin
            case (scan_index)
                2'd0: begin
                    an            = 4'b1110;
                    active_symbol = digit_0_symbol;
                end

                2'd1: begin
                    an            = 4'b1101;
                    active_symbol = digit_1_symbol;
                end

                2'd2: begin
                    an            = 4'b1011;
                    active_symbol = digit_2_symbol;
                end

                2'd3: begin
                    an            = 4'b0111;
                    active_symbol = digit_3_symbol;
                end

                default: begin
                    an            = 4'b1111;
                    active_symbol = SYMBOL_BLANK;
                end
            endcase

            seg = symbol_to_segments(active_symbol);
        end
    end

    // Active-Low 7-Segment Font
    function [6:0] symbol_to_segments;
        input [4:0] symbol;
        begin
            case (symbol)
                5'd0:  symbol_to_segments = 7'b1000000;
                5'd1:  symbol_to_segments = 7'b1111001;
                5'd2:  symbol_to_segments = 7'b0100100;
                5'd3:  symbol_to_segments = 7'b0110000;
                5'd4:  symbol_to_segments = 7'b0011001;
                5'd5:  symbol_to_segments = 7'b0010010;
                5'd6:  symbol_to_segments = 7'b0000010;
                5'd7:  symbol_to_segments = 7'b1111000;
                5'd8:  symbol_to_segments = 7'b0000000;
                5'd9:  symbol_to_segments = 7'b0010000;
                5'd10: symbol_to_segments = 7'b0001000; // A
                5'd11: symbol_to_segments = 7'b0000011; // b
                5'd12: symbol_to_segments = 7'b1000110; // C
                5'd13: symbol_to_segments = 7'b0100001; // d
                5'd14: symbol_to_segments = 7'b0000110; // E
                5'd15: symbol_to_segments = 7'b0001110; // F
                5'd17: symbol_to_segments = 7'b0111111; // -
                default:
                    symbol_to_segments = 7'b1111111;     // Blank
            endcase
        end
    endfunction

endmodule
