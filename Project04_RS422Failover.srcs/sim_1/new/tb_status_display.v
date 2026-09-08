`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// status_display Self-checking Testbench
//
// 검증 항목
//   1. Reset 중 LED/FND가 모두 꺼지는지
//   2. System Disable에서 LED가 모두 꺼지고 "0FF "가 표시되는지
//   3. Healthy/Single A/Single B/Both Fault Mode 표시가 맞는지
//   4. 마지막 선택 채널과 8-bit Sequence가 정상적으로 Latch되는지
//   5. Disable 후 재활성화하면 이전 Frame 표시가 제거되는지
//   6. LED[1] 종합 Alert와 LED[15:2] 고정 0 Mapping이 정확한지
//   7. Duplicate Alert가 설정한 시간만큼 유지되고 재Trigger되는지
//   8. LED[0] Heartbeat가 Enable에서만 Toggle되고 Disable에서 Clear되는지
//   9. FND Anode가 Active-Low One-Hot으로 순환하고 DP가 꺼져 있는지
//////////////////////////////////////////////////////////////////////////////////

module tb_status_display;

    localparam [6:0] SEG_0     = 7'b1000000;
    localparam [6:0] SEG_5     = 7'b0010010;
    localparam [6:0] SEG_A     = 7'b0001000;
    localparam [6:0] SEG_B     = 7'b0000011;
    localparam [6:0] SEG_D     = 7'b0100001;
    localparam [6:0] SEG_F     = 7'b0001110;
    localparam [6:0] SEG_DASH  = 7'b0111111;
    localparam [6:0] SEG_BLANK = 7'b1111111;

    reg        clk;
    reg        reset_p;

    reg        system_enable;
    reg        channel_a_alive;
    reg        channel_b_alive;
    reg        a_fault;
    reg        b_fault;
    reg        pair_wait_active;
    reg        output_busy;
    reg        event_fifo_not_empty;
    reg        frame_mismatch_latched;
    reg        both_invalid_latched;
    reg        event_lost_latched;
    reg        irq;

    reg        output_frame_valid;
    reg [7:0]  output_sequence;
    reg        last_selected_b;
    reg        duplicate_drop_pulse;

    wire [15:0] led;
    wire [6:0]  seg;
    wire        dp;
    wire [3:0]  an;

    integer error_count;
    integer monitor_index;
    integer heartbeat_before;

    status_display #(
        .SCAN_TICK_CYCLES        (2),
        .HEARTBEAT_TOGGLE_CYCLES (4),
        .ALERT_HOLD_CYCLES        (3)
    ) dut (
        .clk                    (clk),
        .reset_p                (reset_p),

        .system_enable          (system_enable),
        .channel_a_alive        (channel_a_alive),
        .channel_b_alive        (channel_b_alive),
        .a_fault                (a_fault),
        .b_fault                (b_fault),
        .pair_wait_active       (pair_wait_active),
        .output_busy            (output_busy),
        .event_fifo_not_empty   (event_fifo_not_empty),
        .frame_mismatch_latched (frame_mismatch_latched),
        .both_invalid_latched   (both_invalid_latched),
        .event_lost_latched     (event_lost_latched),
        .irq                    (irq),

        .output_frame_valid     (output_frame_valid),
        .output_sequence        (output_sequence),
        .last_selected_b        (last_selected_b),
        .duplicate_drop_pulse   (duplicate_drop_pulse),

        .led                    (led),
        .seg                    (seg),
        .dp                     (dp),
        .an                     (an)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    task report_error;
        input [8*80-1:0] error_text;
        begin
            error_count = error_count + 1;
            $display("[FAIL] time=%0t %0s", $time, error_text);
        end
    endtask

    task apply_reset;
        begin
            reset_p = 1'b1;
            repeat (2) @(negedge clk);

            #1;
            if (led !== 16'h0000)
                report_error("LED must be all zero during reset");

            if ((an !== 4'b1111) ||
                (seg !== SEG_BLANK) ||
                (dp !== 1'b1))
                report_error("FND must be blank during reset");

            reset_p = 1'b0;
            @(negedge clk);
        end
    endtask

    // 특정 자리의 Anode가 활성화될 때 Segment 값을 확인한다.
    task expect_digit;
        input [3:0] expected_an;
        input [6:0] expected_seg;
        input [8*40-1:0] digit_name;

        integer attempt;
        integer found;
        begin
            found = 0;

            for (attempt = 0; attempt < 12; attempt = attempt + 1) begin
                @(negedge clk);
                #1;

                if (!found && (an === expected_an)) begin
                    found = 1;

                    if (seg !== expected_seg) begin
                        error_count = error_count + 1;
                        $display(
                            "[FAIL] time=%0t %0s seg=%b expected=%b",
                            $time,
                            digit_name,
                            seg,
                            expected_seg
                        );
                    end

                    if (dp !== 1'b1)
                        report_error("Decimal point must remain off");
                end
            end

            if (!found) begin
                error_count = error_count + 1;
                $display(
                    "[FAIL] time=%0t %0s anode %b was not scanned",
                    $time,
                    digit_name,
                    expected_an
                );
            end
        end
    endtask

    task expect_display;
        input [6:0] expected_digit_3;
        input [6:0] expected_digit_2;
        input [6:0] expected_digit_1;
        input [6:0] expected_digit_0;
        begin
            expect_digit(4'b0111, expected_digit_3, "digit 3");
            expect_digit(4'b1011, expected_digit_2, "digit 2");
            expect_digit(4'b1101, expected_digit_1, "digit 1");
            expect_digit(4'b1110, expected_digit_0, "digit 0");
        end
    endtask

    task send_output_frame;
        input       selected_b;
        input [7:0] frame_sequence;
        begin
            @(negedge clk);
            last_selected_b    = selected_b;
            output_sequence    = frame_sequence;
            output_frame_valid = 1'b1;

            @(negedge clk);
            output_frame_valid = 1'b0;
        end
    endtask

    task send_duplicate_pulse;
        begin
            @(negedge clk);
            duplicate_drop_pulse = 1'b1;

            @(negedge clk);
            duplicate_drop_pulse = 1'b0;
        end
    endtask

    task expect_led_mask;
        input [15:0] mask;
        input [15:0] expected_value;
        input [8*60-1:0] check_name;
        begin
            #1;
            if ((led & mask) !== (expected_value & mask)) begin
                error_count = error_count + 1;
                $display(
                    "[FAIL] time=%0t %0s led=%h mask=%h expected=%h",
                    $time,
                    check_name,
                    led,
                    mask,
                    expected_value
                );
            end
        end
    endtask

    // 모든 테스트 상태에서 사용하지 않는 LED는 항상 꺼져 있어야 한다.
    always @(negedge clk) begin
        #1;
        if (led[15:2] !== 14'd0)
            report_error("LED[15:2] must always remain zero");
    end

    initial begin
        reset_p                = 1'b1;
        system_enable          = 1'b0;
        channel_a_alive        = 1'b0;
        channel_b_alive        = 1'b0;
        a_fault                = 1'b0;
        b_fault                = 1'b0;
        pair_wait_active       = 1'b0;
        output_busy            = 1'b0;
        event_fifo_not_empty   = 1'b0;
        frame_mismatch_latched = 1'b0;
        both_invalid_latched   = 1'b0;
        event_lost_latched     = 1'b0;
        irq                     = 1'b0;
        output_frame_valid      = 1'b0;
        output_sequence         = 8'd0;
        last_selected_b         = 1'b0;
        duplicate_drop_pulse    = 1'b0;
        error_count             = 0;
        monitor_index           = 0;
        heartbeat_before        = 0;

        // ---------------------------------------------------------------------
        // 1. Reset 및 Disable 표시
        // ---------------------------------------------------------------------
        apply_reset;
        expect_display(SEG_0, SEG_F, SEG_F, SEG_BLANK);
        expect_led_mask(16'hFFFF, 16'h0000, "all LEDs off when disabled");

        // ---------------------------------------------------------------------
        // 2. Enable 직후 Wait Sync, 이후 Dual Healthy
        // ---------------------------------------------------------------------
        @(negedge clk);
        system_enable = 1'b1;
        expect_led_mask(16'hFFFE, 16'h0000, "alert off while waiting sync");
        expect_display(SEG_DASH, SEG_DASH, SEG_DASH, SEG_DASH);

        @(negedge clk);
        channel_a_alive = 1'b1;
        channel_b_alive = 1'b1;
        expect_led_mask(16'hFFFE, 16'h0000, "alert off while dual healthy");
        expect_display(SEG_D, SEG_DASH, SEG_DASH, SEG_DASH);

        // Fault 확정 전이라도 한 채널만 Alive이면 Degraded로 표시한다.
        @(negedge clk);
        channel_b_alive = 1'b0;
        expect_led_mask(16'hFFFE, 16'h0000, "alert off without fault");
        expect_display(SEG_A, SEG_DASH, SEG_DASH, SEG_DASH);

        @(negedge clk);
        channel_b_alive = 1'b1;

        // ---------------------------------------------------------------------
        // 3. B가 선택된 SEQ=A5 Frame을 실제 출력
        // ---------------------------------------------------------------------
        send_output_frame(1'b1, 8'hA5);
        expect_display(SEG_D, SEG_B, SEG_A, SEG_5);

        // FND가 보관한 값은 입력이 변해도 다음 Frame Pulse 전에는 유지돼야 한다.
        @(negedge clk);
        last_selected_b = 1'b0;
        output_sequence = 8'h00;
        expect_display(SEG_D, SEG_B, SEG_A, SEG_5);

        // ---------------------------------------------------------------------
        // 4. Health Mode 우선순위
        // ---------------------------------------------------------------------
        @(negedge clk);
        b_fault         = 1'b1;
        channel_b_alive = 1'b0;
        expect_display(SEG_A, SEG_B, SEG_A, SEG_5);
        expect_led_mask(
            16'hFFFE,
            16'h0002,
            "B fault raises aggregate alert"
        );

        @(negedge clk);
        a_fault         = 1'b1;
        b_fault         = 1'b0;
        channel_a_alive = 1'b0;
        channel_b_alive = 1'b1;
        expect_display(SEG_B, SEG_B, SEG_A, SEG_5);
        expect_led_mask(
            16'hFFFE,
            16'h0002,
            "A fault raises aggregate alert"
        );

        @(negedge clk);
        b_fault         = 1'b1;
        channel_b_alive = 1'b0;
        expect_display(SEG_F, SEG_B, SEG_A, SEG_5);
        expect_led_mask(
            16'hFFFE,
            16'h0002,
            "both fault raises aggregate alert"
        );

        // ---------------------------------------------------------------------
        // 5. Latched Error별 종합 Alert
        // ---------------------------------------------------------------------
        @(negedge clk);
        a_fault                = 1'b0;
        b_fault                = 1'b0;
        channel_a_alive        = 1'b1;
        channel_b_alive        = 1'b1;
        pair_wait_active       = 1'b1;
        output_busy            = 1'b1;
        event_fifo_not_empty   = 1'b1;
        irq                     = 1'b1;

        expect_led_mask(
            16'hFFFE,
            16'h0000,
            "non-alert status inputs do not drive LEDs"
        );

        frame_mismatch_latched = 1'b1;
        expect_led_mask(
            16'hFFFE,
            16'h0002,
            "frame mismatch raises aggregate alert"
        );

        frame_mismatch_latched = 1'b0;
        both_invalid_latched   = 1'b1;
        expect_led_mask(
            16'hFFFE,
            16'h0002,
            "both invalid raises aggregate alert"
        );

        both_invalid_latched = 1'b0;
        event_lost_latched   = 1'b1;
        expect_led_mask(
            16'hFFFE,
            16'h0002,
            "event lost raises aggregate alert"
        );

        event_lost_latched = 1'b0;
        expect_led_mask(16'hFFFE, 16'h0000, "latched alerts clear");

        // ---------------------------------------------------------------------
        // 6. Duplicate Pulse Stretch 및 재Trigger
        // ---------------------------------------------------------------------
        send_duplicate_pulse;
        expect_led_mask(16'hFFFE, 16'h0002, "duplicate alert hold cycle 1");

        @(negedge clk);
        expect_led_mask(16'hFFFE, 16'h0002, "duplicate alert hold cycle 2");

        @(negedge clk);
        expect_led_mask(16'hFFFE, 16'h0002, "duplicate alert hold cycle 3");

        @(negedge clk);
        expect_led_mask(16'hFFFE, 16'h0000, "duplicate alert expires");

        send_duplicate_pulse;
        expect_led_mask(16'hFFFE, 16'h0002, "duplicate alert restarts");

        @(negedge clk);
        expect_led_mask(16'hFFFE, 16'h0002, "duplicate restart hold 2");

        // 유지 중 재Trigger하면 3 Cycle을 다시 센다.
        send_duplicate_pulse;
        expect_led_mask(16'hFFFE, 16'h0002, "duplicate alert retrigger");

        @(negedge clk);
        expect_led_mask(16'hFFFE, 16'h0002, "duplicate retrigger hold 2");

        @(negedge clk);
        expect_led_mask(16'hFFFE, 16'h0002, "duplicate retrigger hold 3");

        @(negedge clk);
        expect_led_mask(16'hFFFE, 16'h0000, "retriggered alert expires");

        // ---------------------------------------------------------------------
        // 7. Heartbeat: Enable에서 Toggle, Disable에서 즉시 Clear
        // ---------------------------------------------------------------------
        heartbeat_before = led[0];
        repeat (4) @(posedge clk);
        #1;

        if (led[0] === heartbeat_before)
            report_error("heartbeat must toggle after configured cycles");

        @(negedge clk);
        system_enable          = 1'b0;
        a_fault                = 1'b1;
        b_fault                = 1'b1;
        frame_mismatch_latched = 1'b1;
        both_invalid_latched   = 1'b1;
        event_lost_latched     = 1'b1;
        @(posedge clk);
        #1;

        if (led !== 16'h0000)
            report_error("all LEDs must clear while system is disabled");

        // Disable은 마지막 Frame 표시도 Clear한다.
        @(negedge clk);
        system_enable          = 1'b1;
        a_fault                = 1'b0;
        b_fault                = 1'b0;
        frame_mismatch_latched = 1'b0;
        both_invalid_latched   = 1'b0;
        event_lost_latched     = 1'b0;
        expect_display(SEG_D, SEG_DASH, SEG_DASH, SEG_DASH);

        // ---------------------------------------------------------------------
        // 8. FND Scan Anode는 네 가지 Active-Low One-Hot 값만 사용
        // ---------------------------------------------------------------------
        for (monitor_index = 0;
             monitor_index < 16;
             monitor_index = monitor_index + 1) begin

            @(negedge clk);
            #1;

            if ((an !== 4'b1110) &&
                (an !== 4'b1101) &&
                (an !== 4'b1011) &&
                (an !== 4'b0111))
                report_error("invalid FND active-low anode pattern");

            if (dp !== 1'b1)
                report_error("decimal point must remain off");
        end

        if (error_count == 0)
            $display("[PASS] All status_display core tests passed.");
        else
            $display("[FAIL] error_count=%0d", error_count);

        $finish;
    end

    initial begin
        #20_000;
        $display("[FAIL] Simulation timeout");
        $finish;
    end

endmodule
