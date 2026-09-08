`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// tb_event_arbiter
//
// Self-checking 항목
//   1. 단일 Event 전달
//   2. 동시 Event 보관 및 Source 번호 우선순위
//   3. event_ready=0일 때 event_valid/event_data 유지
//   4. Event 전달과 같은 클럭의 동일 Source 새 Event 교체
//   5. Pending 중 동일 Source 재발생 시 기존 Event 보존 및 유실 검출
//   6. 한 클럭에 여러 Source가 충돌할 때 정확한 유실 개수 누적
//////////////////////////////////////////////////////////////////////////////////

module tb_event_arbiter;

    // 실제 Core 구성과 같은 16개 Source로 검증한다.
    localparam integer SOURCE_COUNT = 16;
    localparam integer EVENT_WIDTH  = 64;

    localparam [63:0] EVENT_A = 64'h0000_0001_AA00_0001;
    localparam [63:0] EVENT_B = 64'h0000_0002_BB00_0002;
    localparam [63:0] EVENT_C = 64'h0000_0003_CC00_0003;
    localparam [63:0] EVENT_D = 64'h0000_0004_DD00_0004;
    localparam [63:0] EVENT_E = 64'h0000_0005_EE00_0005;
    localparam [63:0] EVENT_F = 64'h0000_0006_FF00_0006;

    reg clk;
    reg reset_p;
    reg statistics_clear;

    reg  [SOURCE_COUNT-1:0] source_valid;
    reg  [(SOURCE_COUNT*EVENT_WIDTH)-1:0] source_data;

    wire                    event_valid;
    reg                     event_ready;
    wire [EVENT_WIDTH-1:0]  event_data;

    wire                    pending_any;
    wire [7:0]              pending_count;
    wire                    event_lost_pulse;
    wire [15:0]             event_lost_count;

    integer error_count;

    event_arbiter #(
        .SOURCE_COUNT(SOURCE_COUNT),
        .EVENT_WIDTH (EVENT_WIDTH)
    ) dut (
        .clk              (clk),
        .reset_p          (reset_p),
        .statistics_clear (statistics_clear),
        .source_valid     (source_valid),
        .source_data      (source_data),
        .event_valid      (event_valid),
        .event_ready      (event_ready),
        .event_data       (event_data),
        .pending_any      (pending_any),
        .pending_count    (pending_count),
        .event_lost_pulse (event_lost_pulse),
        .event_lost_count (event_lost_count)
    );

    always #5 clk = ~clk;

    task check_bit;
        input actual;
        input expected;
        input [8*80-1:0] check_name;
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
        input [8*80-1:0] check_name;
        begin
            if (actual !== expected) begin
                $display("[FAIL] %0s: expected=%0d actual=%0d",
                         check_name, expected, actual);
                error_count = error_count + 1;
            end
        end
    endtask

    task check_u16;
        input [15:0] actual;
        input [15:0] expected;
        input [8*80-1:0] check_name;
        begin
            if (actual !== expected) begin
                $display("[FAIL] %0s: expected=%0d actual=%0d",
                         check_name, expected, actual);
                error_count = error_count + 1;
            end
        end
    endtask

    task check_event_data;
        input [63:0] actual;
        input [63:0] expected;
        input [8*80-1:0] check_name;
        begin
            if (actual !== expected) begin
                $display("[FAIL] %0s: expected=%h actual=%h",
                         check_name, expected, actual);
                error_count = error_count + 1;
            end
        end
    endtask

    // 지정한 Source들의 Event를 정확히 1클럭 Pulse로 입력한다.
    task pulse_sources;
        input [SOURCE_COUNT-1:0] valid_bits;
        input [63:0] data0;
        input [63:0] data1;
        input [63:0] data2;
        input [63:0] data3;
        begin
            @(negedge clk);
            source_data[0*EVENT_WIDTH +: EVENT_WIDTH] = data0;
            source_data[1*EVENT_WIDTH +: EVENT_WIDTH] = data1;
            source_data[2*EVENT_WIDTH +: EVENT_WIDTH] = data2;
            source_data[3*EVENT_WIDTH +: EVENT_WIDTH] = data3;
            source_valid = valid_bits;

            @(negedge clk);
            source_valid = {SOURCE_COUNT{1'b0}};
        end
    endtask

    // 현재 가장 높은 우선순위 Event를 1개 Handshake한다.
    task consume_expect;
        input [63:0] expected_data;
        input [8*80-1:0] check_name;
        begin
            event_ready = 1'b1;

            while (event_valid !== 1'b1)
                @(negedge clk);

            check_event_data(event_data, expected_data, check_name);

            // 다음 상승 에지에서 event_valid && event_ready Handshake
            @(posedge clk);
            #1;
            event_ready = 1'b0;

            @(negedge clk);
        end
    endtask

    initial begin
        clk          = 1'b0;
        reset_p      = 1'b1;
        statistics_clear = 1'b0;
        source_valid = {SOURCE_COUNT{1'b0}};
        source_data  = {(SOURCE_COUNT*EVENT_WIDTH){1'b0}};
        event_ready  = 1'b0;
        error_count  = 0;

        repeat (3) @(posedge clk);
        #1;
        reset_p = 1'b0;
        @(negedge clk);

        // ---------------------------------------------------------------------
        // TEST 1: Reset 및 단일 Event
        // ---------------------------------------------------------------------
        check_bit(event_valid, 1'b0, "reset event_valid");
        check_bit(pending_any, 1'b0, "reset pending_any");
        check_u8(pending_count, 8'd0, "reset pending_count");
        check_u16(event_lost_count, 16'd0, "reset lost_count");

        pulse_sources(4'b0100, 64'd0, 64'd0, EVENT_C, 64'd0);

        check_bit(event_valid, 1'b1, "single event_valid");
        check_event_data(event_data, EVENT_C, "single source2 data");
        check_u8(pending_count, 8'd1, "single pending_count");

        consume_expect(EVENT_C, "single consume");
        check_bit(event_valid, 1'b0, "single drained");
        check_u8(pending_count, 8'd0, "single drained count");

        // ---------------------------------------------------------------------
        // TEST 2: 동시 Event는 모두 저장하고 Source 0부터 출력
        // ---------------------------------------------------------------------
        pulse_sources(4'b1011, EVENT_A, EVENT_B, 64'd0, EVENT_D);

        check_u8(pending_count, 8'd3, "simultaneous pending_count");
        check_event_data(event_data, EVENT_A, "priority source0 first");

        consume_expect(EVENT_A, "simultaneous consume source0");
        consume_expect(EVENT_B, "simultaneous consume source1");
        consume_expect(EVENT_D, "simultaneous consume source3");

        check_bit(pending_any, 1'b0, "simultaneous all drained");
        check_u16(event_lost_count, 16'd0,
                  "simultaneous different sources no loss");

        // ---------------------------------------------------------------------
        // TEST 3: Backpressure 동안 출력 유지
        // ---------------------------------------------------------------------
        event_ready = 1'b0;
        pulse_sources(4'b0010, 64'd0, EVENT_B, 64'd0, 64'd0);

        repeat (3) begin
            @(negedge clk);
            check_bit(event_valid, 1'b1, "backpressure valid held");
            check_event_data(event_data, EVENT_B,
                             "backpressure data held");
            check_u8(pending_count, 8'd1,
                     "backpressure pending held");
        end

        consume_expect(EVENT_B, "backpressure release");
        check_bit(event_valid, 1'b0, "backpressure drained");

        // ---------------------------------------------------------------------
        // TEST 4: 기존 Event 전달과 같은 클럭에 같은 Source 새 Event 입력
        // ---------------------------------------------------------------------
        event_ready = 1'b0;
        pulse_sources(4'b0001, EVENT_A, 64'd0, 64'd0, 64'd0);

        @(negedge clk);
        check_event_data(event_data, EVENT_A, "replace old event visible");
        event_ready = 1'b1;
        source_data[0*EVENT_WIDTH +: EVENT_WIDTH] = EVENT_E;
        source_valid = 4'b0001;

        // 상승 에지에서 EVENT_A는 전달되고 EVENT_E는 같은 Slot에 저장된다.
        @(posedge clk);
        #1;
        source_valid = 4'b0000;
        event_ready  = 1'b0;

        @(negedge clk);
        check_bit(event_valid, 1'b1, "replace new event pending");
        check_event_data(event_data, EVENT_E, "replace new event data");
        check_u16(event_lost_count, 16'd0, "replace no loss");

        consume_expect(EVENT_E, "replace consume new event");

        // ---------------------------------------------------------------------
        // TEST 5: Pending 중 동일 Source 재발생은 새 Event 유실 처리
        // ---------------------------------------------------------------------
        event_ready = 1'b0;
        pulse_sources(4'b0100, 64'd0, 64'd0, EVENT_C, 64'd0);
        pulse_sources(4'b0100, 64'd0, 64'd0, EVENT_F, 64'd0);

        #1;
        check_bit(event_lost_pulse, 1'b1, "same source lost pulse");
        check_u16(event_lost_count, 16'd1, "same source lost count");
        check_event_data(event_data, EVENT_C,
                         "same source preserves oldest event");

        @(negedge clk);
        check_bit(event_lost_pulse, 1'b0, "lost pulse one clock");
        consume_expect(EVENT_C, "same source consume preserved event");

        // ---------------------------------------------------------------------
        // TEST 6: 한 클럭에 2개의 동일 Source 충돌이면 Count +2
        // ---------------------------------------------------------------------
        event_ready = 1'b0;
        pulse_sources(4'b0110, 64'd0, EVENT_B, EVENT_C, 64'd0);
        check_u8(pending_count, 8'd2, "multi collision setup");

        pulse_sources(4'b0110, 64'd0, EVENT_E, EVENT_F, 64'd0);

        #1;
        check_bit(event_lost_pulse, 1'b1, "multi collision lost pulse");
        check_u16(event_lost_count, 16'd3,
                  "multi collision exact lost count");
        check_event_data(event_data, EVENT_B,
                         "multi collision source1 oldest");

        consume_expect(EVENT_B, "multi collision consume source1");
        consume_expect(EVENT_C, "multi collision consume source2");
        check_bit(pending_any, 1'b0, "final pending empty");

        // ---------------------------------------------------------------------
        // TEST 7: statistics_clear는 진단 Count만 동기식 Clear
        // ---------------------------------------------------------------------
        @(negedge clk);
        statistics_clear = 1'b1;
        #1;
        check_u16(event_lost_count, 16'd3,
                  "statistics clear not asynchronous");

        @(posedge clk);
        #1;
        check_u16(event_lost_count, 16'd0,
                  "statistics clear count");
        check_bit(pending_any, 1'b0,
                  "statistics clear preserves pending state");

        @(negedge clk);
        statistics_clear = 1'b0;

        // ---------------------------------------------------------------------
        // TEST 8: Core의 16-Source 구성에서 동시 충돌 수를 정확히 계산
        // ---------------------------------------------------------------------
        event_ready = 1'b0;
        pulse_sources(
            {SOURCE_COUNT{1'b1}},
            EVENT_A, EVENT_B, EVENT_C, EVENT_D
        );
        check_u8(pending_count, 8'd16,
                 "sixteen-source collision setup");

        // Source 0은 같은 클럭에 전달/교체되므로 유실이 아니고,
        // 아직 Pending인 Source 1~15만 정확히 15회 유실이다.
        @(negedge clk);
        event_ready = 1'b1;
        source_data[0*EVENT_WIDTH +: EVENT_WIDTH] = EVENT_E;
        source_data[1*EVENT_WIDTH +: EVENT_WIDTH] = EVENT_B;
        source_data[2*EVENT_WIDTH +: EVENT_WIDTH] = EVENT_C;
        source_data[3*EVENT_WIDTH +: EVENT_WIDTH] = EVENT_D;
        source_valid = {SOURCE_COUNT{1'b1}};

        @(negedge clk);
        source_valid = {SOURCE_COUNT{1'b0}};
        event_ready = 1'b0;
        #1;
        check_bit(event_lost_pulse, 1'b1,
                  "sixteen-source lost pulse");
        check_u16(event_lost_count, 16'd15,
                  "sixteen-source exact lost count");
        check_u8(pending_count, 8'd16,
                 "sixteen-source replace preserves count");

        // ---------------------------------------------------------------------
        // 결과
        // ---------------------------------------------------------------------
        if (error_count == 0)
            $display("[PASS] All event_arbiter core tests passed.");
        else
            $display("[FAIL] event_arbiter tests failed: %0d error(s).",
                     error_count);

        #20;
        $finish;
    end

    initial begin
        #5000;
        $display("[FAIL] Simulation timeout.");
        $finish;
    end

endmodule
