`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// tb_event_fifo
//
// Self-checking 항목
//   1. Reset 후 Empty/Ready/Count/Front 출력
//   2. 단일 Event Push 및 Pop
//   3. Pop하지 않는 동안 Front Event 유지
//   4. FIFO 16개 Full 및 event_ready Backpressure
//   5. Full 상태의 동시 Pop+Push와 Count 유지
//   6. 전체 Drain 순서 및 Read/Write Pointer Wrap-around
//   7. 일반 상태의 동시 Pop+Push
//   8. Empty Pop Underflow Pulse/Count
//   9. Empty 상태의 동시 Push+Pop 경계 조건
//  10. Reset이 저장 상태와 진단 Count를 초기화하는지 확인
//////////////////////////////////////////////////////////////////////////////////

module tb_event_fifo;

    localparam integer EVENT_WIDTH = 64;
    localparam integer FIFO_DEPTH  = 16;

    reg                    clk;
    reg                    reset_p;
    reg                    clear_fifo;
    reg                    statistics_clear;

    reg                    event_valid;
    wire                   event_ready;
    reg  [EVENT_WIDTH-1:0] event_data;

    wire                   front_valid;
    wire [EVENT_WIDTH-1:0] front_data;
    reg                    pop_request;

    wire                   fifo_empty;
    wire                   fifo_full;
    wire [7:0]             event_count;
    wire                   underflow_pulse;
    wire [15:0]            underflow_count;

    reg [EVENT_WIDTH-1:0] test_event [0:31];

    integer error_count;
    integer event_index;

    event_fifo #(
        .EVENT_WIDTH (EVENT_WIDTH),
        .FIFO_DEPTH  (FIFO_DEPTH)
    ) dut (
        .clk              (clk),
        .reset_p          (reset_p),
        .clear_fifo       (clear_fifo),
        .statistics_clear (statistics_clear),
        .event_valid      (event_valid),
        .event_ready      (event_ready),
        .event_data       (event_data),
        .front_valid      (front_valid),
        .front_data       (front_data),
        .pop_request      (pop_request),
        .fifo_empty       (fifo_empty),
        .fifo_full        (fifo_full),
        .event_count      (event_count),
        .underflow_pulse  (underflow_pulse),
        .underflow_count  (underflow_count)
    );

    initial clk = 1'b0;
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
        input [EVENT_WIDTH-1:0] actual;
        input [EVENT_WIDTH-1:0] expected;
        input [8*80-1:0] check_name;
        begin
            if (actual !== expected) begin
                $display("[FAIL] %0s: expected=%h actual=%h",
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

    // event_ready가 1인 정상 상황에서 Event 1개를 Push한다.
    task push_event;
        input [EVENT_WIDTH-1:0] push_data;
        begin
            @(negedge clk);
            event_valid = 1'b1;
            event_data  = push_data;
            #1;

            check_bit(event_ready, 1'b1, "push event_ready");

            @(posedge clk);
            #1;

            @(negedge clk);
            event_valid = 1'b0;
        end
    endtask

    // 현재 Front Event를 검사한 뒤 정확히 1클럭 Pop한다.
    task pop_expect;
        input [EVENT_WIDTH-1:0] expected_data;
        begin
            @(negedge clk);
            check_bit(front_valid, 1'b1, "pop front_valid");
            check_event_data(front_data, expected_data, "pop front_data");

            pop_request = 1'b1;

            @(posedge clk);
            #1;

            check_bit(underflow_pulse, 1'b0, "normal pop no underflow");

            @(negedge clk);
            pop_request = 1'b0;
        end
    endtask

    initial begin
        reset_p         = 1'b1;
        clear_fifo      = 1'b0;
        statistics_clear = 1'b0;
        event_valid     = 1'b0;
        event_data      = {EVENT_WIDTH{1'b0}};
        pop_request     = 1'b0;
        error_count     = 0;

        for (event_index = 0;
             event_index < 32;
             event_index = event_index + 1) begin

            test_event[event_index] =
                64'hA500_0000_0000_0000 + event_index;
        end

        apply_reset;

        // ---------------------------------------------------------------------
        // TEST 1: Reset 상태
        // ---------------------------------------------------------------------
        check_bit(fifo_empty, 1'b1, "reset fifo_empty");
        check_bit(fifo_full, 1'b0, "reset fifo_full");
        check_bit(event_ready, 1'b1, "reset event_ready");
        check_bit(front_valid, 1'b0, "reset front_valid");
        check_event_data(front_data, 64'd0, "reset front_data");
        check_u8(event_count, 8'd0, "reset event_count");
        check_bit(underflow_pulse, 1'b0, "reset underflow_pulse");
        check_u16(underflow_count, 16'd0, "reset underflow_count");

        // ---------------------------------------------------------------------
        // TEST 2: 단일 Push, Front 유지, Pop
        // ---------------------------------------------------------------------
        push_event(test_event[0]);

        check_bit(fifo_empty, 1'b0, "single push not empty");
        check_bit(front_valid, 1'b1, "single push front_valid");
        check_event_data(front_data, test_event[0], "single push front_data");
        check_u8(event_count, 8'd1, "single push count");

        repeat (3) begin
            @(negedge clk);
            check_bit(front_valid, 1'b1, "front held valid");
            check_event_data(front_data, test_event[0], "front held data");
            check_u8(event_count, 8'd1, "front held count");
        end

        pop_expect(test_event[0]);
        check_bit(fifo_empty, 1'b1, "single pop empty");
        check_bit(front_valid, 1'b0, "single pop front invalid");
        check_event_data(front_data, 64'd0, "single pop front zero");
        check_u8(event_count, 8'd0, "single pop count");

        // ---------------------------------------------------------------------
        // TEST 3: 16개를 채워 Full 상태와 입력 Backpressure 확인
        // ---------------------------------------------------------------------
        for (event_index = 0;
             event_index < FIFO_DEPTH;
             event_index = event_index + 1) begin

            push_event(test_event[event_index]);
        end

        check_bit(fifo_full, 1'b1, "filled fifo_full");
        check_bit(event_ready, 1'b0, "full backpressure ready");
        check_u8(event_count, FIFO_DEPTH, "filled event_count");
        check_event_data(front_data, test_event[0], "filled oldest event");

        // Full 동안 event_arbiter가 동일 Event를 유지하는 상황
        @(negedge clk);
        event_valid = 1'b1;
        event_data  = test_event[16];

        repeat (3) begin
            @(posedge clk);
            #1;
            check_bit(event_ready, 1'b0, "full ready held low");
            check_bit(fifo_full, 1'b1, "full state held");
            check_u8(event_count, FIFO_DEPTH, "full count held");
            check_event_data(front_data, test_event[0],
                             "full front data held");
        end

        // ---------------------------------------------------------------------
        // TEST 4: Full 상태에서 Pop+Push 동시 처리
        // ---------------------------------------------------------------------
        @(negedge clk);
        pop_request = 1'b1;
        #1;

        // Pop으로 한 칸이 비므로 같은 클럭의 Push를 받을 수 있어야 한다.
        check_bit(event_ready, 1'b1, "full simultaneous ready");
        check_event_data(front_data, test_event[0],
                         "full simultaneous pop data");

        @(posedge clk);
        #1;

        check_bit(fifo_full, 1'b1, "full simultaneous remains full");
        check_u8(event_count, FIFO_DEPTH,
                 "full simultaneous count unchanged");
        check_event_data(front_data, test_event[1],
                         "full simultaneous next front");
        check_bit(underflow_pulse, 1'b0,
                  "full simultaneous no underflow");

        @(negedge clk);
        event_valid = 1'b0;
        pop_request = 1'b0;

        // 가장 오래된 0번은 이미 Pop됐고, 16번이 Tail에 들어갔다.
        for (event_index = 1;
             event_index <= FIFO_DEPTH;
             event_index = event_index + 1) begin

            pop_expect(test_event[event_index]);
        end

        check_bit(fifo_empty, 1'b1, "wrap drain empty");
        check_u8(event_count, 8'd0, "wrap drain count");

        // ---------------------------------------------------------------------
        // TEST 5: 일반 상태에서 동시 Pop+Push
        // ---------------------------------------------------------------------
        push_event(test_event[17]);
        push_event(test_event[18]);
        push_event(test_event[19]);

        @(negedge clk);
        check_event_data(front_data, test_event[17],
                         "normal simultaneous old front");

        event_valid = 1'b1;
        event_data  = test_event[20];
        pop_request = 1'b1;

        @(posedge clk);
        #1;

        check_u8(event_count, 8'd3,
                 "normal simultaneous count unchanged");
        check_event_data(front_data, test_event[18],
                         "normal simultaneous next front");

        @(negedge clk);
        event_valid = 1'b0;
        pop_request = 1'b0;

        pop_expect(test_event[18]);
        pop_expect(test_event[19]);
        pop_expect(test_event[20]);

        // ---------------------------------------------------------------------
        // TEST 6: Empty Pop은 Underflow 1회로 기록
        // ---------------------------------------------------------------------
        @(negedge clk);
        pop_request = 1'b1;

        @(posedge clk);
        #1;

        check_bit(underflow_pulse, 1'b1, "empty pop underflow pulse");
        check_u16(underflow_count, 16'd1, "empty pop underflow count");
        check_u8(event_count, 8'd0, "empty pop count unchanged");
        check_bit(front_valid, 1'b0, "empty pop front invalid");

        @(negedge clk);
        pop_request = 1'b0;

        @(posedge clk);
        #1;
        check_bit(underflow_pulse, 1'b0, "underflow pulse one clock");

        // ---------------------------------------------------------------------
        // TEST 7: Empty에서 Push+Pop 동시 발생
        //   상승 에지 전에 기존 Event가 없으므로 Pop은 Underflow이고,
        //   새 Event는 정상 Push되어 다음 클럭부터 Front에 나타난다.
        // ---------------------------------------------------------------------
        @(negedge clk);
        event_valid = 1'b1;
        event_data  = test_event[21];
        pop_request = 1'b1;

        check_bit(event_ready, 1'b1, "empty simultaneous ready");
        check_bit(front_valid, 1'b0, "empty simultaneous no old event");

        @(posedge clk);
        #1;

        check_bit(underflow_pulse, 1'b1,
                  "empty simultaneous underflow pulse");
        check_u16(underflow_count, 16'd2,
                  "empty simultaneous underflow count");
        check_u8(event_count, 8'd1, "empty simultaneous push count");
        check_bit(front_valid, 1'b1, "empty simultaneous new front valid");
        check_event_data(front_data, test_event[21],
                         "empty simultaneous new front data");

        @(negedge clk);
        event_valid = 1'b0;
        pop_request = 1'b0;

        pop_expect(test_event[21]);

        // ---------------------------------------------------------------------
        // TEST 8: statistics_clear는 FIFO 내용은 보존하고 Count만 동기 Clear
        // ---------------------------------------------------------------------
        push_event(test_event[22]);
        push_event(test_event[23]);
        check_u8(event_count, 8'd2, "pre-statistics-clear fifo count");

        @(negedge clk);
        statistics_clear = 1'b1;
        #1;
        check_u16(underflow_count, 16'd2,
                  "statistics clear not asynchronous");
        check_u8(event_count, 8'd2,
                 "statistics clear pre-edge fifo count");

        @(posedge clk);
        #1;
        check_u16(underflow_count, 16'd0,
                  "statistics clear underflow count");
        check_u8(event_count, 8'd2,
                 "statistics clear preserves fifo contents");
        check_event_data(front_data, test_event[22],
                         "statistics clear preserves front");

        @(negedge clk);
        statistics_clear = 1'b0;

        // ---------------------------------------------------------------------
        // TEST 9: clear_fifo는 저장 상태를 동기식 Clear
        // ---------------------------------------------------------------------
        clear_fifo = 1'b1;
        #1;
        check_u8(event_count, 8'd2, "fifo clear not asynchronous");

        @(posedge clk);
        #1;
        check_u8(event_count, 8'd0, "fifo clear count");
        check_bit(fifo_empty, 1'b1, "fifo clear empty");
        check_bit(front_valid, 1'b0, "fifo clear front invalid");
        check_u16(underflow_count, 16'd0,
                  "fifo clear preserves diagnostic count");

        @(negedge clk);
        clear_fifo = 1'b0;

        // Reset 검증용으로 상태와 진단 Count를 다시 만든다.
        pop_request = 1'b1;
        @(posedge clk);
        #1;
        check_u16(underflow_count, 16'd1,
                  "pre-reset underflow count");
        @(negedge clk);
        pop_request = 1'b0;

        push_event(test_event[24]);
        push_event(test_event[25]);
        check_u8(event_count, 8'd2, "pre-reset count");

        // ---------------------------------------------------------------------
        // TEST 10: Reset이 FIFO 상태와 진단 Count를 모두 초기화
        // ---------------------------------------------------------------------

        apply_reset;

        check_bit(fifo_empty, 1'b1, "final reset fifo_empty");
        check_bit(fifo_full, 1'b0, "final reset fifo_full");
        check_bit(front_valid, 1'b0, "final reset front_valid");
        check_event_data(front_data, 64'd0, "final reset front_data");
        check_u8(event_count, 8'd0, "final reset event_count");
        check_bit(underflow_pulse, 1'b0,
                  "final reset underflow_pulse");
        check_u16(underflow_count, 16'd0,
                  "final reset underflow_count");

        // ---------------------------------------------------------------------
        // 결과
        // ---------------------------------------------------------------------
        if (error_count == 0)
            $display("[PASS] All event_fifo core tests passed.");
        else
            $display("[FAIL] event_fifo tests failed: %0d error(s).",
                     error_count);

        #20;
        $finish;
    end

    initial begin
        #30000;
        $display("[FAIL] Simulation timeout.");
        $finish;
    end

endmodule
