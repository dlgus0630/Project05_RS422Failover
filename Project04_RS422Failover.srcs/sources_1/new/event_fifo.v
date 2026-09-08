`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// event_fifo
//
// 역할
//   event_arbiter가 전달한 64-bit Event를 발생 순서대로 저장하고,
//   AXI-Lite Register Block이 가장 오래된 Event를 안정적으로 읽게 한다.
//
// 입력 Handshake
//   event_valid && event_ready인 상승 에지에서 Event 1개를 저장한다.
//   FIFO가 가득 차면 event_ready=0이 되어 event_arbiter에 Backpressure를 건다.
//
// CPU/AXI-Lite 쪽 읽기
//   front_valid=1일 때 front_data가 FIFO의 가장 오래된 Event이다.
//   CPU는 front_data의 Low/High 32-bit를 모두 읽은 뒤 pop_request를
//   정확히 1클럭 Pulse로 발생시켜야 한다.
//
// 동시 Push/Pop
//   같은 상승 에지에 Push와 Pop이 모두 발생하면 저장 개수는 유지된다.
//   FIFO가 Full이어도 정상 Pop이 함께 발생하면 event_ready=1이 되어
//   새 Event를 빈자리에 즉시 저장할 수 있다.
//
// Overflow 정책
//   event_ready=0은 Event 유실이 아니라 정상적인 Backpressure이다.
//   event_arbiter가 event_valid/event_data를 유지하므로 이 FIFO에서는
//   Overflow Count를 만들지 않는다. 실제 상위 유실은 event_arbiter의
//   event_lost_pulse/event_lost_count가 관리한다.
//
// Underflow 정책
//   FIFO가 Empty인데 pop_request가 들어오면 데이터와 Pointer는 바꾸지 않고
//   underflow_pulse를 1클럭 발생시키며 underflow_count를 포화 누적한다.
//////////////////////////////////////////////////////////////////////////////////

module event_fifo #(
    parameter integer EVENT_WIDTH = 64,
    parameter integer FIFO_DEPTH  = 16
)(
    input  wire                     clk,
    input  wire                     reset_p,
    input  wire                     clear_fifo,
    input  wire                     statistics_clear,

    // event_arbiter와 연결되는 입력
    input  wire                     event_valid,
    output wire                     event_ready,
    input  wire [EVENT_WIDTH-1:0]   event_data,

    // AXI-Lite Register Block과 연결되는 Front Event/Pop
    output wire                     front_valid,
    output wire [EVENT_WIDTH-1:0]   front_data,
    input  wire                     pop_request,

    // 상태 및 진단
    output wire                     fifo_empty,
    output wire                     fifo_full,
    output wire [7:0]               event_count,
    output reg                      underflow_pulse,
    output reg  [15:0]              underflow_count
);

    localparam integer PTR_WIDTH =
        (FIFO_DEPTH <= 1) ? 1 : $clog2(FIFO_DEPTH);

    localparam integer COUNT_WIDTH =
        (FIFO_DEPTH <= 1) ? 1 : $clog2(FIFO_DEPTH + 1);

    reg [EVENT_WIDTH-1:0] fifo_mem [0:FIFO_DEPTH-1];

    reg [PTR_WIDTH-1:0]   write_ptr;
    reg [PTR_WIDTH-1:0]   read_ptr;
    reg [COUNT_WIDTH-1:0] count;

    wire push_event;
    wire pop_event;
    wire underflow_event;

    assign fifo_empty = (count == 0);
    assign fifo_full  = (count == FIFO_DEPTH);

    assign front_valid = !fifo_empty;
    assign front_data  =
        fifo_empty ? {EVENT_WIDTH{1'b0}} : fifo_mem[read_ptr];

    // Full 상태에서도 같은 클럭에 Pop이 확정되면 한 칸이 비므로
    // 새 Event를 동시에 받을 수 있다.
    assign pop_event   = pop_request && !fifo_empty && !clear_fifo;
    assign event_ready = !clear_fifo && (!fifo_full || pop_event);
    assign push_event  = event_valid && event_ready;

    assign underflow_event =
        pop_request && fifo_empty && !clear_fifo;
    assign event_count = count;

    // FIFO 데이터 배열은 Reset/Clear하지 않는다. 데이터 쓰기를 비동기
    // Reset 제어 블록과 분리하여 RAM 추론을 막는 비동기 Set/Reset
    // 구조가 생기지 않게 한다. Reset/Clear 클럭의 쓰기 금지는 유지한다.
    always @(posedge clk) begin
        if (!reset_p && !clear_fifo && push_event)
            fifo_mem[write_ptr] <= event_data;
    end

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            write_ptr        <= {PTR_WIDTH{1'b0}};
            read_ptr         <= {PTR_WIDTH{1'b0}};
            count            <= {COUNT_WIDTH{1'b0}};
            underflow_pulse  <= 1'b0;
            underflow_count  <= 16'd0;

            // FIFO Data 자체는 Reset하지 않는다.
            // count=0이므로 Reset 이전 값은 front_data로 노출되지 않는다.
        end
        else begin
            underflow_pulse <= 1'b0;

            if (statistics_clear)
                underflow_count <= 16'd0;

            if (underflow_event) begin
                underflow_pulse <= 1'b1;

                if (!statistics_clear &&
                    (underflow_count != 16'hFFFF))
                    underflow_count <= underflow_count + 1'b1;
            end

            if (clear_fifo) begin
                write_ptr <= {PTR_WIDTH{1'b0}};
                read_ptr  <= {PTR_WIDTH{1'b0}};
                count     <= {COUNT_WIDTH{1'b0}};
            end
            else begin
                if (push_event) begin
                    if (write_ptr == FIFO_DEPTH - 1)
                        write_ptr <= {PTR_WIDTH{1'b0}};
                    else
                        write_ptr <= write_ptr + 1'b1;
                end

                if (pop_event) begin
                    if (read_ptr == FIFO_DEPTH - 1)
                        read_ptr <= {PTR_WIDTH{1'b0}};
                    else
                        read_ptr <= read_ptr + 1'b1;
                end

                case ({push_event, pop_event})
                    2'b10: count <= count + 1'b1;
                    2'b01: count <= count - 1'b1;
                    default: count <= count;
                endcase
            end
        end
    end

endmodule
