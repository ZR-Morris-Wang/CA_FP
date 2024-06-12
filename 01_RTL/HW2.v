module ALU #(
    parameter DATA_W = 32
)
(
    input                       i_clk,   // clock
    input                       i_rst_n, // reset

    input                       i_valid, // input valid signal
    input [DATA_W - 1 : 0]      i_A,     // input operand A
    input [DATA_W - 1 : 0]      i_B,     // input operand B
    input [         2 : 0]      i_inst,  // instruction

    output [2*DATA_W - 1 : 0]   o_data,  // output value
    output                      o_done   // output valid signal
);
// Do not Modify the above part !!!

// Parameters
    // ======== choose your FSM style ==========
    // 1. FSM based on operation cycles
    // parameter S_IDLE           = 2'd0;
    // parameter S_ONE_CYCLE_OP   = 2'd1;
    // parameter S_MULTI_CYCLE_OP = 2'd2;
    // 2. FSM based on operation modes
    parameter S_IDLE = 4'd0;
    parameter S_ADD  = 4'd1;
    parameter S_SUB  = 4'd2;
    parameter S_AND  = 4'd3;
    parameter S_OR   = 4'd4;
    parameter S_SLT  = 4'd5;
    parameter S_SLL  = 4'd6;
    parameter S_MUL  = 4'd7;
    parameter S_DIV  = 4'd8;
    parameter S_OUT  = 4'd9;

// Wires & Regs
    // Todo
    // state
    reg  [         4: 0] state, state_nxt; // remember to expand the bit width if you want to add more states!
    // load input
    reg  [  DATA_W-1: 0] operand_a, operand_a_nxt;
    reg  [  DATA_W-1: 0] operand_b, operand_b_nxt;
    reg  [         2: 0] inst, inst_nxt;
    reg  [2*DATA_W-1: 0] result, result_nxt;
    reg valid_signal, valid_signal_nxt;
// Wire Assignments
    // Todo
    assign o_data = result;
    assign o_done = valid_signal;

// Always Combination
    // load input
    always @(*) begin
        if (i_valid) begin
            operand_a_nxt = i_A;
            operand_b_nxt = i_B;
            inst_nxt      = i_inst;
        end
        else begin
            operand_a_nxt = operand_a;
            operand_b_nxt = operand_b;
            inst_nxt      = inst;
        end
    end
    // Todo: FSM
    always @(*) begin
        case(state)
            S_IDLE : begin
                if(!i_valid) begin
                    state_nxt = S_IDLE;
                end
                state_nxt = {0, inst};
            end

            // S_ADD :
            // S_SUB :
            S_AND : begin 
                state_nxt = S_OUT; 
                result_nxt = {{32'b0}, operand_a & operand_b};
                valid_signal_nxt = 1;
            end
            // S_OR :
            // S_SLT :
            // S_SLL :
            // S_MUL :
            // S_DIV :
            S_OUT : begin
                valid_signal_nxt = 0;
                state_nxt = S_IDLE;
            end

            default : state_nxt = state;
        endcase
    end
    // Todo: Counter

    // Todo: ALU output
    
    // Todo: output valid signal

    // Todo: Sequential always block
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            state       <= S_IDLE;
            result      <= 0;
            valid_signal <= 0;
            operand_a   <= 0;
            operand_b   <= 0;
            inst        <= 0;
        end
        else begin
            state       <= state_nxt;
            result      <= result_nxt;
            valid_signal <= valid_signal_nxt;
            operand_a   <= operand_a_nxt;
            operand_b   <= operand_b_nxt;
            inst        <= inst_nxt;
        end
    end



endmodule