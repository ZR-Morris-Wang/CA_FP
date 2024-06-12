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
    reg  [         3: 0] state, state_nxt; // remember to expand the bit width if you want to add more states!
    // load input
    reg  [  DATA_W-1: 0] operand_a, operand_a_nxt;
    reg  [  DATA_W-1: 0] operand_b, operand_b_nxt;
    reg  [         2: 0] inst, inst_nxt;
    reg  [  2*DATA_W: 0] result, result_nxt;
    reg  [2*DATA_W-1: 0] temp;
    reg  [  2*DATA_W: 0] divTemp;
    reg  [         4: 0] counter, counter_nxt;
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
                else begin
                    case(i_inst)
                        3'd0: state_nxt = S_ADD;
                        3'd1: state_nxt = S_SUB;
                        3'd2: state_nxt = S_AND;
                        3'd3: state_nxt = S_OR;
                        3'd4: state_nxt = S_SLT;
                        3'd5: state_nxt = S_SLL;
                        3'd6: state_nxt = S_MUL;
                        3'd7: state_nxt = S_DIV;
                        default: state_nxt = S_IDLE;
                    endcase
                end
                result_nxt = 0;
                valid_signal_nxt = 0;
            end

            S_ADD : begin
                state_nxt = S_OUT;
                if($signed(operand_a) > 0 && $signed(operand_b) > 0) begin
                    temp = $signed(operand_a) + $signed(operand_b);
                    result_nxt = (temp[31] == 1) ? {1, {30{1'b1}}} : {33'b0, (operand_a + operand_b)};
                end
                else if ($signed(operand_a) < 0 && $signed(operand_b) < 0) begin
                    temp = $signed(operand_a) + $signed(operand_b);
                    result_nxt = (temp[31] == 0) ? {33'b0, {1, 31'b0}} : {33'b0, (operand_a + operand_b)};
                end
                else begin
                    result_nxt = {33'b0, operand_a + operand_b};
                end
                valid_signal_nxt = 1;
            end

            S_SUB : begin
                state_nxt = S_OUT;
                if($signed(operand_a) > 0 && $signed(operand_b) < 0) begin
                    temp = $signed(operand_a) + $signed(~operand_b + 1);
                    result_nxt = (temp[31] == 1) ? {1, {30{1'b1}}} : {33'b0, (operand_a + ~operand_b + 1)};
                end
                else if ($signed(operand_a) < 0 && $signed(operand_b) > 0) begin
                    temp = $signed(operand_a) + $signed(~operand_b + 1);
                    result_nxt = (temp[31] == 0) ? {33'b0, {1, 31'b0}} : {33'b0, (operand_a + ~operand_b + 1)};
                end
                else begin
                    result_nxt = {33'b0, operand_a + ~operand_b + 1};
                end
                valid_signal_nxt = 1;
            end

            S_AND : begin 
                state_nxt = S_OUT; 
                result_nxt = {{33'b0}, operand_a & operand_b};
                valid_signal_nxt = 1;
            end
            S_OR : begin
                state_nxt = S_OUT;
                result_nxt = {{33'b0, operand_a | operand_b}};
                valid_signal_nxt = 1;
            end
            S_SLT: begin
                state_nxt = S_OUT;
                result_nxt = ($signed(operand_a) < $signed(operand_b)) ? {63'b0, 1'b1} : {63'b0, 1'b0};
                valid_signal_nxt = 1;
            end
            S_SLL : begin
                state_nxt = S_OUT;
                result_nxt = {33'b0, operand_a << operand_b};
                valid_signal_nxt = 1;
            end
            S_MUL : begin
                state_nxt = (counter == 31) ? S_OUT : S_MUL;
                valid_signal_nxt = (counter == 31) ? 1 : 0;
                result_nxt = (operand_b[counter]) ? (result + ({33'b0, operand_a} << counter)) : result;
            end
            S_DIV : begin
                state_nxt = (counter == 31) ? S_OUT : S_DIV;
                valid_signal_nxt = (counter == 31) ? 1 : 0;
                
                // if(counter == 31) begin
                //     result_nxt[64:32] = result[64:32] >>1;
                //     // $display("%b|%b", result[64:32], result[31:0]);
                // end
                // else begin
                //     result_nxt = result;
                // end

                if(counter == 0) begin
                    divTemp = {33'b0, operand_a} << 1;
                    result_nxt = (divTemp[64:32] < operand_b) ? divTemp << 1 : (({divTemp -  {operand_b, 32'b0}}) << 1) + 1;
                    // $display("%b|%b", result[64:32], result[31:0]);
                end
                else begin
                    result_nxt = (result[64:32] < operand_b) ? ((state_nxt == S_OUT) ? {result[64:32], result[31:0] << 1}/*result[31:0] << 1 */: result << 1) : ((state_nxt == S_OUT) ? {result[64:32] - operand_b, result[30:0], 1'b1} : (({result -  {operand_b, 32'b0}}) << 1) + 1);
                    // $display("%b|%b", result[64:32], result[31:0]);
                end

                
            end
            S_OUT : begin
                valid_signal_nxt = 0;
                state_nxt = S_IDLE;
                result_nxt = 0;
            end

            default : begin
                state_nxt = state;
                result_nxt = 0;
                valid_signal_nxt = 0;
            end
        endcase
    end
    // Todo: Counter
    always @(posedge i_clk or negedge i_rst_n) begin
            if (!i_rst_n) begin
                counter <= 0;
            end 
            else begin
                if((state == S_MUL || state == S_DIV) && counter != 31) begin
                    counter <= counter + 1;
                end
                else if(counter == 31) begin
                    counter <= 0;
                end
                else begin
                    counter <= 0;
                end
            end
    end
    

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