//----------------------------- DO NOT MODIFY THE I/O INTERFACE!! ------------------------------//
module CHIP #(                                                                                  //
    parameter BIT_W = 32                                                                        //
)(                                                                                              //
    // clock                                                                                    //
        input               i_clk,                                                              //
        input               i_rst_n,                                                            //
    // instruction memory                                                                       //
        input  [BIT_W-1:0]  i_IMEM_data,                                                        //
        output [BIT_W-1:0]  o_IMEM_addr,                                                        //
        output              o_IMEM_cen,                                                         //
    // data memory                                                                              //
        input               i_DMEM_stall,                                                       //
        input  [BIT_W-1:0]  i_DMEM_rdata,                                                       //
        output              o_DMEM_cen,                                                         //
        output              o_DMEM_wen,                                                         //
        output [BIT_W-1:0]  o_DMEM_addr,                                                        //
        output [BIT_W-1:0]  o_DMEM_wdata,                                                       //
    // finish procedure                                                                         //
        output              o_finish,                                                           //
    // cache                                                                                    //
        input               i_cache_finish,                                                     //
        output              o_proc_finish                                                       //
);                                                                                              //
//----------------------------- DO NOT MODIFY THE I/O INTERFACE!! ------------------------------//

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Parameters
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: any declaration
    // opcode
    parameter aupicop = 7'b0010111;
    parameter jalop = 7'b1101111;
    parameter jalrop = 7'b1100111;
    
    parameter addop = 7'b0110011;
    parameter subop = 7'b0110011;
    parameter andop = 7'b0110011;
    parameter xorop = 7'b0110011;

    parameter addiop = 7'b0010011;
    parameter slliop = 7'b0010011;
    parameter slti = 7'b0010011;
    parameter srai = 7'b0010011;

    parameter lwop = 7'b0000011;
    parameter swop = 7'b0100011;
    parameter mulop = 7'b0110011;
    parameter divop = 7'b0110011;

    parameter beqop = 7'b1100011;
    parameter bgeop = 7'b1100011;
    parameter bltop = 7'b1100011;
    parameter bneop = 7'b1100011;

    parameter ecallop = 7'b1110011;

    // function 3
    parameter jalrf3 = 3'b000;
    
    parameter addf3 = 3'b000;
    parameter subf3 = 3'b000;
    parameter andf3 = 3'b111;
    parameter xorf3 = 3'b100;

    parameter addif3 = 3'b000;
    parameter sllif3 = 3'b001;
    parameter sltif3 = 3'b010;
    parameter sraif3 = 3'b101;

    parameter lwf3 = 3'b010;
    parameter swf3 = 3'b010;
    parameter mulf3 = 3'b000;

    parameter beqf3 = 3'b000;
    parameter bgef3 = 3'b101;
    parameter bltf3 = 3'b100;
    parameter bnef3 = 3'b001;

    parameter ecallf3 = 3'b000;

    // function 7
    parameter addf7 = 7'b0000000;
    parameter subf7 = 7'b0100000;
    parameter andf7 = 7'b0000000;
    parameter xorf7 = 7'b0000000;

    parameter sllif7 = 7'b0000000;
    parameter sraif7 = 7'b0100000;

    parameter mulf7 = 7'b0000001;
    
    // immediate
    parameter ecallimm = 12'b000000000000;

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Wires and Registers
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // TODO: any declaration
        
        // PC memory
        reg [BIT_W-1: 0] PC, next_PC; 

        //termination
        reg ecallreg;
        reg procreg;

        //Reg_file
        reg [4:0] register_source_1, register_source_2, register_destination; //I
        reg [BIT_W-1: 0] write_data; //I
        reg [BIT_W-1: 0] read_data_1, read_data_2; //O
        reg [31:0] immediate;

        wire [4:0] rs1, rs2, rd;
        wire [BIT_W-1: 0] wdata;
        wire [BIT_W-1: 0] rdata1, rdata2;

        //controller
        reg [6:0] opcode; //I
        reg [2:0] func3;
        reg [6:0] func7;

        wire [6:0] opcode_w;
        wire [2:0] func3_w;
        wire [6:0] func7_w;

        wire MemRnW, MemtoReg, MemWrite, ALUSrc, RegWrite; //O
        wire [1:0] Branch; 
        wire [3:0] ALUOp; 
        
        //ALU/MULDIV
        reg [BIT_W - 1:0] i_Breg;
        reg [BIT_W - 1:0] final_op;
        reg mul_en, mul_en_old;
        reg mul_valid, mul_counter_valid;
        reg mul_done_reg, mul_done_reg_old;
        wire [BIT_W - 1:0] i_A_wire, i_B_wire;
        wire [BIT_W - 1:0] o_ALU_wire, o_MULDIV_wire;
        wire mul_valid_wire, mul_counter_valid_wire, mul_done;

        //state
        reg state, state_nxt;

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Continuous Assignment
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: any wire assignment

    // Assign wires to F/Fs
        
        //State
        assign o_IMEM_cen = state;
        assign o_IMEM_addr = PC;

        //termination
        assign o_finish = ecallreg;
        assign o_proc_finish = procreg;

        //Reg_file
        assign rs1 = register_source_1; 
        assign rs2 = register_source_2;
        assign rd = register_destination;
        assign wdata = write_data;

        //controller
        assign opcode_w = opcode;
        assign func3_w = func3;
        assign func7_w = func7;

        //ALU/MULDIV
        assign i_A_wire = read_data_1;
        assign i_B_wire = i_Breg;
        assign mul_valid_wire = mul_valid;
        assign mul_counter_valid_wire = mul_counter_valid;

        //To o/p
        assign o_DMEM_cen = MemRnW;
        assign o_DMEM_wen = MemWrite;
        assign o_DMEM_wdata = read_data_2;
        assign o_DMEM_addr = final_op;
    
// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Submoddules
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: Reg_file wire connection
    Reg_file reg0(               
        .i_clk  (i_clk),
        .i_rst_n(i_rst_n),
        .wen    (RegWrite),
        .rs1    (rs1),
        .rs2    (rs2),
        .rd     (rd),
        .wdata  (wdata),
        .rdata1 (rdata1),
        .rdata2 (rdata2)
    );
    ALU alu0(
        .i_clk   (i_clk),
        .i_rst_n (i_rst_n),
        .i_A     (i_A_wire),
        .i_B     (i_B_wire),
        .ALUOp  (ALUOp),
        .o_data  (o_ALU_wire)
    );
    MULDIV_unit md0(
        .i_clk   (i_clk),
        .i_rst_n (i_rst_n),
        .i_valid (mul_valid_wire),
        .i_A     (i_A_wire),
        .i_B     (i_B_wire),
        .i_counter_valid(mul_counter_valid_wire),
        .o_data  (o_MULDIV_wire),
        .o_done  (mul_done)
    );
    ControlUnit cu0(
        .opcode  (opcode_w),
        .func3   (func3_w),
        .func7   (func7_w),
        .Branch  (Branch),
        .MemRnW  (MemRnW),
        .MemtoReg(MemtoReg),
        .ALUOp   (ALUOp),
        .MemWrite(MemWrite),
        .ALUSrc  (ALUSrc),
        .RegWrite(RegWrite)
    );

    // Cache cache0(
    //     .i_clk(i_clk),
    //     .i_rst_n(i_rst_n),
    //     .o_proc_stall(i_DMEM_stall),
    //     .o_proc_rdata(i_DMEM_rdata),
    //     .i_proc_cen(o_DMEM_cen),
    //     .i_proc_wen(o_DMEM_wen),
    //     .i_proc_addr(o_DMEM_addr),
    //     .i_proc_wdata(o_DMEM_wdata),
    // );

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Always Blocks
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // Todo: any combinational/sequential circuit

    always @(*) begin //get opcode
        opcode = i_IMEM_data[6:0];
    end

    always @(*) begin //knowing if instuctions should be read
        if ( ( !i_DMEM_stall && !mul_en) || mul_done ) begin //from Kmap
            state_nxt = 1'b1;
        end
        else begin
            state_nxt = 1'b0;
        end
    end

    always @(*) begin //deciding if the program is terminated, decide PC addr
        case (i_IMEM_data)
            32'h73: begin
                ecallreg = 1;
                procreg = 1;
            end
            default: begin
                ecallreg = 0;
                procreg = 0;
            end
        endcase

        case (Branch) //update PC address (MUX1)
            2'b00: begin //no branch
                case (o_IMEM_cen)
                    1'b0: begin
                        next_PC = PC;
                    end
                    1'b1: begin
                        next_PC = PC + 4;
                    end
                    default: begin
                        next_PC = PC;
                    end
                endcase
            end

            2'b01: begin //usual branch
                case (o_DMEM_addr)
                    1: begin
                        next_PC = PC + ({i_IMEM_data[31], i_IMEM_data[7], i_IMEM_data[30:25], i_IMEM_data[11:8]} << 1);
                    end
                    default: begin
                        next_PC = PC + 4;
                    end
                endcase
            end

            2'b10: begin //jal, aupic
                case (opcode)
                    aupicop: begin
                        next_PC = PC + 4;
                    end
                    jalop: begin
                        next_PC = $signed(PC) + $signed(immediate);
                    end
                    default: begin
                        next_PC = PC;
                    end
                endcase
            end

            2'b11: begin //jalr
                next_PC = $signed(read_data_1) + $signed(immediate);
            end

            default: begin
                next_PC = PC + 4;
            end
        endcase
    end

    always @(*) begin //decoding, update rs1, rs2, rd, opcode
        case (i_IMEM_data[6:0])
            7'b0110011: begin //R-type
                register_source_1 = i_IMEM_data[19:15];
                register_source_2 = i_IMEM_data[24:20];
                register_destination = i_IMEM_data[11:7];
                immediate = 0;
                func3 = i_IMEM_data[14:12];
                func7 = i_IMEM_data[31:25];
            end
            
            7'b1100111: begin //I-type (jalr)
                register_source_1 = i_IMEM_data[19:15];
                register_source_2 = 0;
                register_destination = i_IMEM_data[11:7];
                immediate = $signed(i_IMEM_data[31:20]);
                func3 = i_IMEM_data[14:12];
                func7 = 7'b0;
            end

            7'b0010011: begin //I-type (addi...)
                register_source_1 = i_IMEM_data[19:15];
                register_source_2 = 0;
                register_destination = i_IMEM_data[11:7];
                func3 = i_IMEM_data[14:12];
                case (func3) 
                    3'b000: immediate = $signed(i_IMEM_data[31:20]); //addi
                    3'b010: immediate = $signed(i_IMEM_data[31:20]); //slti
                    3'b001: immediate = $signed(i_IMEM_data[24:20]); //slli
                    3'b101: immediate = $signed(i_IMEM_data[24:20]); //srli
                    default: immediate = 0;
                endcase
                func7 = 7'b0;
            end

            7'b0000011: begin //I-type (lw)
                register_source_1 = i_IMEM_data[19:15];
                register_source_2 = 0;
                register_destination = i_IMEM_data[11:7];
                immediate = $signed(i_IMEM_data[31:20]);
                func3 = i_IMEM_data[14:12];
                func7 = 7'b0;
            end

            7'b1110011: begin //I-type (Ecall)
                register_source_1 = i_IMEM_data[19:15];
                register_source_2 = 0;
                register_destination = i_IMEM_data[11:7];
                immediate = 0;
                func3 = i_IMEM_data[14:12];
                func7 = 7'b0;
            end

            7'b0100011: begin //S-type
                register_source_1 = i_IMEM_data[19:15];
                register_source_2 = i_IMEM_data[24:20];
                register_destination = 0;
                immediate = $signed({i_IMEM_data[31:25], i_IMEM_data[11:7]});
                func3 = i_IMEM_data[14:12];
                func7 = 7'b0;
            end

            7'b1100011: begin //SB-type
                register_source_1 = i_IMEM_data[19:15];
                register_source_2 = i_IMEM_data[24:20];
                register_destination = 0;
                immediate = $signed({i_IMEM_data[31], i_IMEM_data[7], i_IMEM_data[30:25], i_IMEM_data[11:8]}  << 1);
                func3 = i_IMEM_data[14:12];
                func7 = 7'b0;
            end

            7'b0010111: begin //U-type
                register_source_1 = 0;
                register_source_2 = 0;
                register_destination = i_IMEM_data[11:7];
                immediate = $signed(i_IMEM_data[31:12] << 12);
                func3 = 3'b0;
                func7 = 7'b0;
            end

            7'b1101111: begin //UJ-type
                register_source_1 = 0;
                register_source_2 = 0;
                register_destination = i_IMEM_data[11:7];
                immediate = $signed({i_IMEM_data[31], i_IMEM_data[19:12], i_IMEM_data[20], i_IMEM_data[30:21]} << 1);
                func3 = 3'b0;
                func7 = 7'b0;
            end

            default: begin
                register_source_1 = 0;
                register_source_2 = 0;
                register_destination = 0;
                immediate = 0;
                func3 = 3'b0;
                func7 = 7'b0;
            end
        endcase

        case (MemtoReg) //update write_data (MUX3)
            0: begin
                case (opcode) 
                    aupicop: begin
                        write_data = $signed(PC) +$signed(immediate);
                    end
                    jalop: begin
                        write_data = PC + 4;
                    end
                    default: begin
                    write_data = o_DMEM_addr; 
                    end
                endcase                        
            end
            1: begin
                write_data = i_DMEM_rdata; 
            end
            default: begin
                write_data = 0;
            end
        endcase
    end
    
    always @(*) begin //update r_data_1, r_data_2, mul_done_reg
        read_data_1 = rdata1;
        read_data_2 = rdata2;
        mul_done_reg = mul_done;
    end

    always @(*) begin //update i_Breg (MUX2)
        case (ALUSrc)
            0: begin
                i_Breg = read_data_2; 
            end
            1: begin
                i_Breg = immediate; 
            end
        endcase
    end

    always @(*) begin //choose which output to be used (MUX4)
        case (mul_en)
            1'b1: begin
                final_op = o_MULDIV_wire;
            end
            1'b0: begin
                final_op = o_ALU_wire;
            end
            default: begin
                final_op = o_ALU_wire;
            end
        endcase
    end

    always @(*) begin //if multiplication is being executed
        case ({opcode, func7})
            {7'b0110011, 7'b0000001}: begin
                mul_en = 1'b1;
            end
            default begin
                mul_en = 1'b0;
            end
        endcase
    end

    always @(*) begin //multiplcation ready to load input
        if ( (!mul_en_old && mul_en) || (mul_done_reg_old && !mul_done_reg) ) begin
            mul_valid = 1;
        end
        else begin
            mul_valid = 0;
        end
    end

    always @(*) begin //deciding whether multiplication counter can execute
        if ( mul_en && !mul_done_reg ) begin
            mul_counter_valid = 1;
        end
        else begin
            mul_counter_valid = 0;
        end
    end

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
            state <= 1'b1;
            mul_en_old <= mul_en;
            mul_done_reg_old <= mul_done_reg;
        end
        else begin
            PC <= next_PC;
            state <= state_nxt;
            mul_en_old <= mul_en;
            mul_done_reg_old <= mul_done_reg;
        end
    end
endmodule



module Reg_file(i_clk, i_rst_n, wen, rs1, rs2, rd, wdata, rdata1, rdata2);
   
    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth
    
    input i_clk, i_rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] wdata;
    input [addr_width-1:0] rs1, rs2, rd;

    output [BITS-1:0] rdata1, rdata2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign rdata1 = mem[rs1];
    assign rdata2 = mem[rs2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (rd == i)) ? wdata : mem[i];
    end

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'hbffffff0;
                    32'd3: mem[i] <= 32'h10008000;
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end       
    end
endmodule

module ALU #(
    parameter BIT_W = 32
)(
    input                       i_clk,   // clock
    input                       i_rst_n, // reset

    input [BIT_W - 1 : 0]       i_A,     // input operand A
    input [BIT_W - 1 : 0]       i_B,     // input operand B
    input [        3 : 0]       ALUOp,  // instruction

    output [2*BIT_W - 1 : 0]    o_data  // output value

);
    reg  [ BIT_W-1: 0 ] i_a;
    reg  [ BIT_W-1: 0 ] i_b;
    reg  [ 3: 0 ] aluop;
    reg  [ 2 * BIT_W-1: 0] op;

    assign o_data = op;

    always @(*) begin
        i_a = i_A;
        i_b = i_B;
        case (ALUOp)
            4'b0000: begin // add, addi
                op = $signed(i_a) + $signed(i_b);
            end
            4'b0001: begin // sub
                op = $signed(i_a) - $signed(i_b);
            end
            4'b0010: begin // and
                op = i_a & i_b;
            end
            4'b0011: begin // xor
                op = i_a ^ i_b;
            end
            4'b0100: begin // slli
                op = i_a << i_b;
            end
            4'b0101: begin // slti
                op = ($signed(i_a) < $signed(i_b)) ? {63'b0, 1'b1} : {63'b0, 1'b0};
            end
            4'b0110: begin // srai
                op = i_a >>> i_b;
            end
            4'b1001: begin //beq
                op = (($signed(i_a) - $signed(i_b)) == 0) ? 1 : 0;
            end
            4'b1010: begin //bge
                op = (($signed(i_a) - $signed(i_b)) >= 0) ? 1 : 0;
            end      
            4'b1011: begin //blt
                op = (($signed(i_a) - $signed(i_b)) < 0) ? 1 : 0;
            end
            4'b1100: begin //bne
                op = (($signed(i_a) - $signed(i_b)) != 0) ? 1 : 0;
            end
            default: begin
                op = 0;
            end
        endcase
    end
endmodule

module MULDIV_unit #(
    parameter BIT_W = 32
)(
    input                       i_clk,   // clock
    input                       i_rst_n, // reset

    input                       i_valid, // input valid signal
    input [BIT_W - 1 : 0]       i_A,     // input operand A
    input [BIT_W - 1 : 0]       i_B,     // input operand B
    input                       i_counter_valid,
    output [2*BIT_W - 1 : 0]    o_data,  // output value
    output                      o_done   // output valid signal

);
    reg  [  BIT_W-1: 0] operand_a, operand_a_nxt;
    reg  [  BIT_W-1: 0] operand_b, operand_b_nxt;
    reg  [  2*BIT_W: 0] result, result_nxt;    
    reg  [         4: 0] counter, counter_nxt;
    reg valid_signal, valid_signal_nxt;
    reg counter_accu;
    reg result_reset;
    assign o_data = result;
    assign o_done = valid_signal;

    always @(*) begin
        if(!i_valid && i_counter_valid && counter < 31) begin
            counter_nxt = counter + 1;
        end
        else begin
            counter_nxt = 0;
        end
    end

    always @(*) begin
        if (i_valid && counter === 0) begin
            operand_a_nxt = i_A;
            operand_b_nxt = i_B;
        end
        else begin
            operand_a_nxt = operand_a;
            operand_b_nxt = operand_b;
        end
        valid_signal_nxt = (counter == 31) ? 1 : 0;
    end

    always @(*) begin
        if (i_valid && counter === 0) begin
            result_nxt = 0;
        end
        else begin
            result_nxt = (operand_b[counter]) ? (result + ({33'b0, operand_a} << counter)) : result;    
            result_nxt = (counter == 31) ? result_nxt[31:0] : result_nxt;
        end
    end

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n ) begin
            result      <= 0;
            valid_signal <= 0;
            operand_a   <= 0;
            operand_b   <= 0;
            counter <= 0;
        end
        else begin
            result      <= result_nxt;
            valid_signal <= valid_signal_nxt;
            operand_a   <= operand_a_nxt;
            operand_b   <= operand_b_nxt;
            counter <= counter_nxt;
        end
    end
endmodule

module ControlUnit(
    input[6:0]      opcode,
    input[2:0]      func3,
    input[6:0]      func7,
    output[1:0]     Branch,
    output          MemRnW,
    output          MemtoReg,
    output[3:0]     ALUOp,
    output          MemWrite,
    output          ALUSrc,
    output          RegWrite
);
    reg memrnw, memtoreg, memwrite, alusrc, regwrite;
    reg [1:0] branch;
    reg [3:0] aluop;

    assign Branch = branch;
    assign MemRnW = memrnw;
    assign MemtoReg = memtoreg;
    assign ALUOp = aluop;
    assign MemWrite = memwrite;
    assign ALUSrc = alusrc;
    assign RegWrite = regwrite;

    parameter aupicop = 7'b0010111;

    always @(*) begin
        case (opcode)
            // R-type instructions
            7'b0110011: begin
                branch = 2'b00;
                memrnw = 0;
                memtoreg = 0;
                case (func3)
                    3'b000: begin
                        case (func7)
                            7'b0000000: aluop = 4'b0000; // add
                            7'b0100000: aluop = 4'b0001; // sub
                            7'b0000001: aluop = 4'b0111; // mul
                            default: aluop = 4'b0000;
                        endcase
                    end
                    3'b111: aluop = 4'b0010; // and
                    3'b100: begin
                        case(func7)
                            7'b0000000: aluop = 4'b0011; // xor
                            7'b0000001: aluop = 4'b1000; // div
                            default: aluop = 4'b0000;
                        endcase
                    end
                    default: aluop = 4'b0000;
                endcase
                memwrite = 0;
                alusrc = 0;
                regwrite = 1;
            end

            // Load instructions
            7'b0000011: begin
                branch = 2'b00;
                memrnw = 1;
                memtoreg = 1;
                aluop = 4'b0000;
                memwrite = 0;
                alusrc = 1;
                regwrite = 1;
            end

            // Store instructions
            7'b0100011: begin
                branch = 2'b00;
                memrnw = 1;
                memtoreg = 0;
                aluop = 4'b0000;
                memwrite = 1;
                alusrc = 1;
                regwrite = 0;
            end

            // branch instructions
            7'b1100011: begin
                branch = 2'b01;
                memrnw = 0;
                memtoreg = 0;
                case (func3)
                    3'b000: aluop = 4'b1001; // beq
                    3'b101: aluop = 4'b1010; // bge
                    3'b100: aluop = 4'b1011; // blt
                    3'b001: aluop = 4'b1100; // bne
                    default: aluop = 4'b0000;
                endcase
                memwrite = 0;
                alusrc = 0;
                regwrite = 0;
            end

            // Immediate instructions
            7'b0010011: begin
                branch = 2'b00;
                memrnw = 0;
                memtoreg = 0;
                case (func3)
                    3'b000: aluop = 4'b0000; // addi
                    3'b001: aluop = 4'b0100; // slli
                    3'b010: aluop = 4'b0101; // slti
                    3'b101: aluop = 4'b0110; // srai
                    default: aluop = 4'b0000;
                endcase
                memwrite = 0;
                alusrc = 1;
                regwrite = 1;
            end

            // Jal instructions
            7'b1101111: begin
                branch = 2'b10;
                memrnw = 0;
                memtoreg = 0;
                aluop = 4'b0000;
                memwrite = 0;
                alusrc = 0;
                regwrite = 1;
            end

            // Jalr instructions
            7'b1100111: begin
                branch = 2'b11;
                memrnw = 0;
                memtoreg = 0;
                aluop = 4'b0000;
                memwrite = 0;
                alusrc = 0;
                regwrite = 1;
            end

            // Ecall instruction
            7'b1110011: begin
                branch = 2'b00;
                memrnw = 0;
                memtoreg = 0;
                aluop = 4'b0000;
                memwrite = 0;
                alusrc = 0;
                regwrite = 0;
            end

            // aupic instruction
            aupicop: begin
                branch = 2'b10;
                memrnw = 0;
                memtoreg = 0;
                aluop = 4'b0000;
                memwrite = 0;
                alusrc = 1;
                regwrite = 1;
            end

            default: begin
                branch = 2'b00;
                memrnw = 0;
                memtoreg = 0;
                aluop = 4'b0000;
                memwrite = 0;
                alusrc = 0;
                regwrite = 0;
            end
        endcase
    end
endmodule

module Cache#(
        parameter BIT_W = 32,
        parameter ADDR_W = 32
    )(
        input i_clk,
        input i_rst_n,
        // processor interface
        input i_proc_cen,
        input i_proc_wen,
        input [ADDR_W-1:0] i_proc_addr,
        input [BIT_W-1:0]  i_proc_wdata,
        output [BIT_W-1:0] o_proc_rdata,
        output o_proc_stall,
        input i_proc_finish,
        output o_cache_finish,
        // memory interface
        output o_mem_cen,
        output o_mem_wen,
        output [ADDR_W-1:0] o_mem_addr,
        output [BIT_W*4-1:0]  o_mem_wdata,
        input [BIT_W*4-1:0] i_mem_rdata,
        input i_mem_stall,
        output o_cache_available,
        // others
        input  [ADDR_W-1: 0] i_offset
    );

    assign o_cache_available = 1; // change this value to 1 if the cache is implemented

    // parameters
    parameter Not_Called = 1'b0;
    parameter Called = 1'b1;

    parameter Read = 1'b1;  // b/c xor operation
    parameter Write = 1'b0;

    parameter Hit = 1'b1;
    parameter Miss = 1'b0;

    parameter Full = 1'b1;
    parameter Not_Full = 1'b0;

    parameter Dirty = 1'b1;  // for checking dirty
    parameter Not_Dirty = 1'b0;  // for checking dirty

    parameter Valid = 1'b1; 
    parameter Not_Valid = 1'b0;

    // states
    parameter ReadMissNot = 3'b000;     // 1-1
    parameter ReadMissFull = 3'b001;    // 1-2
    parameter ReadHitX = 3'b010;        // 1-3 1-4
    parameter WriteMissNot = 3'b011;    // 1-5
    parameter WriteHitX = 3'b100;       // 1-6 1-8
    parameter WriteMissFull = 3'b101;   // 1-7
    parameter noChange = 3'b110;        // 1-9 // 2-9
    parameter Wait = 3'b111;           

    // useful parameters
    parameter LINE_W = 158;
    parameter LINE_NUM = 16;

    // regs and wires 
    reg [LINE_W - 1:0] cache [0:15]; // 16 lines, 4 words
    reg [LINE_W - 1:0] cache_data_nxt;

    // reg [27:0] tag                   // incoming tag 可以直接從 i_proc_addr 切
    reg [1:0] block_offset;           // incoming offset 可以直接從 i_proc_addr 切

    reg full;
    reg hit;
    reg dirty;
    reg [4:0]line_hit;
    reg [1:0] base [0:LINE_NUM - 1];
    // reg RW, RW_nxt;                  // can be o_proc_cen ^ o_proc_wen
    reg [2:0] state, state_nxt;

    // memory interface
    reg mem_cen;
    reg mem_wen;  
    reg [ADDR_W - 1: 0] mem_addr;
    reg mem_stall;
    reg [127: 0] mem_rdata;
    reg [127: 0] mem_wdata;

    assign o_mem_addr = mem_addr;
    assign o_mem_cen = mem_cen;
    assign o_mem_wen = mem_wen;
    assign o_mem_wdata = mem_wdata;

    // processor interface
    reg [BIT_W - 1: 0] output_data;
    reg proc_stall;
    reg cache_finish;
    reg [4:0] vacant;
    assign o_proc_rdata = output_data;
    assign o_proc_stall = proc_stall;
    assign o_cache_finish = cache_finish;

    //------------------------------------------//
    //          default connection              //
    // assign o_mem_cen = i_proc_cen;              //
    // assign o_mem_wen = i_proc_wen;              //
    // assign o_mem_addr = i_proc_addr;            //
    // assign o_mem_wdata = i_proc_wdata;          //
    // assign o_proc_rdata = i_mem_rdata[0+:BIT_W];//
    // assign o_proc_stall = i_mem_stall;          //
    //------------------------------------------//
    integer i;
    
    always @ (*) begin      // get hit, full and states
        if(i_proc_cen || i_proc_wen) begin
            for (i = 0; i < LINE_NUM; i = i + 1) begin
                hit = ((cache[i][LINE_W - 3:LINE_W - 30] === i_proc_addr[BIT_W - 1:4]) && (cache[i][LINE_W - 2] === 1) && (i_proc_addr[3:2] >= base[i])) ? (Hit || hit) : (Miss || hit);
                full = (cache[i][LINE_W - 2] && full) ? Full : !Full;
            end
        end
        else begin
            hit = 1'b0;
            full = 1'b0;
        end

     //decide state
        casex ({i_proc_cen, i_proc_wen, i_mem_stall, i_proc_finish})
            {1'b1, 1'b0, 1'bx, 1'bx}: begin //read
                case ({hit, full})
                    {Miss, !Full}: begin
                        state_nxt = ReadMissNot;
                    end
                    {Miss, Full}: begin
                        state_nxt = ReadMissFull;
                    end
                    {Hit, !Full}: begin
                        state_nxt = ReadHitX;
                    end
                    {Hit, Full}: begin
                        state_nxt = ReadHitX;
                    end
                    default: begin
                        state_nxt = state;
                    end
                endcase
            end
            {1'b1, 1'b1, 1'bx, 1'b0}: begin //write
                case ({hit, full})
                    {Miss, !Full}: begin
                        state_nxt = WriteMissNot;
                    end
                    {Miss, Full}: begin
                        state_nxt = WriteMissFull;
                    end
                    {Hit, !Full}: begin
                        state_nxt = WriteHitX;
                    end
                    {Hit, Full}: begin
                        state_nxt = WriteHitX;
                    end
                    default: begin
                        state_nxt = state;
                    end
                endcase
            end
            {1'b0, 1'b0, 1'bx, 1'bx}: begin //neither send nor write
                state_nxt = Wait;
            end
            // {1'b1, 1'b1, 1'b0, 1'b1}: begin
            //     state_nxt = Wait;
            // end
            // {1'bx, 1'bx, 1'b1, 1'bx}:begin //stalled by memory
            //     state_nxt = state;
            // end
            default: begin
                state_nxt = state;
            end
        endcase

        line_hit = 5'b11111;
        for (i = 0; i < LINE_NUM; i = i + 1) begin
            line_hit = (i_proc_addr[BIT_W - 1:4] === cache[i][LINE_W - 3:LINE_W - 30] && cache[i][LINE_W - 2] === 1 && (i_proc_addr[3:2] >= base[i])) ?  i : (line_hit !== 5'b11111) ? line_hit + 0 : 5'b11111;     // set false to 5'b11111    如果有 latch 就寫開
            vacant = (cache[i][LINE_W - 2] === 0) ? i : (i !== 0) ? (i - 1) : 5'b11111;   // set false to 5'b11111, if vacant is 5'b11111 then cache is full
        end

        case (state)
            ReadMissNot: begin
                block_offset = i_proc_addr[3:2];
                if(!hit && i_proc_cen) begin
                    cache_data_nxt = cache_finish ? {1'b0, 1'b1, i_proc_addr[ADDR_W - 1:4], i_mem_rdata} : cache[vacant];
                    proc_stall = i_mem_stall;
                    cache_finish = !i_mem_stall;
                    mem_cen = cache_finish ? 1'b0 : 1'b1;
                    mem_wen = 1'b0;
                    mem_addr = i_proc_addr;
                    output_data = i_mem_rdata[(i_proc_addr[3:2] - base[vacant]) * BIT_W +: BIT_W];

                end
                else if (hit && i_proc_cen) begin
                    cache_data_nxt = cache_finish ? {1'b0, 1'b1, i_proc_addr[ADDR_W - 1:4], i_mem_rdata} : cache[line_hit];
                    proc_stall = 1'b0;
                    cache_finish = !i_mem_stall;
                    mem_cen = cache_finish ? 1'b0 : 1'b1;
                    mem_wen = 1'b0;
                    mem_addr = i_proc_addr;
                    output_data = cache[line_hit][(i_proc_addr[3:2] - base[line_hit]) * BIT_W +: BIT_W];
                end
                else begin  // !i_proc_cen
                    cache_data_nxt = cache[vacant];
                    proc_stall = i_mem_stall;
                    cache_finish = !i_mem_stall;
                    mem_cen = cache_finish ? 1'b0 : 1'b1;
                    mem_wen = 1'b0;
                    mem_addr = i_proc_addr;
                    output_data = 32'bx;
                end
            end

            ReadMissFull: begin
                
            end

            ReadHitX: begin
                if(i_proc_cen) begin
                    cache_data_nxt = cache[line_hit];
                    output_data = cache[line_hit][(i_proc_addr[3:2] - base[line_hit]) * BIT_W +: BIT_W];
                    cache_finish = 1'b1;
                end
                else begin
                    cache_data_nxt = cache[vacant];
                    proc_stall = i_mem_stall;
                    cache_finish = !i_mem_stall;
                    mem_cen = 1'b0;
                    mem_wen = 1'b0;
                    mem_addr = i_proc_addr;
                    output_data = 32'bx;
                end
            end
            WriteMissNot: begin
                mem_cen = 1'b1;
                mem_wen = 1'b0;
                mem_addr = i_proc_addr[ADDR_W - 1:4] << 4;
                cache_data_nxt = {1'b1, 1'b1, i_proc_addr[31:4], i_mem_rdata};
                cache_data_nxt[i_proc_addr[3:2] * BIT_W +: BIT_W] = i_proc_wdata;
                proc_stall = i_mem_stall;
            end

            WriteHitX: begin
                cache[line_hit][(i_proc_addr[3:2] - base[line_hit]) * BIT_W +: BIT_W] = i_proc_wdata;
                cache[line_hit][LINE_W - 1] = 1'b1;
                cache_finish = 1'b1;
            end

            WriteMissFull: begin
               
            end

            noChange: begin
                cache_finish = i_mem_stall ? 1'b0 : 1'b1;
                mem_cen = i_proc_cen;
                mem_wen = i_proc_wen;
                mem_addr = i_proc_addr;
                cache_data_nxt = {1'b0, 1'b0, i_proc_addr[31:4], i_mem_rdata};
                proc_stall = i_mem_stall;
                output_data = cache[vacant][(i_proc_addr[3:2] - base[vacant]) * BIT_W +: BIT_W];
  
            end

            Wait: begin
                if(!i_proc_finish) begin
                    cache_finish = !i_mem_stall;

                    if(!hit && i_proc_cen) begin
                        cache_data_nxt = i_mem_stall ? {1'b0, 1'b1, i_proc_addr[ADDR_W - 1:4], i_mem_rdata} : cache[vacant];
                        proc_stall = i_mem_stall;
                        mem_cen = i_proc_cen;
                        mem_wen = i_proc_wen;
                        mem_addr = i_proc_addr;
                        base[vacant] = i_proc_addr[3:2];
                        output_data = i_mem_rdata[(i_proc_addr[3:2] - base[vacant]) * BIT_W +: BIT_W];
                    end
                    else if(hit && i_proc_cen) begin
                        cache_data_nxt = i_mem_stall ? {1'b0, 1'b1, i_proc_addr[ADDR_W - 1:4], i_mem_rdata} : cache[line_hit];
                        proc_stall = 1'b0;
                        mem_cen = 1'b0;
                        mem_wen = 1'b0;
                        mem_addr = i_proc_addr;
                        output_data = cache[line_hit][(i_proc_addr[3:2] - base[line_hit]) * BIT_W +: BIT_W];
                    end
                    else begin  // !i_proc_cen
                        cache_data_nxt = cache[vacant];
                        proc_stall = i_mem_stall;
                        cache_finish = !i_mem_stall;
                        mem_cen = cache_finish ? 1'b0 : 1'b1;
                        mem_wen = 1'b0;
                        mem_addr = i_proc_addr;
                        output_data = 32'bx;
                    end
                end
                else begin
                    cache_finish = !i_mem_stall;
                    mem_cen = 1'b1;
                    mem_wen = 1'b1;
                    for(i = 0; i < LINE_NUM; i = i + 1) begin
                        if(cache[i][LINE_W - 1] === 1) begin
                            mem_addr = (cache[i][LINE_W - 3:LINE_W - 30] << 4) + 4;
                            mem_wdata = cache[i][127:0];
                        end
                    end

                    if(!hit) begin
                        cache_data_nxt = i_mem_stall ? {1'b0, 1'b1, i_proc_addr[ADDR_W - 1:4], i_mem_rdata} : cache[vacant];
                    end
                    else begin
                        cache_data_nxt = i_mem_stall ? {1'b0, 1'b1, i_proc_addr[ADDR_W - 1:4], i_mem_rdata} : cache[line_hit];
                    end
                        proc_stall = 1'b1;
                end
            end
        endcase
    
    end


    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n ) begin
            state <= Wait;
            hit <= 0;
            full <= 0;
            dirty <= 0;
            mem_cen <= 0;
            mem_wen <= 0;
            for (i = 0; i < LINE_NUM; i = i + 1) begin
                cache[i][LINE_W - 1: 0] <= 158'b0;
            end
        end
        else if(i_proc_finish) begin
            mem_cen <= 1;
            mem_wen <= 1;
            state <= state_nxt;
            for (i = 0; i < LINE_NUM; i = i + 1) begin
                if(cache[i][LINE_W - 1] === 1) begin
                                        
                end
            end
        end
        else begin
            state <= state_nxt;
            if(!hit && cache_finish) begin
                cache[vacant][LINE_W - 1: 0] <= cache_data_nxt;
            end
            else begin
                cache[line_hit][LINE_W - 1: 0] <= cache_data_nxt;
            end
        end
    end
    // Todo: BONUS

endmodule