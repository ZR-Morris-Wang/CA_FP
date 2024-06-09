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
    parameter jalrop = 7'b110111;
    
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
        reg [BIT_W-1:0] PC, next_PC; // PC memory
        
        //Reg_file
        reg write_enable; //I
        reg [4:0] register_source_1, register_source_2, register_destination; //I
        reg [BIT_W-1; 0] write_data; //I
        reg [BIT_W-1; 0] read_data_1, read_data_2; //O

        wire wen;
        wire [4:0] rs1, rs2, rd;
        wire [BIT_W-1; 0] wdata;
        wire [BIT_W-1; 0] rdata1, rdata2;

        wire mem_cen, mem_wen; //cache/data memory
        wire [BIT_W-1:0] mem_addr, mem_wdata, mem_rdata;
        wire mem_stall;
        
        

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Continuous Assignment
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: any wire assignment

    // Assign wires to F/Fs
        assign wen = write_enable; //1b I

        assign rs1 = register_source_1; //4b I
        assign rs2 = register_source_2;
        assign rd = register_destination;

        assign wdata = write_data; //32b I

        // assign rdata1 = read_data_1; //32b O
        // assign rdata2 = read_data_2;
    
// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Submoddules
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: Reg_file wire connection
    Reg_file reg0(               
        .i_clk  (i_clk),
        .i_rst_n(i_rst_n),
        .wen    (wen),
        .rs1    (rs1),
        .rs2    (rs2),
        .rd     (rd),
        .wdata  (wdata),
        .rdata1 (rdata1),
        .rdata2 (rdata2)
    );

    MULDIV_unit md0(
        .i_clk   (i_clk),
        .i_rst_n (i_rst_n),
        .i_valid (),
        .i_A     (rdata1),
        .i_B     (rdata2),
        .i_inst   (),
        .o_data   (),
        .o_done   (),
    );

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Always Blocks
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // Todo: any combinational/sequential circuit

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
        end
        else begin
            PC <= next_PC;
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

module ALU (
    input                       i_clk,   // clock
    input                       i_rst_n, // reset

    input                       i_valid, // input valid signal
    input [BIT_W - 1 : 0]       i_A,     // input operand A
    input [BIT_W - 1 : 0]       i_B,     // input operand B
    input [        3 : 0]       ALUOp,  // instruction

    output [2*BIT_W - 1 : 0]    o_data,  // output value
    output                      o_done   // output valid signal

);
    always @(posedge i_clk or negedge i_rst_n) begin
        if(i_valid && !i_rst_n) begin
            case (ALUOp)
                4'b0000: begin // add
                    o_data <= i_A + i_B;
                    o_done <= 1;
                end
                4'b0001: begin // sub
                    o_data <= i_A - i_B;
                    o_done <= 1;
                end
                4'b0010: begin // and
                    o_data <= i_A & i_B;
                    o_done <= 1;
                end
                4'b0011: begin // xor
                    o_data <= i_A ^ i_B;
                    o_done <= 1;
                end
                4'b0100: begin // slli
                    o_data <= i_A << i_B;
                    o_done <= 1;
                end
                4'b0101: begin // slti
                    o_data <= ($signed(i_A) < $signed(i_B)) ? {63'b0, 1'b1} : {63'b0, 1'b0};
                    o_done <= 1;
                end
                4'b0110: begin // srai
                    o_data <= i_A >>> i_B;
                    o_done <= 1;
                end
                default: begin
                    o_data <= 0;
                    o_done <= 0;
                end
            endcase
        end
        else begin
            o_data <= 0;
            o_done <= 0;
        end
    end
endmodule

module ControlUnit(
    input[6:0]      opcode,
    input[2:0]      func3,
    input[6:0]      func7,
    output          Branch,
    output          MemRead,
    output          MemtoReg,
    output[2:0]     ALUOp,
    output          MemWrite,
    output          ALUSrc,
    output          RegWrite
);

    always @(*) begin
        case (opcode)
            // R-type instructions
            7'b0110011: begin
                Branch = 0;
                MemRead = 0;
                MemtoReg = 0;
                case (func3)
                    3'b000: begin
                        case (func7)
                            7'b0000000: ALUOp = 4'b0000; // add
                            7'b0100000: ALUOp = 4'b0001; // sub
                            7'b0000001: ALUOp = 4'b0111; // mul
                            default: ALUOp = 4'b0000;
                        endcase
                    end
                    3'b111: ALUOp = 4'b0010; // and
                    3'b100: begin
                        case(func7)
                            ALUOp = 4'b0011; // xor
                            ALUOp = 4'b1000; // div
                            default: ALUOp = 4'b0000;
                        endcase
                    end
                    default: ALUOp = 4'b0000;
                endcase
                MemWrite = 0;
                ALUSrc = 0;
                RegWrite = 1;
            end

            // Load instructions
            7'b0000011: begin
                Branch = 0;
                MemRead = 1;
                MemtoReg = 1;
                ALUOp = 4'b0000;
                MemWrite = 0;
                ALUSrc = 1;
                RegWrite = 1;
            end

            // Store instructions
            7'b0100011: begin
                Branch = 0;
                MemRead = 0;
                MemtoReg = 0;
                ALUOp = 4'b0000;
                MemWrite = 1;
                ALUSrc = 1;
                RegWrite = 0;
            end

            // Branch instructions
            7'b1100011: begin
                Branch = 1;
                MemRead = 0;
                MemtoReg = 0;
                ALUOp = 4'b1111;
                MemWrite = 0;
                ALUSrc = 0;
                RegWrite = 0;
            end

            // Immediate instructions
            7'b0010011: begin
                Branch = 0;
                MemRead = 0;
                MemtoReg = 0;
                case (func3)
                    3'b000: ALUOp = 4'b0000; // addi
                    3'b001: ALUOp = 4'b0100; // slli
                    3'b010: ALUOp = 4'b0101; // slti
                    3'b101: ALUOp = 4'b0110; // srai
                    default: ALUOp = 4'b0000;
                endcase
                MemWrite = 0;
                ALUSrc = 1;
                RegWrite = 1;
            end

            // Jump instructions
            7'b1101111: begin
                Branch = 0;
                MemRead = 0;
                MemtoReg = 0;
                ALUOp = 4'b0000;
                MemWrite = 0;
                ALUSrc = 0;
                RegWrite = 1;
            end

            // Ecall instruction
            7'b1110011: begin
                Branch = 0;
                MemRead = 0;
                MemtoReg = 0;
                ALUOp = 4'b0000;
                MemWrite = 0;
                ALUSrc = 0;
                RegWrite = 0;
            end

            default: begin
                Branch = 0;
                MemRead = 0;
                MemtoReg = 0;
                ALUOp = 4'b0000;
                MemWrite = 0;
                ALUSrc = 0;
                RegWrite = 0;
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

    assign o_cache_available = 0; // change this value to 1 if the cache is implemented

    //------------------------------------------//
    //          default connection              //
    assign o_mem_cen = i_proc_cen;              //
    assign o_mem_wen = i_proc_wen;              //
    assign o_mem_addr = i_proc_addr;            //
    assign o_mem_wdata = i_proc_wdata;          //
    assign o_proc_rdata = i_mem_rdata[0+:BIT_W];//
    assign o_proc_stall = i_mem_stall;          //
    //------------------------------------------//

    // Todo: BONUS

endmodule