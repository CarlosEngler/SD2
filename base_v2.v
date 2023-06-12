//---------------------------RF_components---------------------

module RF(we, clk, Rw, Ra, Rb, din, douta, doutb);
    input [4:0] Rw, Ra, Rb;
    input we, clk; //we -> decide se vai ser registrado depois lido, ou so lido
    input [63:0] din;
    output [63:0] douta, doutb;
 
    wire [63:0] reg_outputs [0:31];

    // banco de registradores
    registrador X0(.clk(clk), .load((we && Rw == 0)), .entrada(din), .reset(1'b1), .saida(reg_outputs[0]));
    registrador X1(.clk(clk), .load((we && Rw == 1)), .entrada(din), .reset(1'b0), .saida(reg_outputs[1]));
    registrador X2(.clk(clk), .load((we && Rw == 2)), .entrada(din), .reset(1'b0), .saida(reg_outputs[2]));
    registrador X3(.clk(clk), .load((we && Rw == 3)), .entrada(din), .reset(1'b0), .saida(reg_outputs[3]));
    registrador X4(.clk(clk), .load((we && Rw == 4)), .entrada(din), .reset(1'b0), .saida(reg_outputs[4]));
    registrador X5(.clk(clk), .load((we && Rw == 5)), .entrada(din), .reset(1'b0), .saida(reg_outputs[5]));
    registrador X6(.clk(clk), .load((we && Rw == 6)), .entrada(din), .reset(1'b0), .saida(reg_outputs[6]));
    registrador X7(.clk(clk), .load((we && Rw == 7)), .entrada(din), .reset(1'b0), .saida(reg_outputs[7]));
    registrador X8(.clk(clk), .load((we && Rw == 8)), .entrada(din), .reset(1'b0), .saida(reg_outputs[8]));
    registrador X9(.clk(clk), .load((we && Rw == 9)), .entrada(din), .reset(1'b0), .saida(reg_outputs[9]));
    registrador X10(.clk(clk), .load((we && Rw == 10)), .entrada(din), .reset(1'b0), .saida(reg_outputs[10]));
    registrador X11(.clk(clk), .load((we && Rw == 11)), .entrada(din), .reset(1'b0), .saida(reg_outputs[11]));
    registrador X12(.clk(clk), .load((we && Rw == 12)), .entrada(din), .reset(1'b0), .saida(reg_outputs[12]));
    registrador X13(.clk(clk), .load((we && Rw == 13)), .entrada(din), .reset(1'b0), .saida(reg_outputs[13]));
    registrador X14(.clk(clk), .load((we && Rw == 14)), .entrada(din), .reset(1'b0), .saida(reg_outputs[14]));
    registrador X15(.clk(clk), .load((we && Rw == 15)), .entrada(din), .reset(1'b0), .saida(reg_outputs[15]));
    registrador X16(.clk(clk), .load((we && Rw == 16)), .entrada(din), .reset(1'b0), .saida(reg_outputs[17]));
    registrador X17(.clk(clk), .load((we && Rw == 17)), .entrada(din), .reset(1'b0), .saida(reg_outputs[17]));
    registrador X18(.clk(clk), .load((we && Rw == 18)), .entrada(din), .reset(1'b0), .saida(reg_outputs[18]));
    registrador X19(.clk(clk), .load((we && Rw == 19)), .entrada(din), .reset(1'b0), .saida(reg_outputs[19]));
    registrador X20(.clk(clk), .load((we && Rw == 20)), .entrada(din), .reset(1'b0), .saida(reg_outputs[20]));
    registrador X21(.clk(clk), .load((we && Rw == 21)), .entrada(din), .reset(1'b0), .saida(reg_outputs[21]));
    registrador X22(.clk(clk), .load((we && Rw == 22)), .entrada(din), .reset(1'b0), .saida(reg_outputs[22]));
    registrador X23(.clk(clk), .load((we && Rw == 23)), .entrada(din), .reset(1'b0), .saida(reg_outputs[23]));
    registrador X24(.clk(clk), .load((we && Rw == 24)), .entrada(din), .reset(1'b0), .saida(reg_outputs[24]));
    registrador X25(.clk(clk), .load((we && Rw == 25)), .entrada(din), .reset(1'b0), .saida(reg_outputs[25]));
    registrador X26(.clk(clk), .load((we && Rw == 26)), .entrada(din), .reset(1'b0), .saida(reg_outputs[26]));
    registrador X27(.clk(clk), .load((we && Rw == 27)), .entrada(din), .reset(1'b0), .saida(reg_outputs[27]));
    registrador X28(.clk(clk), .load((we && Rw == 28)), .entrada(din), .reset(1'b0), .saida(reg_outputs[28]));
    registrador X29(.clk(clk), .load((we && Rw == 29)), .entrada(din), .reset(1'b0), .saida(reg_outputs[29]));
    registrador X30(.clk(clk), .load((we && Rw == 30)), .entrada(din), .reset(1'b0), .saida(reg_outputs[30]));
    registrador X31(.clk(clk), .load((we && Rw == 31)), .entrada(din), .reset(1'b0), .saida(reg_outputs[31]));

    //retorna o valor referente ao endereco Ra e Rb
    assign douta = reg_outputs[Ra];
    assign doutb = reg_outputs[Rb];

endmodule

//----------------ram_componentes------------

module ram(saida, address, entrada, we, clk);

  output reg [63:0] saida;
 
  input [63:0] entrada;
  input [63:0] address;
  input we, clk;
 
  reg [63:0] mem [31:0];    
   
    always @(posedge clk) begin
      mem[0] <= 64'd12;
      mem[1] <= 64'd11;
      //a linha 63 e 64 foram setadas apenas para fazer a testbench
      if (we) begin
        mem[address] = entrada;
      end

        saida <= mem[address];

   end
endmodule

//---------------Memória_de_instruções-----------
module InstructionMemory (
  output reg [31:0] saida,
  input clk,
  input [63:0] address,
);

  reg [31:0] mem [31:0];    
   
    always @(posedge clk) begin
      //mem[0] <= 32'b00000000000000000011000010000011; //l-format Load: mem(0) => X1
      //mem[1] <= 32'b00000000000000010011000010000011; //load 

      //mem[0] <= 32'b00000000000000000011000010100011; //s-format Store: X0 -> mem(0+1)
      
      //mem[2] <= 32'b00000000010000010000001100110011;
      //mem[3] <= 32'b00000000000100010000001100110011;   //r-format Add: X6 <= X2 + X4
      //mem[3] <= 32'b01000000001100110000001110110011; //r-format Sub: X7 <= X6 - X3
      //mem[0] <= 32'b00000000100100000000000010010011; //l-format Addi: X1 <= X0 + 9
      //mem[5] <= 32'b00000000101000011000001010010011; //l-format Subi: X5 <= X3 - 10 nota: nossa ula inverte o número passado pra ela dentro de seu próprio funcionamento, portanto passamos o valor do immediate em módulo.
      // Branch functions (SB-format)
      mem[0] <= 32'b00000110000100000100100001100111; //beq
      //mem[7] <= 32'b00000110000100000100100001100111; //bne
      //mem[8] <= 32'b00000110000100000110000001100111; //blt
      //mem[9] <= 32'b00000110000100000101000001100111; //bge
      //mem[10] <=32'b00000110000100000111000001100111; //bltu
      //mem[11] <=32'b00000110000100000111100001100111; //bgeu
     
     //AUIPC, JAL, JALR

      //mem[12] <=32'b00000000000000000010000110110111; //AUIPC 
      //mem[13] <=32'b00000000000000000001000111101111; //JAL 
      //mem[14] <=32'b00000000000100010000011001100111; // JALR 

        saida <= mem[address];

   end
 
endmodule

//----------------registrador_componentes---------

module registrador(
  input clk,
  input load,
  input [63:0] entrada,
  input reset,
  output reg [63:0] saida
);

  always @(posedge clk) begin
    if (reset) begin
      saida <= 64'd0;
    end 
    else begin
        if (load) begin
            saida <= entrada;
        end
    end
  end

endmodule

//----------------IMM_GEN---------------
module IMM_GEN(
  input [31:0] entrada,
  output reg [31:0] saida 
);
    assign = entra[31:20];

endmodule

//--------------ULA_components--------

module half_adder (
  input A,
  input B,
  input carryIn,
  output soma,
  output carryOut
);

assign soma = (A ^ B) ^ carryIn;  
assign carryOut = (A & B) | (A & carryIn) | (B & carryIn);

endmodule

//-----------------Somador_componentes---------------

`define NBITS 64

module full_adder (
  input [`NBITS-1:0] NA,
  input [`NBITS-1:0] NB,
  input carryIn,
  output [`NBITS-1:0] soma_total,
  output [`NBITS-1:0] lista_carryOut
);

 half_adder UUT(
    .A(NA[0]),
    .B(NB[0]),
    .carryIn(carryIn),
    .soma(soma_total[0]),
    .carryOut(lista_carryOut[0])
  );
 
generate
  genvar i;
 
    for (i=1; i < `NBITS; i = i+1) begin
        half_adder UUX(
        .A(NA[i]),
        .B(NB[i]),
        .carryIn(lista_carryOut[i-1]),
        .soma(soma_total[i]),
        .carryOut(lista_carryOut[i])
        );
        end
endgenerate
   
endmodule

//------------------------ULA----------------

module ULA (
  input [`NBITS-1:0] A,
  input [`NBITS-1:0] B,
  input [3:0] alu_cmd,
  output [`NBITS-1:0] resultado,
  output [3:0] alu_flags,
);

wire [`NBITS-1:0] B_final;
wire resultadoR;
wire subtrador;
wire BEQ;
wire [`NBITS-1:0] lista_carryOut;

assign BEQ = (A == B) ? 1 : 0;
assign subtrador <= (4'b0011) ? 1 : 0;
assign B_final = (B != `NBITS'b0 && subtrador == 1) ? ~B : B;

//assign BLT = (A < B_final) ? 1 : 0;
//assign BLTU = (A < B) ? 1 : 0;

full_adder UUT(
    .NA(A),
    .NB(B_final),
    .carryIn(subtrador),
    .soma_total(resultadR),
    .lista_carryOut(lista_carryOut)
  );

assign resultado = (alu_cmd == 4'b0001) ? A & B : (alu_cmd == 4'b0010) ? A | B : resultadoR;
//falta o U e o UJ, mas eles recebem outras entradas

   // always @* begin
   //  // Executar a operação da ALU com base no alu_cmd
   //  case (alu_cmd)
   //      4'b0000: resultado <= resultadoR;  // R: Soma
   //      4'b0001: alu_out = alu_a & alu_b;    // I: AND
   //      4'b0010: alu_out = alu_a | alu_b;    // S: OR
   //      4'b0011: alu_out = alu_a - alu_b;    // SB: Subtração
   //      4'b0100: alu_out = alu_a + imm;      // U: Soma com imediato
   //      4'b0101: alu_out = pc + imm;         // UJ: Soma com imediato e PC
   //      default: alu_out = 32'h0;            // Caso padrão (opcional)
   //  endcase
   // end
 
assign alu_flags = {1'b0, lista_carryOut[`NBITS-1],  resultado[63], BEQ}

endmodule

//--------------Mux_componentes--------------
module Mux_2 (
  input [63:0] S1,
  input [63:0] S0,
  input decisor,
  output [63:0] S
);

  assign S = decisor ? S1 : S0;

endmodule

module fd 
    #(  // Tamanho em bits dos barramentos
        parameter i_addr_bits = 6,
        parameter d_addr_bits = 6
    )(
        input  clk,
        input rst_n,                   // clock borda subida, reset assíncrono ativo baixo
        output [6:0] opcode,                    
        input  d_mem_we, rf_we,              // Habilita escrita na memória de dados e no banco de registradores
        input  [3:0] alu_cmd,                // ver abaixo
        output [3:0] alu_flags, //feito em tese, so nao tenho certeza do MSB
        input  alu_src,                      // 0: rf, 1: imm
               pc_src,                       // 0: +4, 1: +imm
               rf_src,                       // 0: alu, 1:d_mem
        output [i_addr_bits-1:0] i_mem_addr,
        input  [31:0]            i_mem_data,
        output [d_addr_bits-1:0] d_mem_addr,
        inout  [63:0]            d_mem_data

    );
    // AluCmd     AluFlags
    // 0000: R    0: zero
    // 0001: I    1: MSB 
    // 0010: S    2: overflow
    // 0011: SB
    // 0100: U
    // 0101: UJ

    wire [63:0] din; // liga mux da saida da data memory com o din do rf
    wire [63:0] exit_PC; // fio que sai do pc e entra nos somadores

    wire [63:0] plus_four; // caminho do somador que soma 4
    wire [63:0] plus_IMM; // caminho do somador que soma com imm

    wire [63:0] doutA; // saida do register 1/ Ra
    wire [63:0] doutB; // saida do register 2/ Rb

    wire [63:0] entrada_ula;
    wire [63:0] entrada_pc;

    wire [63:0] saida_ula;

    Mux_2 plus_pc(.S0(plus_four), .S1(plus_IMM), .decisor(pc_src), .S(entrada_pc));

    full_adder somadorUC(.NA(64'b4), .NB(exit_PC), .carryIn(1'd0), .soma_total(plus_four), .lista_carryOut());

    full_adder somadorPC(.NA(), .NB(exit_PC), .carryIn(1'd0), .soma_total(plus_IMM), .lista_carryOut());

    registrador PC(.load(load_PC), .clk(clk), .entrada(entrada_pc), .reset(rst_n), .saida(exit_PC));
    
    RF rf( .we(rf_we), .Rw(i_mem_data[11:7]), .Ra(i_mem_data[19:15]), .Rb(i_mem_data[24:20]), .din(din), .douta(doutA), .doutb(doutB), .clk(clk) ); //instancia o RF

    Mux_2 input_ula(.S0(doutB), .S1(entrada_mux_add_sub), .decisor(alu_src), .S(entrada_ula));

    ULA ula(.A(doutA), .B(entrada_ula), .alu_cmd(alu_cmd), .resultado(saida_ula), .alu_flags(alu_flags)); //instancia a ULA

    Mux_2 exit_data_mem(.S0(saida_ula), .S1(d_mem_data), .decisor(rf_src), .S(din));

    assign opcode = i_mem_data[6:0];
    assign i_mem_addr = exit_PC[6:0];
    assign d_mem_addr = saida_ula[i_addr_bits-1:0];

endmodule

// module fdteste 
//     #(  // Tamanho em bits dos barramentos
//         parameter i_addr_bits = 6,
//         parameter d_addr_bits = 6
//     )(
//         input  clk,
//         input rst_n,                   // clock borda subida, reset assíncrono ativo baixo
//         output [6:0] opcode,                    
//         input  d_mem_we, rf_we,              // Habilita escrita na memória de dados e no banco de registradores
//         input  [3:0] alu_cmd,                // ver abaixo
//         output [3:0] alu_flags, //feito em tese, so nao tenho certeza do MSB
//         input  alu_src,                      // 0: rf, 1: imm
//                pc_src,                       // 0: +4, 1: +imm
//                rf_src,                       // 0: alu, 1:d_mem
//         output [i_addr_bits-1:0] i_mem_addr,
//         input  [31:0]            i_mem_data,
//         output [d_addr_bits-1:0] d_mem_addr,
//         inout  [63:0]            d_mem_data
// 
//     );
//     // AluCmd     AluFlags
//     // 0000: R    0: zero
//     // 0001: I    1: MSB 
//     // 0010: S    2: overflow
//     // 0011: SB
//     // 0100: U
//     // 0101: UJ             
// endmodule