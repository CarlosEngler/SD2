//---------------------------RF_components---------------------
module RF(we, clk, Rw, Ra, Rb, din, douta, doutb);
    input [4:0] Rw, Ra, Rb; // Rw -> decide o local que vai ser escrito
    input we, clk; // we -> decide se vai ser registrado depois lido, ou so lido
    input [63:0] din; // din o -> valor que vai ser escrito
    output [63:0] douta, doutb;
 
    wire [63:0] reg_outputs [0:31];

    // banco de registradores
    registrador X0(.clk(clk), .enable((we && Rw == 0)), .entrada(din), .reset(1'b0), .saida(reg_outputs[0]));
    registrador X1(.clk(clk), .enable((we && Rw == 1)), .entrada(din), .reset(1'b1), .saida(reg_outputs[1]));
    registrador X2(.clk(clk), .enable((we && Rw == 2)), .entrada(din), .reset(1'b1), .saida(reg_outputs[2]));
    registrador X3(.clk(clk), .enable((we && Rw == 3)), .entrada(din), .reset(1'b1), .saida(reg_outputs[3]));
    registrador X4(.clk(clk), .enable((we && Rw == 4)), .entrada(din), .reset(1'b1), .saida(reg_outputs[4]));
    registrador X5(.clk(clk), .enable((we && Rw == 5)), .entrada(din), .reset(1'b1), .saida(reg_outputs[5]));
    registrador X6(.clk(clk), .enable((we && Rw == 6)), .entrada(din), .reset(1'b1), .saida(reg_outputs[6]));
    registrador X7(.clk(clk), .enable((we && Rw == 7)), .entrada(din), .reset(1'b1), .saida(reg_outputs[7]));
    registrador X8(.clk(clk), .enable((we && Rw == 8)), .entrada(din), .reset(1'b1), .saida(reg_outputs[8]));
    registrador X9(.clk(clk), .enable((we && Rw == 9)), .entrada(din), .reset(1'b1), .saida(reg_outputs[9]));
    registrador X10(.clk(clk), .enable((we && Rw == 10)), .entrada(din), .reset(1'b1), .saida(reg_outputs[10]));
    registrador X11(.clk(clk), .enable((we && Rw == 11)), .entrada(din), .reset(1'b1), .saida(reg_outputs[11]));
    registrador X12(.clk(clk), .enable((we && Rw == 12)), .entrada(din), .reset(1'b1), .saida(reg_outputs[12]));
    registrador X13(.clk(clk), .enable((we && Rw == 13)), .entrada(din), .reset(1'b1), .saida(reg_outputs[13]));
    registrador X14(.clk(clk), .enable((we && Rw == 14)), .entrada(din), .reset(1'b1), .saida(reg_outputs[14]));
    registrador X15(.clk(clk), .enable((we && Rw == 15)), .entrada(din), .reset(1'b1), .saida(reg_outputs[15]));
    registrador X16(.clk(clk), .enable((we && Rw == 16)), .entrada(din), .reset(1'b1), .saida(reg_outputs[17]));
    registrador X17(.clk(clk), .enable((we && Rw == 17)), .entrada(din), .reset(1'b1), .saida(reg_outputs[17]));
    registrador X18(.clk(clk), .enable((we && Rw == 18)), .entrada(din), .reset(1'b1), .saida(reg_outputs[18]));
    registrador X19(.clk(clk), .enable((we && Rw == 19)), .entrada(din), .reset(1'b1), .saida(reg_outputs[19]));
    registrador X20(.clk(clk), .enable((we && Rw == 20)), .entrada(din), .reset(1'b1), .saida(reg_outputs[20]));
    registrador X21(.clk(clk), .enable((we && Rw == 21)), .entrada(din), .reset(1'b1), .saida(reg_outputs[21]));
    registrador X22(.clk(clk), .enable((we && Rw == 22)), .entrada(din), .reset(1'b1), .saida(reg_outputs[22]));
    registrador X23(.clk(clk), .enable((we && Rw == 23)), .entrada(din), .reset(1'b1), .saida(reg_outputs[23]));
    registrador X24(.clk(clk), .enable((we && Rw == 24)), .entrada(din), .reset(1'b1), .saida(reg_outputs[24]));
    registrador X25(.clk(clk), .enable((we && Rw == 25)), .entrada(din), .reset(1'b1), .saida(reg_outputs[25]));
    registrador X26(.clk(clk), .enable((we && Rw == 26)), .entrada(din), .reset(1'b1), .saida(reg_outputs[26]));
    registrador X27(.clk(clk), .enable((we && Rw == 27)), .entrada(din), .reset(1'b1), .saida(reg_outputs[27]));
    registrador X28(.clk(clk), .enable((we && Rw == 28)), .entrada(din), .reset(1'b1), .saida(reg_outputs[28]));
    registrador X29(.clk(clk), .enable((we && Rw == 29)), .entrada(din), .reset(1'b1), .saida(reg_outputs[29]));
    registrador X30(.clk(clk), .enable((we && Rw == 30)), .entrada(din), .reset(1'b1), .saida(reg_outputs[30]));
    registrador X31(.clk(clk), .enable((we && Rw == 31)), .entrada(din), .reset(1'b1), .saida(reg_outputs[31]));

    //retorna o valor referente ao endereco Ra e Rb
    assign douta = reg_outputs[Ra];
    assign doutb = reg_outputs[Rb];

endmodule

//----------------registrador-componentes---------
module registrador(
  // registrador de 64 bits
  input clk,
  input enable,
  input [63:0] entrada,
  input reset,
  output reg [63:0] saida
);

  always @(posedge clk or negedge reset) begin
    // se reset = 0, ele reseto, caso nao so altera o valor salvo caso enable = 1
    if (!reset) begin
      saida <= 64'd0;
    end 
    else begin
        if (enable) begin
            saida <= entrada;
        end
    end
  end

endmodule

//----------------IMM_GEN---------------
module IMM_GEN( // pega o opcode e gera seu imediato respectivo
  input [31:0] entrada,
  output [63:0] saida 
);
    assign saida =  (entrada[6:0] == 7'b0110011) ? 0 :  // R format add,sub,or,and
                    (entrada[6:0] == 7'b0000011) ? {52'b0, entrada[31:20]} : // I format load 
                    (entrada[6:0] == 7'b0010011) ? {52'b0, entrada[31:20]} : // L format addi
                    (entrada[6:0] == 7'b0100011) ? {52'b0, entrada[31:25], entrada[11:7]} : // S format store
                    (entrada[6:0] == 7'b1100011) ? {51'b0, entrada[31], entrada[7], entrada[30:25], entrada[11:8], 1'b0} : // SB branching format
                    (entrada[6:0] == 7'b1101111) ? {43'b0, entrada[31], entrada[19:12], entrada[20], entrada[30:21], 1'b0} :// UJ format jal
                    (entrada[6:0] == 7'b0010111) ? {32'b0, entrada[31:12], 12'b0} : 64'b0;  //U format auipc
                    
endmodule

//--------------ULA_components--------
  //-----------------Somador-componentes---------------

module half_adder ( //somador de 1 bit
  input A,
  input B,
  input carryIn,
  output soma,
  output carryOut
);

assign soma = (A ^ B) ^ carryIn;  
assign carryOut = (A & B) | (A & carryIn) | (B & carryIn);

endmodule

`define NBITS 64

module full_adder ( // somador de nbits
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
  // NBITS = 64
  input [`NBITS-1:0] A,
  input [`NBITS-1:0] B,
  input [3:0] lists,
  output [`NBITS-1:0] resultado,
  output [3:0] alu_flags
);

wire [`NBITS-1:0] B_final;
wire [`NBITS-1:0] resultadoR; //utilizado para instrucoes do tipo R/U/UJ
wire subtrador;
wire Zero; // vale 1 se resultado e igual a zero
wire [`NBITS-1:0] lista_carryOut;

assign subtrador = (lists == 4'd1) ? 1 : 0;
assign Zero = (A == B) ? 1 : 0;
assign B_final = (B != `NBITS'b0 && subtrador == 1) ? ~B : B; // inverte bits para a subtracao

// somador da ULA
full_adder UUT(
    .NA(A),
    .NB(B_final),
    .carryIn(subtrador),
    .soma_total(resultadoR),
    .lista_carryOut(lista_carryOut)
  );
//--------------------------------

assign resultado = (lists == 4'd2) ? (A & B) : (lists == 4'd3) ? (A | B) : resultadoR;
//falta o U e o UJ, mas eles recebem outras entradas

assign alu_flags = {1'b0, lista_carryOut[`NBITS-1],  resultado[63], Zero}; //descobre as flags
                    //0   //overflow                 //MSB          //Zero

endmodule
//-------------------ULA_control------------

  module Ula_control (
    input [3:0] alu_cmd,
    input [31:0] instruction,
    output reg [3:0] lists
  );
                   
    always @* begin
    case(alu_cmd)
        4'b000: begin
            if(instruction[14:12] == 3'b000) begin
                if(instruction[31:25] == 7'd0) lists <= 4'd0;
                else lists <= 4'd1;
            end
            else if(instruction[14:12] == 3'b111) lists <= 4'd2;
            else if(instruction[14:12] == 3'b110) lists <= 4'd3;
            else lists <= 4'dx;
        end
        4'b0001: lists <= 4'd0;
        4'b0010: lists <= 4'd0;
        4'b0011: lists <= 4'd2;
        default: lists <= 4'd0;
    endcase
end

endmodule

//--------------Mux_componentes--------------
module Mux_2 (
  input [63:0] S1,
  input [63:0] S0,
  input decisor,
  output [63:0] S
);
  // mux que decide S1 se decisor = 1 e S0 se for igual a zero
  assign S = decisor ? S1 : S0;

endmodule

module fd 
    #(  // Tamanho em bits dos barramentos
        parameter i_addr_bits = 6,
        parameter d_addr_bits = 6
    )(
        input  clk, rst_n,                   // clock borda subida, reset assíncrono ativo baixo
        output [6:0] opcode,                    
        input  d_mem_we, rf_we,              // Habilita escrita na memória de dados e no banco de registradores
        input  [3:0] alu_cmd,                // ver abaixo
        output [3:0] alu_flags, 
        input  alu_src,                      // 0: rf (doutA ou doutB), 1: imm (imeddiate_generated)
               pc_src,                       // 0: +4, 1: +imm
               rf_src,                       // 0: saida_ula, 1: dout_ram
        output [i_addr_bits-1:0] i_mem_addr, //valor de endereco da IM
        input  [31:0]            i_mem_data, //valor de dados que chega da IM
        output [d_addr_bits-1:0] d_mem_addr, // valor de endereco da DM/ram
        inout  [63:0]            d_mem_data  // valor de dados que entra e sai da ram
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

    wire [63:0] entrada_ula; // valor que entra na entrada B da ula(a auqe pode inverter o sinal)
    wire [63:0] entrada_pc; //saida do mux, entrada PC
    wire [63:0] imeddiate_generated; // saida do imm gen, ou seja, imediato que entra na ula
    wire [63:0] dout_ram; //recebe o valor de input da ram, e entra no din rf

    wire [3:0] lists;  //lista de valores de itens para serem feitos na ULA

    wire [63:0] saida_ula; //saida da ula, entra no din do rf

    Mux_2 plus_pc(.S0(plus_four), .S1(plus_IMM), .decisor(pc_src), .S(entrada_pc));

    full_adder somadorPC0(.NA(64'd4), .NB(exit_PC), .carryIn(1'd0), .soma_total(plus_four), .lista_carryOut());

    full_adder somadorPC1(.NA({imeddiate_generated[62:0], 1'b0}), .NB(exit_PC), .carryIn(1'd0), .soma_total(plus_IMM), .lista_carryOut());

    registrador PC(.enable(1'b1), .clk(clk), .entrada(entrada_pc), .reset(rst_n), .saida(exit_PC));
    
    RF rf( .we(rf_we), .Rw(i_mem_data[11:7]), .Ra(i_mem_data[19:15]), .Rb(i_mem_data[24:20]), .din(din), .douta(doutA), .doutb(doutB), .clk(clk) ); //instancia o RF

    Mux_2 input_ula(.S0(doutB), .S1(imeddiate_generated), .decisor(alu_src), .S(entrada_ula));

    ULA ula(.A(doutA), .B(entrada_ula), .lists(lists), .resultado(saida_ula), .alu_flags(alu_flags));
    
    Ula_control alu_control(.alu_cmd(alu_cmd), .lists(lists), .instruction(i_mem_data)); //instancia a ULA

    Mux_2 exit_data_mem(.S0(saida_ula), .S1(dout_ram), .decisor(rf_src), .S(din));

    IMM_GEN gen(.entrada(i_mem_data), .saida(imeddiate_generated));

    assign d_mem_data = (d_mem_we) ? doutB : 64'bz;
    assign dout_ram = (~d_mem_we) ? d_mem_data : 64'bz;

    assign opcode = i_mem_data[6:0];
    assign i_mem_addr = exit_PC[i_addr_bits-1:0];
    assign d_mem_addr = saida_ula[d_addr_bits-1:0];

endmodule