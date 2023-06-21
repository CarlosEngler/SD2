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

module full_adder ( 
  input [22:0] NA,
  input [22:0] NB,
  input carryIn,
  output [22:0] soma_total,
  output [22:0] lista_carryOut
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
 
    for (i=1; i < 23; i = i+1) begin
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

module Big_ULA (
  input [22:0] A,
  input [22:0] B,
  input [3:0] lists,
  output [22:0] resultado,
  output [3:0] alu_flags
);


endmodule

module Small_Ula (
  input [7:0] A,
  input [7:0] B,
  output [7:0] resultado
);

    wire menor_valor;
    wire [7:0] inverte;
    wire [7:0] entrada_A;
    wire subtrador;
    wire [22:0] resultado23bits;
    wire [22:0] lista_carryOut;

    assign subtrador = 1;
    assign menor_valor = (A < B) ? 1 : 0;

    assign entrada_A = (menor_valor) ? B : A;
    assign inverte = (menor_valor) ? ~A : ~B;

        full_adder UUT(
        .NA({15'd0, entrada_A}),
        .NB({15'd0, inverte}),
        .carryIn(subtrador),
        .soma_total(resultado23bits),
        .lista_carryOut(lista_carryOut)
        );

    assign resultado = [7:0]resultado23bits; 


endmodule

module Somador_subtrador (
  input [7:0] A,
  input subtrador,
  output [7:0] resultado,
);

wire [22:0] resultado23bits;
wire [7:0] B;
wire [7:0] B_final;
wire [22:0] lista_carryOut;


assign B = 8'd1;
assign B_final = (subtrador) ? ~B : B; // inverte bits para a subtracao

// somador
full_adder UUT(
    .NA({15'd0, A}),
    .NB({15'd0, B_final}),
    .carryIn(subtrador),
    .soma_total(resultado23bits),
    .lista_carryOut(lista_carryOut)
  );

  assign resultado = [7:0]resultado23bits; 

endmodule

module Mux_2_23bits (
  input [22:0] S1,
  input [22:0] S0,
  input decisor,
  output [22:0] S
);
  // mux que decide S1 se decisor = 1 e S0 se for igual a zero
  assign S = decisor ? S1 : S0;

endmodule

module Mux_2_8bits (
  input [7:0] S1,
  input [7:0] S0,
  input decisor,
  output [7:0] S
);
  // mux que decide S1 se decisor = 1 e S0 se for igual a zero
  assign S = decisor ? S1 : S0;

endmodule

module registrador(
  // registrador de 8 bits
  input clk,
  input [7:0] entrada,
  output reg [7:0] saida
);

  always @(posedge clk) begin
         saida <= entrada;
 end

endmodule

module Shift_Right(
    input [22:0] entrada,
    input [4:0] tamanho,
    output [22:0] saida
);

    assign saida = {tamanho'b0, [22:tamanho]entrada};

endmodule

module Shift_Right_left(
    input [22:0] entrada,
    input [4:0] tamanho,
    input decisor,
    output [22:0] saida
);
    assign saida = (decisor) ? {tamanho'b0, [22:tamanho]entrada} : {[(22-tamanho):0]entrada, tamanho'b0};
                                //direita              //esquerda
endmodule

module arredondamento(
    
);

endmodule

module Datapath(
    input clk,
    input [31:0] input_1,
    input [31:0] input_2,
    input decisor_mux_expoentes,
    input decisor_mux_expoente_escolhido,
    input decisor_mux_escolhe_shift_right,
    input decisor_mux_entrada_dois_ula,
    input decisor_mux_saida_big_ula,
    input decisor_shift_right_left,
    input subtrador_Somador_subtrador, 
    output reg [22:0] saida_registrador
);

    wire [7:0] resultado;
    wire [7:0] saida_mux_expoentes;
    wire [7:0] saida_mux_expoentes_escolhido;
    wire [7:0] saida_subtrador_somador;

    wire[22:0] saida_escolhe_shift_right;
    wire[22:0] saida_escolhe_entrada_dois_ula;
    wire[22:0] saida_shift_right;

    wire[22:0] saida_big_ula;
    wire[22:0] mux_saida_big_ula;

    wire[22:0] saida_shift_right_left;


    Small_Ula pequena(.A(input_1[30:23]), .B(input_2[30:23]), .resultado(resultado));
    registrador ula_pequena(.clk(clk), .entrada(resultado), .saida(saida_registrador));

    //parte da esquerda
    Mux_2_8bits Expoentes1_2(.S0(input_1[30:23]), .S1(input_2[30:23]), .S(saida_mux_expoentes), .decisor(decisor_mux_expoentes)); //escolhe o menor expoente
    Mux_2_8bits ExpoenteEscolhido_final(.S0(saida_mux_expoentes), .S1(), .S(saida_mux_expoentes_escolhido), .decisor(decisor_mux_expoente_escolhido));
    Somador_subtrador incrementa_subtrai(.A(saida_segundo_mux), .subtrador(subtrador_Somador_subtrador), .resultado(saida_subtrador_somador));

    //parte da direita
    Mux_2_23bits escolhe_shift_right(.S0(input_1[22:0]), .S1(input_2[22:0]), .S(saida_escolhe_shift_right), .decisor(decisor_mux_escolhe_shift_right));
    Shift_Right direita(.entrada(saida_escolhe_shift_right), .saida(saida_shift_right)); //verificar isso
    Mux_2_23bits escolhe_entrada_dois_ula(.S0(input_2[22:0]), .S1(input_1[22:0]), .S(saida_escolhe_entrada_dois_ula), .decisor(decisor_mux_entrada_dois_ula));
    Big_ULA grande_ula(.A(saida_shift_right), .B(saida_escolhe_entrada_dois_ula), .resultado(saida_big_ula)); //verificar isso

    //parte da final
    Mux_2_23bits saida_ula_grande(.S0(saida_big_ula), .S1(), .S(mux_saida_big_ula), .decisor(decisor_mux_saida_big_ula));
    Shift_Right_left direita_esquerda(.entrada(mux_saida_big_ula), .saida(saida_shift_right_left), .decisor(decisor_shift_right_left)); //verificar isso
    arredondamento arredonda(); //falta coisa


endmodule


module testbench();

    reg clk;
    reg [31:0] input_1;
    reg [31:0] input_2;
    reg decisor_mux_expoentes;
    reg decisor_mux_expoente_escolhido;
    reg decisor_mux_escolhe_shift_right;
    reg decisor_mux_entrada_dois_ula;
    reg decisor_mux_saida_big_ula;
    reg decisor_shift_right_left;
    reg subtrador_Somador_subtrador;
    reg saida_registrador;
    
    Datapath uut(input clk,
      .input_1(input_1),
      .input_2(input_2),
      .decisor_mux_expoentes(decisor_mux_expoentes),
      .decisor_mux_expoente_escolhido(decisor_mux_expoente_escolhido),
      .decisor_mux_escolhe_shift_right(decisor_mux_escolhe_shift_right),
      .decisor_mux_entrada_dois_ula(decisor_mux_entrada_dois_ula),
      .decisor_mux_saida_big_ula(decisor_mux_saida_big_ula),
      .decisor_shift_right_left(decisor_shift_right_left),
      .subtrador_Somador_subtrador(subtrador_Somador_subtrador), 
      .saida_registrador(saida_registrador)
      );

    always #5 clk= ~clk;
endmodule
