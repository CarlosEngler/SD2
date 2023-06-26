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
  input [25:0] NA,
  input [25:0] NB,
  input carryIn,
  output [25:0] soma_total,
  output [25:0] lista_carryOut
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
 
    for (i=1; i < 26; i = i+1) begin
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

module Multiplicador(
  input [25:0] b,
  input [25:0] a,
  output [51:0] result
);
  wire [51:0] resultado;
  wire [51:0] entrada;
  
  half_multiplica multiplicamuito(.b(1'b0), .a(a), .soma(resultado), .entrada(52'd0));

 generate
    genvar i;
    	for (i = 0; i < 52; i = i + 1) begin
        half_multiplica multiplica(
        .b(b[i]), 
        .a(a), 
        .soma(resultado), 
        .entrada(entrada));
        assign entrada = resultado;
        end
 endgenerate
  
  assign result = resultado;

endmodule

module half_multiplica(
	input b,
  	input [25:0] a,
  	input [51:0] entrada,
  	output [51:0] soma
);
  
  assign soma = (b == 1'b1) ? (entrada + (a << i)) : entrada;
endmodule

//-----------------ULAS----------------

module Big_ULA (
  input [25:0] A,
  input [25:0] B,
  input decisor,
  input subtrador,
  output [51:0] resultado
);

  integer i;

  // se o decisor for 1 -> multiplicacao 0 -> é uma soma
  wire [25:0] resultado26bits;
  wire [51:0] result;
  wire [25:0] lista_carryOut;
  wire [25:0] B_final;

  assign B_final = (B != 25'd0 && subtrador == 1'b1) ? ~B : B;

  full_adder UUT(
        .NA(A),
        .NB(B_final),
        .carryIn(subtrador),
        .soma_total(resultado26bits),
        .lista_carryOut(lista_carryOut)
  );

  Multiplicador UUY(
    .a(A),
    .b(B),
    .result(result)
  );
  
  assign resultado = (decisor) ? {25'b0, lista_carryOut[52], resultado26bits} : result;

endmodule

module Small_Ula (saida
  input [7:0] A,
  input [7:0] B,
  input subtrador,
  output [7:0] resultado
);

    wire menor_valor;
    wire [7:0] inverte;
    wire [7:0] entrada_A;
    wire subtrador;
    wire [25:0] resultado23bits;
    wire [25:0] lista_carryOut;

    assign menor_valor = (A < B) ? 1 : 0;

    assign entrada_A = (menor_valor) ? B : A;
    assign inverte = (!subtrador) ? ((menor_valor) ? A : B) : ((menor_valor) ? ~A : ~B);

        full_adder UUT(
        .NA({18'd0, entrada_A}),
        .NB({18'd0, inverte}),
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

wire [25:0] resultado23bits;
wire [7:0] B;
wire [7:0] B_final;
wire [25:0] lista_carryOut;


assign B = 8'd1;
assign B_final = (subtrador) ? ~B : B; // inverte bits para a subtracao

// somador
full_adder UUT(
    .NA({18'd0, A}),
    .NB({18'd0, B_final}),
    .carryIn(subtrador),
    .soma_total(resultado23bits),
    .lista_carryOut(lista_carryOut)
  );

  assign resultado = [7:0]resultado23bits; 

endmodule

module Mux_2_23bits (
  input [25:0] S1,
  input [25:0] S0,
  input decisor,
  output [25:0] S
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
    input [25:0] entrada,
    input [4:0] tamanho,
    output [25:0] saida
);

    assign saida = entrada>>tamanho;

endmodule

module Shift_Right_left(
    input [22:0] entrada,
    input [4:0] tamanho,
    input decisor,
    output [22:0] saida
);
    assign saida = (decisor) ? entrada<<tamanho : entrada>>tamanho;
                                //esquerda              //direita
endmodule

module arredondamento(
    input clk,
    input [7:0] expoente,
    input [25:0] entrada,
    output [31:0] saida
);

 
reg exponent;
always (posedge clk) begin
 exponent = expoente;
end

reg signal; //se o número é negativo, signal = 0; tem que ligar o cabo

reg fraction = (entrada[2] == 0) ? entrada[25:3] : ((entrada[1:0] > 2'b00) ? entrada[25:3] : (entrada[25:3] + 1) /*esse ultimo caso tem que jogar o número dnv no circuito */ );

assign saida  = {signal, exponent, fraction};

endmodule

module Datapath(
    input clk,
    input [31:0] input_1,
    input [31:0] input_2,
    input [4:0] tamanho,
    input [4:0] tamanho2,
    input soma_multiplica,
    input decisor_mux_expoentes,
    input decisor_mux_expoente_escolhido,
    input decisor_mux_escolhe_shift_right,
    input decisor_mux_entrada_dois_ula,
    input decisor_mux_saida_big_ula,
    input decisor_shift_right_left,
    input subtrador_Somador_subtrador, 
    output reg [31:0] saida_registrador
);

    wire [7:0] resultado;
    wire [7:0] saida_mux_expoentes;
    wire [7:0] saida_mux_expoentes_escolhido;
    wire [7:0] saida_subtrador_somador;

    wire[25:0] saida_escolhe_shift_right;
    wire[25:0] saida_escolhe_entrada_dois_ula;
    wire[25:0] saida_shift_right;

    wire[25:0] saida_big_ula;
    wire[25:0] mux_saida_big_ula;

    wire[25:0] saida_shift_right_left;

    wire [31:0] saida_arredondamento;



    Small_Ula pequena(.A(input_1[30:23]), .B(input_2[30:23]), .resultado(resultado), .subtrador(soma_multiplica));
    registrador ula_pequena(.clk(clk), .entrada(resultado), .saida(saida_registrador));

    //parte da esquerda
    Mux_2_8bits Expoentes1_2(.S0(input_1[30:23]), .S1(input_2[30:23]), .S(saida_mux_expoentes), .decisor(decisor_mux_expoentes)); //escolhe o menor expoente
    Mux_2_8bits ExpoenteEscolhido_final(.S0(saida_mux_expoentes), .S1(), .S(saida_mux_expoentes_escolhido), .decisor(decisor_mux_expoente_escolhido));
    Somador_subtrador incrementa_subtrai(.A(saida_segundo_mux), .subtrador(subtrador_Somador_subtrador), .resultado(saida_subtrador_somador));

    //parte da direita
    Mux_2_23bits escolhe_shift_right(.S0({input_1[22:0], 3'd0}), .S1({input_2[22:0], 3'd0}), .S(saida_escolhe_shift_right), .decisor(decisor_mux_escolhe_shift_right));
    Shift_Right direita(.entrada(saida_escolhe_shift_right), .saida(saida_shift_right), .tamanho(tamanho));
    Mux_2_23bits escolhe_entrada_dois_ula(.S0({input_2[22:0], 3'd0}), .S1({input_1[22:0], 3'd0}), .S(saida_escolhe_entrada_dois_ula), .decisor(decisor_mux_entrada_dois_ula));
    Big_ULA grande_ula(.A(saida_shift_right), .B(saida_escolhe_entrada_dois_ula), .resultado(saida_big_ula));

    //parte da final
    Mux_2_23bits saida_ula_grande(.S0(saida_big_ula), .S1(), .S(mux_saida_big_ula), .decisor(decisor_mux_saida_big_ula));
    Shift_Right_left direita_esquerda(.entrada(mux_saida_big_ula), .saida(saida_shift_right_left), .decisor(decisor_shift_right_left), .tamanho(tamanho2));
    arredondamento arredonda(.expoente(saida_subtrador_somador), .entrada(saida_shift_right_left), .saida(saida), .clk(clk));


assign saida-saida_registrador = saida_arredondamento;

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

    initial begin
    end

    always #5 clk= ~clk;
endmodule
