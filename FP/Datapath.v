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
  input [27:0] NA,
  input [27:0] NB,
  input carryIn,
  output [27:0] soma_total,
  output [27:0] lista_carryOut
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
 
    for (i=1; i < 28; i = i+1) begin
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

//-----------------ULAS----------------

module Big_ULA (
  input [27:0] A,
  input [27:0] B,
  input decisor,
  input subtrador,
  output [27:0] resultado,
  output [7:0] tamanhoShift,
);

  // se o decisor for 1 -> multiplicacao 0 -> é uma soma
  integer i;
  reg [7:0] auxiliar;
  reg [28:0] resultadoAuxiliar;
  reg boolean;

  wire [27:0] resultado26bits;
  wire [27:0] lista_carryOut;
  wire [27:0] B_final;
  reg [47:0] soma;

  assign B_final = (B != 28'd0 && subtrador == 1'b1) ? ~B : B;

  full_adder UUT(
        .NA(A),
        .NB(B_final),
        .carryIn(subtrador),
        .soma_total(resultado26bits),
        .lista_carryOut(lista_carryOut)
  );

//multiplicador
  always @* begin
    soma = 48'd0;
    for(i = 3; i < 26; i = i + 1) begin
        if (B[i] == 1'b1) soma = soma + (A << i);
    end
  end

//normalizador soma
  always@* begin
    auxiliar = 8'd0;
    boolean = 1'd1;
    resultadoAuxiliar = resultado26bits;
      for(i = 25; i >= 0; i = i - 1) begin
        if(resultadoAuxiliar[i] == 0 && boolean)
        begin
          auxiliar = auxiliar + 1;
        end
        else boolean = 1'b0;
      end
  end
  
  assign resultado = (decisor) ? resultadoAuxiliar[27:0] : soma[47:25];
  assign tamanhoShift = auxiliar;

endmodule

module Small_Ula (
  input [7:0] A,
  input [7:0] B,
  input subtrador,
  output Bmaior,
  output [7:0] resultado
);

    wire menor_valor;
    wire [7:0] inverte;
    wire [7:0] entrada_A;
    wire [27:0] resultado23bits;
    wire [27:0] lista_carryOut;

    assign menor_valor = (A < B) ? 1 : 0;

    assign entrada_A = (menor_valor) ? B : A;
    assign inverte = (!subtrador) ? ((menor_valor) ? A : B) : ((menor_valor) ? ~A : ~B);

        full_adder UUT(
        .NA({20'd0, entrada_A}),
        .NB({20'd0, inverte}),
        .carryIn(subtrador),
        .soma_total(resultado23bits),
        .lista_carryOut(lista_carryOut)
        );

    assign resultado = (subtrador) ? resultado23bits[7:0] : (A + B - 8'd127); //decisor = 0 -> multiplicacao, decisor = 1 -> soma.
    assign Bmaior = menor_valor;

endmodule

module Somador_subtrador (
  input [7:0] A,
  input [7:0] n_shifts,
  input subtrador,
  output [7:0] resultado
);

wire [27:0] resultado23bits;
wire [7:0] B_final;
wire [27:0] lista_carryOut;

assign B_final = (subtrador) ? ~n_shifts : n_shifts; // inverte bits para a subtracao

// somador
full_adder UUT(
    .NA({20'd0, A}),
    .NB({20'd0, B_final}),
    .carryIn(subtrador),
    .soma_total(resultado23bits),
    .lista_carryOut(lista_carryOut)
  );

  assign resultado = resultado23bits[7:0]; 

endmodule

module Mux_2_23bits (
  input [27:0] S1,
  input [27:0] S0,
  input decisor,
  output [27:0] S
);
  // mux que decide S1 se decisor = 1 e S0 se for igual a zero
  assign S = decisor ? S1 : S0;

endmodule

module Mux_3_8bits (
  input [27:0] S1,
  input [27:0] S0,
  input [27:0] S2
  input [1:0] decisor,
  output [27:0] S
);
  //S0 00 // S1 01 // S2 10
  assign S = (decisor[1]) ? S2 : ((decisor[0]) ? S1 : S0);

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
    input [27:0] entrada,
    input [4:0] tamanho,
    output [27:0] saida
);

    assign saida = entrada>>tamanho;

endmodule

module Shift_Right_left(
    input [27:0] entrada,
    input [4:0] tamanho,
    input decisor,
    output [27:0] saida
);
    assign saida = (decisor) ? entrada<<tamanho : entrada>>tamanho;
                                //esquerda              //direita
endmodule

module arredondamento(
    input clk,
    input [7:0] expoente,
    input load,
    input [27:0] entrada,
    output [27:0] saida_fraction,
    output overflow,
    output [7:0] expoente_saida
);

wire overflow_auxiliar;

wire [23:0] fraction;

reg [7:0] exponent;

always @(posedge load) begin 
  exponent <= expoente;
end

assign fraction = (entrada[2:0] > 3'b100) ? (entrada[25:3] + 1) : (entrada[2:0] == 3'b100) ? ((entrada[3] == 1'b0) ? entrada[25:3] : (entrada[25:3] + 1)) : entrada[25:3];

assign overflow_auxiliar = fraction[23];

assign overflow = fraction[23];

assign saida_fraction = (overflow_auxiliar) ?  entrada : {2'b00, fraction[22:0], 3'b000}; 

assign expoente_saida = exponent;

endmodule

module Datapath(
    input clk,
    input load,
    input [31:0] input_1,
    input [31:0] input_2,
    input [4:0] tamanho,
    input [4:0] tamanho2,
    input [7:0] tamanho3,
    input soma_multiplica_small_ula,
    input soma_multiplica_big_ula,
    input [1:0] decisor_mux_expoente_escolhido,
    input decisor_mux_saida_big_ula,
    input decisor_shift_right_left,
    input subtrador_big_ula,
    input subtrador_Somador_subtrador,
    output overflow,
    output [7:0]tamanhoShift, 
    output [7:0] saida_registrador,
    output [31:0] saida_final,
    output [27:0] data_out_big_ula
);

    wire [7:0] resultado;
    wire Bmaior;
    wire [7:0] saida_mux_expoentes;
    wire [7:0] saida_mux_expoentes_escolhido;
    wire [7:0] saida_subtrador_somador;

    wire[27:0] saida_escolhe_shift_right;
    wire[27:0] saida_escolhe_entrada_dois_ula;
    wire[27:0] saida_shift_right;

    wire[27:0] saida_big_ula;
    wire[27:0] mux_saida_big_ula;

    wire[27:0] saida_shift_right_left;

    wire [31:0] saida_arredondamento;
    wire [7:0] saida_arredonda_expoente;
    wire [25:0] saida_arredonda_fracao;


    Small_Ula pequena(.A(input_1[30:23]), .B(input_2[30:23]), .resultado(resultado), .subtrador(soma_multiplica_small_ula), .Bmaior(Bmaior)); //retorna a diferenca de exponet no caso da soma, e soma expoente no caso da multiplicação, alem de retornar qual input tinha maior expoente
    registrador ula_pequena(.clk(clk), .entrada(resultado), .saida(saida_registrador));

    //parte da esquerda
    Mux_2_8bits Expoentes1_2(.S0(input_1[30:23]), .S1(input_2[30:23]), .S(saida_mux_expoentes), .decisor(~Bmaior)); //escolhe o menor expoente
    Mux_3_8bits ExpoenteEscolhido_final(.S0(saida_mux_expoentes), .S1(saida_arredonda_expoente), .S(saida_mux_expoentes_escolhido), .decisor(decisor_mux_expoente_escolhido)); //falta coisa
    Somador_subtrador incrementa_subtrai(.A(saida_mux_expoentes_escolhido), .subtrador(subtrador_Somador_subtrador), .n_shifts(tamanho3), .resultado(saida_subtrador_somador));

    //parte da direita
    Mux_2_23bits escolhe_shift_right(.S0({2'b01, input_1[22:0], 3'd0}), .S1({2'b01, input_2[22:0], 3'd0}), .S(saida_escolhe_shift_right), .decisor(~Bmaior));
    Shift_Right direita(.entrada(saida_escolhe_shift_right), .saida(saida_shift_right), .tamanho(tamanho));
    Mux_2_23bits escolhe_entrada_dois_ula(.S0({2'b01, input_1[22:0], 3'd0}), .S1({2'b01, input_2[22:0], 3'd0}), .S(saida_escolhe_entrada_dois_ula), .decisor(Bmaior));
    Big_ULA grande_ula(.A(saida_shift_right), .B(saida_escolhe_entrada_dois_ula), .resultado(saida_big_ula), .decisor(soma_multiplica_big_ula), .subtrador(subtrador_big_ula), .tamanhoShift(tamanhoShift));

    //parte da final
    Mux_2_23bits saida_ula_grande(.S0(saida_big_ula), .S1(saida_arredonda_fracao), .S(mux_saida_big_ula), .decisor(decisor_mux_saida_big_ula)); //falta coisa
    Shift_Right_left direita_esquerda(.entrada(mux_saida_big_ula), .saida(saida_shift_right_left), .decisor(decisor_shift_right_left), .tamanho(tamanho2));
    arredondamento arredonda(.expoente(saida_subtrador_somador), .entrada(saida_shift_right_left), .saida_fraction(saida_arredonda_fracao), .expoente_saida(saida_arredonda_expoente), .clk(clk), .overflow(overflow), .load(load));


assign saida_final[31] = input_1[31];
assign saida_final[30:23] = saida_arredonda_expoente;
assign saida_final[22:0] = saida_arredonda_fracao[25:3];

assign data_out_big_ula = saida_big_ula;

endmodule

module UC (
    input clk,
    input rst,
    input start,
    input [31:0] input_1,
    input [31:0] input_2,
    input add_or_mul,
    input overflow_roundign_hardware,
    input [7:0] tamanhoShift,
    input [27:0] data_out_big_ula,
    input [7:0] saida_registrador,
    output [7:0] tamanho_incrementa_decrementa,
    output [4:0] tamanho_shift_right_left, 
    output [4:0] tamanho_shift_right,
    output soma_multiplica_small_ula,
    output soma_multiplica_big_ula,
    output [1:0] decisor_mux_expoente_escolhido,
    output decisor_mux_saida_big_ula,
    output decisor_shift_right_left, // descobre se da shift right ou left
    output subtrador_big_ula,
    output subtrador_incrementa_decrementa,
    output load_rouding_hardware,
    output done,
    output sinal
    );

    reg [3:0] state;
    reg [3:0] next_state;
    // reg aux, mul;

    reg [1:0] increment_mux;
    reg exit_ula_mux, enable, operation_increment, its_over;
    reg operation_shift;
    reg big_ula_operation;

    reg [7:0] increment;
    reg [4:0] shift_R, shift_RL;

    parameter RESET = 3'b000, DIFERENCA_EXPOENTE = 3'b001, OPERATION = 3'b010, 
              NORMALIZACAO = 3'b011, ARREDONDAMENTO = 3'b100, SINAL = 3'b101;

    assign decisor_mux_expoente_escolhido = increment_mux;
    assign decisor_mux_saida_big_ula =  exit_ula_mux;

    assign load_rouding_hardware = enable;
    assign decisor_shift_right_left = operation_shift;
    assign tamanho_shift_right_left = shift_RL;
    assign tamanho_shift_right = shift_R;

    assign tamanho_incrementa_decrementa = increment;
    assign subtrador_incrementa_decrementa = operation_increment;
    assign subtrador_big_ula = big_ula_operation;
    assign done = its_over;


    always @(posedge clk or rst) begin 
        if (rst) begin
            state <= RESET;     
        end
        else begin 
            state <= next_state;
        end
    end

    always@ (state or clk or start)
        begin
            case (state)
                RESET:
                    begin
                        increment_mux <= 2'd0;
                        ula_mux <= 0;
                        enable <= 0;
                        shift_R <= 0;
                        shift_RL <= 0;
                        operation_increment <= 0;
                        its_over <= 0;

                        next_state <= DIFERENCA_EXPOENTE;
                    end
                DIFERENCA_EXPOENTE:
                    begin
                        increment_mux <= 2'd0;
                        exit_ula_mux <= 0;
                        shift_R <= 0;
                        shift_RL <= 0;
                        operation_increment <= 0;
                        its_over <= 0;

                        if (start) begin
                            next_state <= MULTIPLY_OR_ADD;
                        end
                    end
                OPERATION:
                    begin
                        if (!add_or_mul) begin
                          big_ula_operation = 1'b0; //zero-soma, um-multiplica
                          shift_R = saida_registrador;
                        end
                        else begin
                          big_ula_operation = 1'b1;
                          shift_R <= 8'd0;
                        end
                        enable <= 1'b1;
                        next_state <= NORMALIZACAO;
                    end
                NORMALIZACAO:
                    begin
                        enable <= 1'b0;
                        if(!add_or_mul) increment_mux <= 2'd1;
                        else increment_mux <= 2'd2; 
                        exit_ula_mux = 1'b0;
                        if (data_out_big_ula[27:26] == 2'b11 || data_out_big_ula[27:26] == 2'b10) begin
                          //shift
                          shift_RL = 5'd1;
                          operation_shift = 1'b0;
                          //somador
                          increment = 8'd1;
                          operation_increment = 1'b0;
                        end else if(data_out_big_ula[27:26] == 2'b00) begin
                          //shitf -> left
                          shift_RL = tamanhoShift[4:0];
                          operation_shift = 1'b1;
                          //subtrador
                          increment = tamanhoShift;
                          operation_increment = 1'b1;
                        end
                        else begin
                          // não faz nada
                          shift_RL = 8'd0;
                          operation_shift = 1'b0;
                          increment = 8'd0;
                          operation_increment = 1'b0;
                        end
                        next_state <= ARREDONDAMENTO;
                    end
                ARREDONDAMENTO:
                    begin
                        enable <= 1;
                        if (overflow_roundign_hardware) begin
                            increment_mux <= 2'd1;
                            exit_ula_mux <= 1;
                            next_state <= NORMALIZE;
                        end
                        else begin
                            next_state <= SINAL;
                        end
                    end
                SINAL:
                  begin
                    if(!add_or_mul) begin
                      sinal = input_1[31];
                    end
                    else begin
                      sinal = (input_1[31] ^ input_2[31]);
                    end
                    next_state <= DIFERENCA_EXPOENTE;
                    its_over <= 1;
                  end
            endcase
        end 

endmodule


