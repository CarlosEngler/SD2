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

//-----------------ULAS----------------

module Big_ULA (
  input [25:0] A,
  input [25:0] B,
  input decisor,
  input subtrador,
  output [25:0] resultado,
  output [7:0] tamanhoShift,
  output  directionShift
);

  // se o decisor for 1 -> multiplicacao 0 -> é uma soma
  integer i;
  reg [7:0] auxiliar;
  reg [26:0] resultadoAuxiliar;
  reg boolean;

  wire [25:0] resultado26bits;
  wire [25:0] lista_carryOut;
  wire [25:0] B_final;
  reg [47:0] soma;

  assign B_final = (B != 26'd0 && subtrador == 1'b1) ? ~B : B;

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
    resultadoAuxiliar = {1'b1, resultado26bits};
    if(lista_carryOut[25] == 1'b1) resultadoAuxiliar = resultadoAuxiliar >> 1;
      for(i = 25; i >= 0; i = i - 1) begin
        if(resultadoAuxiliar[i] == 0 && boolean)
        begin
          auxiliar = auxiliar + 1;
        end
        else boolean = 1'b0;
      end
  end
  
  assign resultado = (decisor) ? resultadoAuxiliar[25:0] : soma[47:25];
  assign tamanhoShift = auxiliar;
  assign directionShift = lista_carryOut[25];

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
    .NA({18'd0, A}),
    .NB({18'd0, B_final}),
    .carryIn(subtrador),
    .soma_total(resultado23bits),
    .lista_carryOut(lista_carryOut)
  );

  assign resultado = resultado23bits[7:0]; 

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
    input [25:0] entrada,
    input [4:0] tamanho,
    input decisor,
    output [25:0] saida
);
    assign saida = (decisor) ? entrada<<tamanho : entrada>>tamanho;
                                //esquerda              //direita
endmodule

module arredondamento(
    input clk,
    input [7:0] expoente,
    input load,
    input [25:0] entrada,
    output [25:0] saida_fraction,
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

assign saida_fraction = (overflow_auxiliar) ?  entrada : {fraction[23:0], 2'b00}; 

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
    input decisor_mux_expoente_escolhido,
    input decisor_mux_saida_big_ula,
    input decisor_shift_right_left,
    input subtrador_big_ula,
    input subtrador_Somador_subtrador,
    output overflow,
    output directionShift,
    output [7:0]tamanhoShift, 
    output [7:0] saida_registrador,
    output [31:0] saida_final,
    output [25:0] data_out_big_ula
);

    wire [7:0] resultado;
    wire Bmaior;
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
    wire [7:0] saida_arredonda_expoente;
    wire [25:0] saida_arredonda_fracao;


    Small_Ula pequena(.A(input_1[30:23]), .B(input_2[30:23]), .resultado(resultado), .subtrador(soma_multiplica_small_ula), .Bmaior(Bmaior)); //retorna a diferenca de exponet no caso da soma, e soma expoente no caso da multiplicação, alem de retornar qual input tinha maior expoente
    registrador ula_pequena(.clk(clk), .entrada(resultado), .saida(saida_registrador));

    //parte da esquerda
    Mux_2_8bits Expoentes1_2(.S0(input_1[30:23]), .S1(input_2[30:23]), .S(saida_mux_expoentes), .decisor(~Bmaior)); //escolhe o menor expoente
    Mux_2_8bits ExpoenteEscolhido_final(.S0(saida_mux_expoentes), .S1(saida_arredonda_expoente), .S(saida_mux_expoentes_escolhido), .decisor(decisor_mux_expoente_escolhido)); //falta coisa
    Somador_subtrador incrementa_subtrai(.A(saida_mux_expoentes_escolhido), .subtrador(subtrador_Somador_subtrador), .n_shifts(tamanho3), .resultado(saida_subtrador_somador));

    //parte da direita
    Mux_2_23bits escolhe_shift_right(.S0({input_1[22:0], 3'd0}), .S1({input_2[22:0], 3'd0}), .S(saida_escolhe_shift_right), .decisor(~Bmaior));
    Shift_Right direita(.entrada(saida_escolhe_shift_right), .saida(saida_shift_right), .tamanho(tamanho));
    Mux_2_23bits escolhe_entrada_dois_ula(.S0({input_1[22:0], 3'd0}), .S1({input_2[22:0], 3'd0}), .S(saida_escolhe_entrada_dois_ula), .decisor(Bmaior));
    Big_ULA grande_ula(.A(saida_shift_right), .B(saida_escolhe_entrada_dois_ula), .resultado(saida_big_ula), .decisor(soma_multiplica_big_ula), .subtrador(subtrador_big_ula), .tamanhoShift(tamanhoShift), .directionShift(directionShift));

    //parte da final
    Mux_2_23bits saida_ula_grande(.S0(saida_big_ula), .S1(saida_arredonda_fracao), .S(mux_saida_big_ula), .decisor(decisor_mux_saida_big_ula)); //falta coisa
    Shift_Right_left direita_esquerda(.entrada(mux_saida_big_ula), .saida(saida_shift_right_left), .decisor(decisor_shift_right_left), .tamanho(tamanho2));
    arredondamento arredonda(.expoente(saida_subtrador_somador), .entrada(saida_shift_right_left), .saida_fraction(saida_arredonda_fracao), .expoente_saida(saida_arredonda_expoente), .clk(clk), .overflow(overflow), .load(load));


assign saida_final[31] = input_1[31];
assign saida_final[30:23] = saida_arredonda_expoente;
assign saida_final[22:0] = saida_arredonda_fracao[24:2];

assign data_out_big_ula = saida_big_ula;

endmodule


// PROBLEMAS: ajustar o signal na multiplicação,  -> fazer isso na uc, é apenas um xor

module uc_2 (
    input clk,
    input [31:0] input_1,
    input [31:0] input_2,
    output [7:0] tamanho_incrementa_decrementa;
    output [4:0] tamanho_shift_right_left, 
    output [4:0] tamanho_shift_right;
    output soma_multiplica_small_ula,
    output soma_multiplica_big_ula,
    output decisor_mux_expoente_escolhido,
    output decisor_mux_saida_big_ula,
    output decisor_shift_right_left, // descobre se da shift right ou left
    output subtrador_big_ula,
    output subtrador_incrementa_decrementa,
    output operacao,
    output load_rouding_hardware,
    output done,
    input overflow_roundign_hardware, 
    input directionShift,
    input [7:0] tamanhoShift,
    input [25:0] data_out_big_ula,
    input [7:0] saida_registrador,
    input [31:0] saida_final
    );

    reg [3:0] state;
    reg [3:0] next_state;
    reg aux, mul;

    reg write_enable, increment_mux, small_number_mux, ula_mux, enable, enable_2, operation_increment, its_over;

    reg [7:0] increment;
    reg [4:0] shift_R, shift_RL;

    parameter RESET = 3'd0, EXPONENTS_DIFFERENCE = 3'd1, MULTIPLY_OR_ADD = 3'd2, 
              NORMALIZE = 3'd3, ROUND = 3'd4, SIGN = 3'd5;

    assign decisor_mux_expoente_escolhido = increment_mux;
    assign decisor_mux_saida_big_ula =  ula_mux;

    assign round_enable = enable_2;
    assign exp_enable = enable;
    assign decisor_shift_right_left = operation_increment;
    assign tamanho_shift_right_left = shift_RL;
    assign tamanho_shift_right = shift_R;
    assign tamanho_incrementa_decrementa = increment;
    assign rst_mul = mul;
    assign done = its_over;


    always @(posedge clk or rst) begin 
        if (rst) begin
            state <= RESET;     
        end
        else begin 
            state <= next_state;
        end
    end

    always@ (state or clk or round or start)
        begin
            case (state)
                RESET:
                    begin
                        increment_mux <= 0;
                        small_number_mux <= 0;
                        ula_mux <= 0;
                        enable <= 0;
                        shift_R <= 0;
                        shift_RL <= 0;
                        operation_increment <= 0;
                        aux <= 0;
                        mul <= 0;
                        its_over <= 0;

                        next_state <= EXPONENTS_DIFFERENCE;
                    end
                EXPONENTS_DIFFERENCE:
                    begin
                        increment_mux <= 0;
                        small_number_mux <= 0;
                        ula_mux <= 0;
                        shift_R <= 0;
                        shift_RL <= 0;
                        operation_increment <= 0;
                        its_over <= 0;

                        if (start) begin
                            enable <= 1;
                            mul <= 1;
                            next_state <= MULTIPLY_OR_ADD;
                        end
                    end
                MULTIPLY_OR_ADD:
                    begin
                        if (!add_or_mul) begin
                            if (exponent_difference[7]) begin
                                small_number_mux <= 0;
                                shift_R <= exponent_difference*(-1);
                            end
                            else begin
                                small_number_mux <= 1;
                                shift_R <= exponent_difference;
                            end
                            enable <= 0;
                            mul <= 0;
                            next_state <= NORMALIZE;
                        end
                        else begin
                            enable <= 0;
                            mul <= 0;
                            if (done_mul) next_state <= NORMALIZE;
                            else next_state <= MULTIPLY_OR_ADD;
                        end
                    end
                NORMALIZE:
                    begin
                        if (add_or_mul) begin
                            if (aux) begin
                                operation_increment <= 0;
                                shift_RL <= 2'd2;
                                increment <= 1'b1;
                                aux <= 0;
                            end
                            else if (result_ula[25]) begin
                                operation_increment <= 0;
                                shift_RL <= 1'd1;
                                increment <= 1'b1;
                            end
                            else if (result_ula[24]) begin
                                operation_increment <= 1;
                                shift_RL <= 2'd2;
                                increment <= 1'b0;
                            end
                        end
                        else begin
                            if (result_ula[24] | aux) begin
                                operation_increment <= 0;
                                shift_RL <= 2'd2;
                                increment <= 1'b1;
                                aux <= 0;
                            end
                            else if (result_ula[23]) begin
                                operation_increment <= 1;
                                shift_RL <= 2'd3;
                                increment <= 1'b0;
                            end
                            else if (result_ula[22]) begin
                                operation_increment <= 1;
                                shift_RL <= 3'd4;
                                increment <= 1'b1;
                            end
                            else if (result_ula[21]) begin
                                operation_increment <= 1;
                                shift_RL <= 3'd5;
                                increment <= 2'd2;
                            end
                            else if (result_ula[20]) begin
                                operation_increment <= 1;
                                shift_RL <= 3'd6;
                                increment <= 2'd3;
                            end
                            else if (result_ula[19]) begin
                                operation_increment <= 1;
                                shift_RL <= 3'd7;
                                increment <= 3'd4;
                            end
                            else if (result_ula[18]) begin
                                operation_increment <= 1;
                                shift_RL <= 4'd8;
                                increment <= 3'd5;
                            end
                            else if (result_ula[17]) begin
                                operation_increment <= 1;
                                shift_RL <= 4'd9;
                                increment <= 3'd6;
                            end
                            else if (result_ula[16]) begin
                                operation_increment <= 1;
                                shift_RL <= 4'd10;
                                increment <= 3'd7;
                            end
                            else if (result_ula[15]) begin
                                operation_increment <= 1;
                                shift_RL <= 4'd11;
                                increment <= 4'd8;
                            end
                            else if (result_ula[14]) begin
                                operation_increment <= 1;
                                shift_RL <= 4'd12;
                                increment <= 4'd9;
                            end
                            else if (result_ula[13]) begin
                                operation_increment <= 1;
                                shift_RL <= 4'd13;
                                increment <= 4'd10;
                            end
                            else if (result_ula[12]) begin
                                operation_increment <= 1;
                                shift_RL <= 4'd14;
                                increment <= 4'd11;
                            end
                            else if (result_ula[11]) begin
                                operation_increment <= 1;
                                shift_RL <= 4'd15;
                                increment <= 4'd12;
                            end
                            else if (result_ula[10]) begin
                                operation_increment <= 1;
                                shift_RL <= 5'd16;
                                increment <= 4'd13;
                            end
                            else if (result_ula[9]) begin
                                operation_increment <= 1;
                                shift_RL <= 5'd17;
                                increment <= 4'd14;
                            end
                            else if (result_ula[8]) begin
                                operation_increment <= 1;
                                shift_RL <= 5'd18;
                                increment <= 4'd15;
                            end
                            else if (result_ula[7]) begin
                                operation_increment <= 1;
                                shift_RL <= 5'd19;
                                increment <= 5'd16;
                            end
                            else if (result_ula[6]) begin
                                operation_increment <= 1;
                                shift_RL <= 5'd20;
                                increment <= 5'd17;
                            end
                            else if (result_ula[5]) begin
                                operation_increment <= 1;
                                shift_RL <= 5'd21;
                                increment <= 5'd18;
                            end
                            else if (result_ula[4]) begin
                                operation_increment <= 1;
                                shift_RL <= 5'd22;
                                increment <= 5'd19;
                            end
                            else if (result_ula[3]) begin
                                operation_increment <= 1;
                                shift_RL <= 5'd23;
                                increment <= 5'd20;
                            end
                            else if (result_ula[2]) begin
                                operation_increment <= 1;
                                shift_RL <= 5'd24;
                                increment <= 5'd21;
                            end
                            else if (result_ula[1]) begin
                                operation_increment <= 1;
                                shift_RL <= 5'd25;
                                increment <= 5'd22;
                            end
                            else if (result_ula[0]) begin
                                operation_increment <= 1;
                                shift_RL <= 5'd26;
                                increment <= 5'd23;
                            end
                            else begin
                                operation_increment <= 1;
                                shift_RL <= 1'b0;
                                increment <= 1'b0;
                            end
                        end
                        next_state <= ROUND;
                    end
                ROUND:
                    begin
                        enable_2 <= 1;
                        if (round) begin
                            increment_mux <= 1;
                            ula_mux <= 1;
                            next_state <= NORMALIZE;
                            aux <= 1;
                        end
                        else begin
                            next_state <= EXPONENTS_DIFFERENCE;
                            its_over <= 1;
                        end
                    end
            endcase
        end 

endmodule


//corrigir datapath
// fazer a testbench
// olhar a testebench