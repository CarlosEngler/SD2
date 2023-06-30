
module fpu (clk, rst, A, B, R, op, start, done);
    input  clk, rst;   // reset assíncrono
    input  [31:0] A,B; // Entradas
    output [31:0] R;   // Saída
    input  [1:0] op;   // 00:soma, 01:subtração, 10:multiplicação, 11:divisão
    input  start;      // 1: começa o cálculo, done vai pra zero
    output done;       // 1: cálculo terminado, fica em 1 até start ir de zero para um


    wire [7:0] tamanho_incrementa_decrementa, saida_expoente_diferenca, tamanhoShift;
    wire [7:0] tamanho_shift_right, tamanho_shift_right_left;
    wire [31:0] saida_final;
    wire [27:0] data_out_big_ula;
    wire enable, sinal;
    wire [1:0] decisor_mux_expoente_escolhido;
    wire decisor_mux_saida_big_ula, decisor_shift_right_left;
    wire overflow;
    wire soma_multiplica_big_ula, soma_multiplica_small_ula;
    wire subtrador_big_ula, subtrador_incrementa_decrementa;


Datapath dp(
     .clk(clk),
     .load(enable),
     .input_1(A),
     .input_2(B),
     .tamanho(tamanho_shift_right),
     .tamanho2(tamanho_shift_right_left),
     .tamanho3(tamanho_incrementa_decrementa),
     .soma_multiplica_small_ula(soma_multiplica_small_ula),
     .soma_multiplica_big_ula(soma_multiplica_big_ula),
     .decisor_mux_expoente_escolhido(decisor_mux_expoente_escolhido),
     .decisor_mux_saida_big_ula(decisor_mux_saida_big_ula),
     .decisor_shift_right_left(decisor_shift_right_left),
     .subtrador_big_ula(subtrador_big_ula),
     .subtrador_Somador_subtrador(subtrador_incrementa_decrementa),
     .overflow(overflow),
     .tamanhoShift(tamanhoShift), 
     .saida_registrador(saida_expoente_diferenca),
     .saida_final(saida_final),
     .data_out_big_ula(data_out_big_ula)
);


    uc_2 uc(
    .clk(clk),
    .rst(rst),
    .start(start),
    .input_1(A),
    .input_2(B),
    .add_or_mul(op[1]),
    .tamanho_incrementa_decrementa(tamanho_incrementa_decrementa),
    .tamanho_shift_right_left(tamanho_shift_right_left), 
    .tamanho_shift_right(tamanho_shift_right),
    .soma_multiplica_small_ula(soma_multiplica_small_ula),
    .soma_multiplica_big_ula(soma_multiplica_big_ula),
    .decisor_mux_expoente_escolhido(decisor_mux_expoente_escolhido),
    .decisor_mux_saida_big_ula(decisor_mux_saida_big_ula),
    .decisor_shift_right_left(decisor_shift_right_left),
    .subtrador_big_ula(subtrador_big_ula),
    .subtrador_incrementa_decrementa(subtrador_incrementa_decrementa),
    .load_rouding_hardware(enable),
    .done(done),
    .sinal(sinal),
    .overflow_roundign_hardware(overflow),
    .tamanhoShift(tamanhoShift),
    .data_out_big_ula(data_out_big_ula),
    .saida_registrador(saida_expoente_diferenca),
    );

    assign R = (rst) ? 32'd0 : {sinal, saida_final[30:0]};

endmodule
