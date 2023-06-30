
module fpu (clk, rst, A, B, R, op, start, done);
    input  clk, rst;   // reset assíncrono
    input  [31:0] A,B; // Entradas
    output [31:0] R;   // Saída
    input  [1:0] op;   // 00:soma, 01:subtração, 10:multiplicação, 11:divisão
    input  start;      // 1: começa o cálculo, done vai pra zero
    output done;       // 1: cálculo terminado, fica em 1 até start ir de zero para um


Datapath dp(
     .clk(),
     .load(),
     .input_1(),
     .input_2(),
     .tamanho(),
     .tamanho2(),
     .tamanho3(),
     .soma_multiplica_small_ula(),
     .soma_multiplica_big_ula(),
     .decisor_mux_expoente_escolhido(),
     .decisor_mux_saida_big_ula(),
     .decisor_shift_right_left(),
     .subtrador_big_ula(),
     .subtrador_Somador_subtrador(),
     .overflow(),
     .directionShift(),
     .tamanhoShift(), 
     .saida_registrador(),
     .saida_final(),
     .data_out_big_ula()
);


    uc_2






endmodule
