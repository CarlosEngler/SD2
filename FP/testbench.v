module testbench;
    reg clk;
    reg [7:0] auxiliar;
    reg [31:0] input_1;
    reg [31:0] input_2;
    reg [4:0] tamanho;
    reg [4:0] tamanho2;
    reg [7:0] tamanho3;
    reg soma_multiplica_small_ula;
    reg soma_multiplica_big_ula;
    reg decisor_mux_expoente_escolhido;
    reg decisor_mux_saida_big_ula;
    reg decisor_shift_right_left;
    reg subtrador_big_ula;
    reg subtrador_Somador_subtrador;
    reg boolean;
    reg operecao;
    wire overflow; 
    wire directionShift;
    wire [7:0] tamanhoShift;
    wire [25:0] data_out_big_ula;
    wire [7:0] saida_registrador;
    wire [31:0] saida_final;
    reg load;

    
    Datapath dp(
    .clk(clk),
    .input_1(input_1),
    .input_2(input_2),
    .tamanho(tamanho),
    .tamanho2(tamanho2),
    .tamanho3(tamanho3),
    .soma_multiplica_small_ula(soma_multiplica_small_ula),
    .soma_multiplica_big_ula(soma_multiplica_big_ula),
    .decisor_mux_expoente_escolhido(decisor_mux_expoente_escolhido),
    .decisor_mux_saida_big_ula(decisor_mux_saida_big_ula),
    .decisor_shift_right_left(decisor_shift_right_left),
    .subtrador_big_ula(subtrador_big_ula),
    .subtrador_Somador_subtrador(subtrador_Somador_subtrador), 
    .saida_registrador(saida_registrador),
    .saida_final(saida_final),
    .overflow(overflow),
    .data_out_big_ula(data_out_big_ula),
    .load(load),
    .tamanhoShift(tamanhoShift),
    .directionShift(directionShift)
    );

  integer i;

    initial begin
      $dumpfile("test.vcd");
      $dumpvars(0, testbench);
      
      clk = 1'b0;
      load = 1'b0;
      operecao = 1'b1; // soma = 1, multiplicao = 0

      if(operecao) begin //soma
        //set os inputs
        input_1 = 32'b01000001100011000000000010000000;
        input_2 = 32'b01000001000000000001010000000100;

        //inicio
        soma_multiplica_small_ula = 1'b1; // zero-multiplica, um-soma
        soma_multiplica_big_ula = 1'b1; // zero-multiplica, um-soma

        #10;
        $display("saida registrador", saida_registrador);

        //faz a equacao da big ula
          #10;
          subtrador_big_ula = 1'b0; //zero-soma, um-subtrai
          tamanho = saida_registrador;
          
          #10;
          $display("saida big ula %b", data_out_big_ula);
        //pos operecao big_ula

          //normalizacao geral
          #10;
            decisor_mux_expoente_escolhido = 1'b0;
            subtrador_Somador_subtrador = 1'b0; //0:soma , 1:subtrai
            tamanho3 = saida_registrador;
            #10;
            load = 1'b1;
            #10;
            load = 1'b0;

            #10;
            //mux da esquerda
              tamanho3 = tamanhoShift - directionShift;
              tamanho2 = tamanhoShift[4:0] - directionShift;
            
            #10;
            decisor_mux_expoente_escolhido = 1'b1;
            subtrador_Somador_subtrador = 1'b1; //0:soma , 1:subtrai

            // mux da direita
            decisor_mux_saida_big_ula = 1'b0; //0 = big ula, 1 = arredondamento
            decisor_shift_right_left = directionShift;

            #10;
            load = 1'b1;
            #10;
            load = 1'b0;
            
            #10; // gambiarra monstruosa, dando shift right, muitos casos deram certo
            subtrador_Somador_subtrador = 1'b0; //0:soma , 1:subtrai
            tamanho3 = 8'd1;
            tamanho2 = 5'd1;
            decisor_shift_right_left = 1'b0;    // 1 = shift left, 0 = shift right
            
            #10;
            load = 1'b1;
            #10;
            load = 1'b0;


            //caso overflow no rounding -> joga o número original pro shift_right_left e da um shift right, consequentemente soma 1 no expoente
            if(overflow)
            begin
            decisor_mux_saida_big_ula = 1'b1; //shift right left recebe fração do arredonda
            decisor_shift_right_left = 1'b0; //shift right
            tamanho2 = 5'd1;

            decisor_mux_expoente_escolhido = 1'b1; //inc dec recebe expoente arredondado
            subtrador_Somador_subtrador = 1'b0; //0:soma , 1:subtrai
            tamanho3 = 8'd1; //soma 1 no expoente

            #10;
            load = 1'b1;
            #10;
            load = 1'b0;
            end
        end

        else begin //multiplicao

          //set os inputs
          input_1 = 32'b01000001100011000000000010000000;
          input_2 = 32'b01000001000000000001010000000100;
          
          //inicio
          soma_multiplica_small_ula = 1'b0; // zero-multiplica, um-soma
          soma_multiplica_big_ula = 1'b0; // zero-multiplica, um-soma

          #10;
        $display("saida registrador", saida_registrador);

        //faz a equacao da big ula
          #10;
          tamanho = saida_registrador;
          
          #10;
          $display("saida big ula %b", data_out_big_ula);

        end


        #10;
        $display("sinal arredondado %b", saida_final);



      #30;
      $finish;
    end

    always #5 clk= ~clk;
endmodule

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