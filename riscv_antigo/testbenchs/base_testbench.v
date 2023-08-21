
//---------testbench-datapath-------------------------
module testbench;
  reg we, clk, we_ram, we_mi;
  reg load_PC;
  reg load_IR;
  reg [63:0] PCres;
  reg [63:0] somador_PC;
  reg [63:0] imm_PC;
  reg [4:0] Rw;
  reg [4:0] Ra, Rb; //Ra, rb são utilizados pra acessar o endereço dos registradores do RF, cuja conteúdo é exposto por douta e doutb
  reg [63:0] entrada_mux_add_sub;
  reg decisor0;
  reg decisor1;
  reg decisor2;
  reg decisor3;
  reg decisor4;
  reg decisor5;
  reg decisor6;
  reg decisor_somador_subtrator;
  wire [63:0] douta_saida; //contém conteúdo do registrador acessado por Ra
  wire [63:0] data_ram; //contém conteúdo do elemento de memória acessado pela saída da ULA, que é o resultado da soma
  wire [31:0] saida_IR;
  wire [31:0] saida_MI;
  wire [31:0] saida_PC;
  wire BEQ;
  wire BNE;
  wire BGE;
  wire BLT;
  wire BGEU;
  wire BLTU;
   
  datapath dph(
  .we(we), .clk(clk), .we_ram(we_ram), .we_mi(we_mi),
  .load_PC(load_PC), .load_IR(load_IR), .PCres(PCres),
  .Rw(Rw),
  .Ra(Ra), .Rb(Rb),
  .entrada_mux_add_sub(entrada_mux_add_sub),
  .decisor0(decisor0),
  .decisor1(decisor1),
  .decisor2(decisor2),
  .decisor3(decisor3),
  .decisor4(decisor4),
  .decisor5(decisor5),
  .decisor6(decisor6),
  .somador_subtrator(decisor_somador_subtrator),
  .douta_saida(douta_saida),
  .data_ram(data_ram),
  .saida_IR(saida_IR),
  .BEQ(BEQ), .BGE(BGE), .BNE(BNE), .BLT(BLT),
  .BGEU(BGEU), .BLTU(BLTU), .saida_MI(saida_MI),
  .somador_PC(somador_PC), .saida_PC(saida_PC),
  .imm_PC(imm_PC)
);

    initial begin
      //Set valores iniciais
      clk = 0;
      we = 0;
      we_ram = 0;
      Ra = 5'd0;
      Rw = 5'd0;
      Rb = 5'd0;
      somador_PC = 64'd1;
      decisor_somador_subtrator = 0;
      entrada_mux_add_sub = 64'd0;
      decisor0 = 0;
      decisor1 = 0;
      decisor2 = 0;
      decisor3 = 0;
      decisor4 = 0;
      decisor5 = 1;
      decisor6 = 0;
     
      //--------------Exemplos de LOAD------------------------  
      //-----------load mem(X0 + 0) em X2 -> X2 <= mem(0)-------------
     
      $display("----------LOAD EXAMPLES START---------");
      #10;
      Ra = 5'd0;
      Rw = 5'd2;
     
     
      //ajuste dos MUX
      decisor0 = 1;
      decisor1 = 1;
      decisor2 = 1;
     
      #10 we = 1;
      #10 we = 0;
      Ra = Rw;  //fazendo isso para expor o conteúdo do registrador loadado em douta_saida
      #10  $display("registrador X2 = %d", douta_saida);

       //--------load (X0 + 1) em X4 -> X4 <= mem(1)--------
      #10;
      Ra = 5'd0;
      Rw = 5'd4;
      entrada_mux_add_sub = 64'd1;
     
      //ajuste dos MUX
      decisor0 = 1;
      decisor1 = 1;
      decisor2 = 1;
     
      #10 we = 1;
      #10 we = 0;
      Ra = Rw;  //fazendo isso para expor o conteúdo do registrador loadado em douta_saida
      #10  $display("registrador X4 = %d", douta_saida);
      $display("----------LOAD EXAMPLES END----------");
     
      $display("\n---------BNE START---------");
      //descobrir IR
      #10;
      load_PC = 1;
      PCres = 64'd7;
      #10;
      load_PC = 0;
     
      #10;
      load_IR = 1;
      #10;
      $display("conteúdo de IR = %b", saida_IR);
      load_IR = 0;

      //---------Leitura BNE---------------
      Ra = 5'd2;
      Rb = 5'd4;

      decisor0 = 0;
      decisor1 = 1;

      #10 $display("BEQ = %b, BNE = %b, BGE = %b, BGEU = %b , BLT = %b,  BLTU = %b", BEQ, BNE, BGE, BGEU, BLT, BLTU);
      if  (BNE == 1) begin
        somador_PC = saida_IR [31:25];
      end
      else begin
        somador_PC = 64'd1;
      end

      #10;
      decisor3 = 1;
      #10;
      load_PC = 1;
      #10;
      load_PC = 0;
      #10;
      $display("PC = PC + imediatoBNE = %b", saida_MI); //a saída esperada é a da informação de BLTU contida na memória, já que o immediate do bne é 0000011
      $display("\n---------BNE FINISH---------");


       //---------------Exemplo de ADDI e SUBI---------------
      $display("\n---------ADDI START---------");
      //descobrir IR
      decisor3 = 0;
      #10;
      load_PC = 1;
      PCres = 64'd4;
      #10;
      load_PC = 0;
     
      #10;
      load_IR = 1;
      #10;
      $display("conteúdo de IR = %b", saida_IR);
      load_IR = 0;


      //---------------addi X1 = X2 + 9 = 12 + 9-------------
      Ra = 5'd2;
      Rw = 5'd1;
      entrada_mux_add_sub = 52'd0 + saida_IR[31:20];
     
      //Ajuste dos MUX
      decisor0 = 1;
      decisor1 = 1;
      decisor2 = 0;
     
      #10 we = 1;
      #10 we = 0;
     
      Ra = 5'd1; //para expor o conteúdo de X1
      #10 $display("Conteúdo de X1 após addi (X1 <= X2 + 9) = %d", douta_saida);
      #10 $display("BEQ = %b, BNE = %b, BGE = %b, BGEU = %b , BLT = %b,  BLTU = %b", BEQ, BNE, BGE, BGEU, BLT, BLTU);
     
      $display("------------ADDI END------------\n");
     
      //decidindo ir
      $display("-----------SUBI START---------");
     
      #10;
      somador_PC = 64'd1;
      decisor3 = 1;
      #10;
      load_PC = 1;
      // PCres = 64'd5;
      #10;
      decisor3 = 0;
      load_PC = 0;
     
      #10;
      load_IR = 1;
      #10;
      $display("conteúdo de IR = %b", saida_IR);
      load_IR = 0;
     
      //-------subi X3 <= X4 - 10 = 11 - 10------
      #10;
      Ra = 5'd4;
      Rw = 5'd3;
     
      entrada_mux_add_sub = saida_IR[31:20];
     
      #10 decisor_somador_subtrator = 1;
     
      #10 //ajuste dos MUX
      decisor0 = 1;
      decisor1 = 1;
      decisor2 = 0;
     
      #10 we = 1;
      #10 we = 0;
      #10 Ra = 5'd3;
      #10 $display("valor de X3 após subi (X3 <= X4 - 10) = %d", douta_saida);
      #10 $display("BEQ = %b, BNE = %b, BGE = %b, BGEU = %b , BLT = %b,  BLTU = %b", BEQ, BNE, BGE, BGEU, BLT, BLTU);
     
      $display("------------SUBI END----------\n");
     
      //--------------Exemplo de ADD,SUB e STORE--------------
      //-------Add X6 <= X4 + X2 = 12 + 11---------------
      $display("\n---------ADD START---------");
      // ter que ler o PC, 
      Ra = 5'd2;
      Rb = 5'd4;
      Rw = 5'd6;
      decisor_somador_subtrator = 0;
     
      //ajuste dos MUX
      decisor0 = 0;
      decisor1 = 1;
      decisor2 = 0;
     
       we = 1;

       //set as variaveis
       //um clk
      #10 we = 0;
      // desnecessario
       Ra = 5'd6;
       $display("valor de X6 após add (X6 <= X4 + X2) = %d", douta_saida);
       $display("BEQ = %b, BNE = %b, BGE = %b, BGEU = %b , BLT = %b,  BLTU = %b", BEQ, BNE, BGE, BGEU, BLT, BLTU);
     
      $display("---------ADD END---------\n");
     
      //--------Sub X7 <= X6 - X3 = 23 - 1-------------
      $display("\n---------SUB START---------");
      Ra = 5'd6;
      Rb = 5'd3;
      Rw = 5'd7;
      #10 decisor_somador_subtrator = 1;
     
      //ajuste dos MUX
      decisor0 = 0;
      decisor1 = 1;
      decisor2 = 0;
     
      #10 we = 1;
      #10 we = 0;
      #10 Ra = 5'd7;
      #10 $display("valor de X7 após sub(X7 <= X6 - X3 ) = %d", douta_saida);
      #10  $display("BEQ = %b, BNE = %b, BGE = %b, BGEU = %b , BLT = %b,  BLTU = %b", BEQ, BNE, BGE, BGEU, BLT, BLTU);
      $display("---------SUB END---------\n");
     
      //----------Store de X7 into mem(2)-------------
      $display("\n---------STORE START---------");
      Ra = 5'd7;
      Rb = 5'd2;
      entrada_mux_add_sub = 0;
     
      //ajuste dos MUX
      decisor0 = 1;
      decisor1 = 0;
      decisor2 = 1;
     
      #10 we_ram = 1;
      #10 we_ram = 0;
      #10 $display("valor de mem(2) após store (mem(2) <= X7) = %d", data_ram);
      #10  $display("BEQ = %b, BNE = %b, BGE = %b, BGEU = %b , BLT = %b,  BLTU = %b", BEQ, BNE, BGE, BGEU, BLT, BLTU);
      $display("---------STORE END---------\n");
     

      //----------AUIPC START-------------
      $display("\n---------AUIPC START---------");
        decisor3 = 0;
        decisor4 = 1;
        PCres = 64'd12; // PC vale 12
        #10;
        load_PC = 1;
        #10;
        load_PC = 0;

        #10;
        load_IR = 1;
        #10;
        $display("conteúdo de IR = %b", saida_IR);
        load_IR = 0;

        imm_PC = {saida_MI[31:12], 12'd0};
        #10;
        decisor5 = 0;
        #10;
        Rw = saida_MI[11:7]; 
        #10 we = 1;
        #20 we = 0;
        Ra = Rw;
        #10;
        decisor5 = 1;
        $display("conteúdo de Ra = %d", douta_saida); 
    $display("---------AUIPC FIM---------\n");   
      //----------JAL START-------------
    $display("\n---------JAL START---------");
        //primeira parte

        decisor3 = 0;
        decisor4 = 0;
        somador_PC = 64'd4;
        PCres = 64'd13; // PC vale 13
        #10;
        load_PC = 1;
        #10;
        load_PC = 0;

        #10;
        load_IR = 1;
        #10;
        $display("conteúdo de IR = %b", saida_IR);
        load_IR = 0;

        #10;
        decisor5 = 0;
        #10;
        Rw = saida_MI[11:7]; 
        #10 we = 1;
        #20 we = 0;
        Ra = Rw;
        #10;
        decisor5 = 1;
        $display("conteúdo de Ra = %d", douta_saida);    

         $display("---------alteracao no PC---------");

        //segunda parte

        decisor3 = 1;
        decisor4 = 1;
        imm_PC = {saida_MI[31:12], 1'b0};
        #10;
        load_PC = 1;
        #10;
        load_PC = 0;
         $display("conteúdo de PC = %b", saida_PC);
         $display("---------JAL FIM---------\n");
      //----------JALR START-------------
      $display("\n---------JALR START---------");
        //primeira parte

        decisor3 = 0;
        decisor4 = 0;
        somador_PC = 64'd4;
        PCres = 64'd14; // PC vale 14
        #10;
        load_PC = 1;
        #10;
        load_PC = 0;

        #10;
        load_IR = 1;
        #10;
        $display("conteúdo de IR = %b", saida_IR);
        load_IR = 0;

        #10;
        decisor5 = 0;
        #10;
        Rw = saida_MI[11:7]; 
        #10 we = 1;
        #20 we = 0;
        Ra = Rw;
        #10;
        decisor5 = 1;
        $display("conteúdo de Ra = %d", douta_saida);

        //segunda parte

        $display("---------alteracao no PC---------");

        // rs1 = x2;
        decisor3 = 1;
        decisor4 = 1;
        Ra = saida_MI[19:15];
        #10;
        decisor6 = 1;
        #10;
        imm_PC = saida_MI[31:20];
        #10;
        load_PC = 1;
        #10;
        load_PC = 0;
         $display("conteúdo de PC = %d", saida_PC);

        $display("---------JALR FIM---------\n");



      $finish;


    end
   
    always #5 clk = ~clk;

endmodule