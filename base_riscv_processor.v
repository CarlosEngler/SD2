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
  input [31:0] entrada,
  input [63:0] address,
  input we, clk
);

  reg [31:0] mem [31:0];    
   
    always @(posedge clk) begin
      mem[0] <= 32'b00000000000000000 011 00001 0000011; //l-format Load:
      mem[1] <= 32'b00000000000000000 011 00010 0000011; //load X2<=mem(1 + 0)
      //mem[1] <= 32'b00000000000000000011000000100011; //s-format Store:
      //mem[2] <= 32'b00000000010000010000001100110011;
        mem[3] <= 32'b00000000000100010000001100110011;   //r-format Add: X6 <= X2 + X4
      //mem[3] <= 32'b01000000001100110000001110110011; //r-format Sub: X7 <= X6 - X3
      //mem[4] <= 32'b00000000100100010000000010010011; //l-format Addi: X1 <= X2 + 9
      //mem[5] <= 32'b00000000101000011000001010010011; //l-format Subi: X5 <= X3 - 10 nota: nossa ula inverte o número passado pra ela dentro de seu próprio funcionamento, portanto passamos o valor do immediate em módulo.
      // Branch functions (SB-format)
      //mem[6] <= 32'b00000110000100000100100001100111; //beq
      //mem[7] <= 32'b00000110000100000100100001100111; //bne
      //mem[8] <= 32'b00000110000100000110000001100111; //blt
      //mem[9] <= 32'b00000110000100000101000001100111; //bge
      //mem[10] <=32'b00000110000100000111000001100111; //bltu
      //mem[11] <=32'b00000110000100000111100001100111; //bgeu
     
     //AUIPC, JAL, JALR

      //mem[12] <=32'b00000000000000000010000110110111; //AUIPC 
      //mem[13] <=32'b00000000000000000001000111101111; //JAL 
      //mem[14] <=32'b00000000000100010000011001100111; // JALR 

      if (we) begin
        mem[address] = entrada;
      end

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
        if (load) begin
      saida <= entrada;
    end
  end

endmodule

//----------------Registrador_de_32bits---------------
module registrador32b(
  input clk,
  input load,
  input [31:0] entrada,
  output reg [31:0] saida 
);
  // na primeria interacao retorna 64'bX, pois nada foi setado, ate aquele momento

  always @(posedge clk) begin
      if (load) begin
      saida <= entrada;
    end
  end

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
  input input_subtrator,
  output [`NBITS-1:0] resultado,
  output BEQ,
  output BNE,
  output BGE,
  output BGEU,
  output BLT,
  output BLTU,
  output overflow
);
wire [`NBITS-1:0] B_final;
wire [`NBITS-1:0] lista_carryOut;

assign B_final = (B != `NBITS'b0 && input_subtrator == 1) ? ~B : B;

assign BEQ = (A == B) ? 1 : 0;
assign BNE = ~BEQ;

assign BLT = (A < B_final) ? 1 : 0;
assign BGE = ~BLT;

assign BLTU = (A < B) ? 1 : 0;
assign BGEU = ~BLTU;

full_adder UUT(
    .NA(A),
    .NB(B_final),
    .carryIn((B != `NBITS'b0) ? input_subtrator : 1'b0),
    .soma_total(resultado),
    .lista_carryOut(lista_carryOut)
  );
 
assign overflow = lista_carryOut[`NBITS-1];

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

//------------datapath---------

module datapath(
  input we, clk, we_ram, we_mi,
  input load_PC,
  input load_IR,
  input reset_PC,
  input [63:0] PCres,
  input [63:0] somador_PC,
  input [63:0] imm_PC,
  input [4:0] Rw,
  input [4:0] Ra, Rb,
  input [63:0] entrada_mux_add_sub,
  input decisor0,
  input decisor1,
  input decisor2,
  input decisor3,
  input decisor4,
  input decisor5,
  input decisor6,
  input somador_subtrator,
  output BEQ,
  output BNE,
  output BGE,
  output BLT,
  output BGEU,
  output BLTU,
  output [63:0] douta_saida,
  output [63:0] data_ram,
  output [31:0] saida_IR,
  output [63:0] saida_PC,
  output [31:0] saida_MI
);

  wire [63:0] din;
  wire [63:0] funcoesSequnciais;
  wire [63:0] dout_ram;
  wire [31:0] fio_MI;
  wire [63:0] enderecoIM;

  wire [63:0] UC_PC;
  wire [63:0] IMM_PC;

  wire [63:0] douta;
  wire [63:0] doutb;

  wire [63:0] entrada_ula_load_store;
  wire [63:0] endereco_MUX_PC;
  wire [63:0] entrada_ula_add;
  wire [63:0] entrada_pc;
  wire [63:0] somador_imm_PC;

  wire [63:0] saida_ula;

  wire overflow;

  //parte PC

  Mux_2 MUX_PC(.S0(PCres), .S1(endereco_MUX_PC), .decisor(decisor3), .S(entrada_pc));

  Mux_2 SOMA_PC(.S0(UC_PC), .S1(IMM_PC), .decisor(decisor4), .S(endereco_MUX_PC));

  Mux_2 JARL(.S0(enderecoIM), .S1(douta), .decisor(decisor6), .S(somador_imm_PC));

  full_adder somadorUC(.NA(somador_PC), .NB(enderecoIM), .carryIn(1'd0), .soma_total(UC_PC), .lista_carryOut());

  full_adder somadorPC(.NA(imm_PC), .NB(somador_imm_PC), .carryIn(1'd0), .soma_total(IMM_PC), .lista_carryOut());

  registrador PC(.load(load_PC), .clk(clk), .entrada(entrada_pc), .reset(reset_PC), .saida(enderecoIM));
 
  assign saida_PC = enderecoIM;

  InstructionMemory MI(.we(we_mi), .entrada(32'd0), .address(enderecoIM), .saida(fio_MI), .clk(clk));

  assign saida_MI = fio_MI;

  //parte de baixo

  registrador32b IR(.load(load_IR), .clk(clk), .entrada(fio_MI), .saida(saida_IR));
 
  RF rf( .we(we), .Rw(Rw), .Ra(Ra), .Rb(Rb), .din(din), .douta(douta), .doutb(doutb), .clk(clk) ); //instancia o RF

  Mux_2 mux_add_sub(.S0(doutb), .S1(entrada_mux_add_sub), .decisor(decisor0), .S(entrada_ula_add)); // decisor = 1 devolve entrada da uc, enquanto decisor = 0 devolve doutB

  Mux_2 mux_load_store(.S0(doutb), .S1(douta), .decisor(decisor1), .S(entrada_ula_load_store)); // decisor = 1 devolve douta, enquanto decisor = 0 devolve doutb

  ULA soma(.A(entrada_ula_load_store), .B(entrada_ula_add), .input_subtrator(somador_subtrator), .resultado(saida_ula), .overflow(overflow), .BEQ(BEQ), .BNE(BNE), .BGE(BGE), .BLT(BLT), .BGEU(BGEU), .BLTU(BLTU)); //instancia a ULA
  ram RAM(.we(we_ram), .entrada(douta), .address(saida_ula), .saida(dout_ram), .clk(clk)); //instancia a memória

  Mux_2 mux_din(.S0(saida_ula), .S1(dout_ram), .decisor(decisor2), .S(funcoesSequnciais));

  Mux_2 Mux_rf(.S0(endereco_MUX_PC), .S1(funcoesSequnciais), .decisor(decisor5), .S(din));

  assign douta_saida = douta;
  assign data_ram = dout_ram;

endmodule

// Resumo das operações:
// Load: mux_add_sub==mux_load_store==mux_din==1: Ra contém endereço do registrador (conteúdo expicitado em dout) que contém o endereço desejado da RAM. DoutA entra na ula e é somado com o valor definido pela UC, para definir qual endereço da ram será acessado. O resultado da ULA é passado como o input address da RAM, que é o endereço do elemento de memória que vai ser acessado. O conteúdo do endereço é selecionado pelo mux_din e é passado ao registrador do RF definido por Rw.  (o que é feito utilizando Ra podia ser feito utilizando Rb, apenas fazer mux_load_store=0);
//Store:


module uc(
  input clk,
  input BEQ,
  input BNE,
  input BGE,
  input BLT,
  input BGEU,
  input BLTU,
  input [31:0] saida_IR,
  input [31:0] saida_MI,

  output reg decisor0,
  output reg reset_PC,
  output reg decisor1,
  output reg decisor2,
  output reg decisor3,
  output reg decisor4,
  output reg decisor5,
  output reg decisor6,
  output reg we, we_ram, we_mi, we_pc, we_ir,
  output reg somador_subtrator,
  output reg [63:0] PCres,
  output reg [63:0] somador_PC,
  output reg [63:0] imm_PC,
  output reg [4:0] Rw,
  output reg [4:0] Ra, Rb,
  output reg [63:0] entrada_mux_add_sub );

  reg [1:0] sel;
  reg [2:0] selEx;

  initial begin
    we_pc = 1;
    PCres = 64'd0;
    decisor3 = 0;
    decisor4 = 0;
    sel = 2'b00;
    reset_PC = 1'b1;
    we = 0; 
    we_ir = 0;
    we_pc = 0;

    decisor3 <= 1;
    
   
  end
  

  always @(posedge clk)
  begin
    
    case (sel) 

      2'b00: //fetch
      begin
        reset_PC = 1'b0;
        decisor3 = 1'b1;
        decisor4 = 1'b0;
        somador_PC = 64'd1;
        we_pc = 1;
        we_ir = 1;
        sel <= 2'b01;
      end

      2'b01: //decode
      begin
        we_pc = 0;
        we_ir = 0;
        if(saida_MI[6:0] == 7'b0110011) // add/sub
        begin
          sel <= 2'b10; 
          selEx <= 3'b000;
        end

        if(saida_MI[6:0] == 7'b0000011) //load
        begin
          sel<= 2'b10;
          selEx <= 3'b001;
        end

      end

      2'b10: //ex
      begin
        case (selEx)
          3'b000: //0110011 -> ADD ou SUB
          begin
            somador_subtrator = 0;
            we = 1;
            
            //ajuste dos MUX
            decisor0 = 0;
            decisor1 = 1;
            decisor2 = 0;
            decisor5 = 1;

            if(saida_MI[31:25] == 7'b0100000)
            begin
              somador_subtrator = 1;
            end

            Ra = saida_MI[24:20];
            Rb = saida_MI[19:15];
            Rw = saida_MI[11:7]; //salva o resultado da operação no endereço Rw
    
            sel <= 2'b11;
            
          end

         3'b001: //load
          begin
           
           decisor0 = 1;
           decisor1 = 1;
           decisor2 = 1;
           decisor5 = 1;

           Ra = saida_MI[11:7]; //setta o endereço da ram que será loadado        
           Rw = saida_MI[19:15]; //setta o endereço do rf que será loada
           entrada_mux_add_sub = saida_MI[31:20]; //imeddiate
          
           we = 1; 
           sel <= 2'b11;
           end

          3'b011: 
          begin

            sel = 2'b11;
          end

          3'b100: 
          begin
            sel = 2'b11;
          end

          3'b101: 
          begin
            sel = 2'b11;
          end
          
          default: sel <= 2'b11;
          
        endcase
      end

      2'b11: //write back
      begin
        we = 0;
        Ra = Rw;
        we_ram = 0;
        //sel = 2'b00;
      end

    endcase


  end

endmodule

//---------testbench-datapath-------------------------

module testbench2_tb();
  reg clk;
  wire we, we_ram, we_mi;
  wire load_PC;
  wire load_IR;
  wire [63:0] PCres;
  wire [63:0] somador_PC;
  wire [63:0] imm_PC;
  wire [4:0] Rw;
  wire [4:0] Ra, Rb;
  wire [63:0] entrada_mux_add_sub;
  wire decisor0;
  wire decisor1;
  wire decisor2;
  wire decisor3;
  wire decisor4;
  wire decisor5;
  wire decisor6;
  wire decisor_somador_subtrator;
  wire [63:0] douta_saida; //contém conteúdo do registrador acessado por Ra
  wire [63:0] data_ram; //contém conteúdo do elemento de memória acessado pela saída da ULA, que é o resultado da soma
  wire [31:0] saida_IR;
  wire [31:0] saida_MI;
  wire [63:0] saida_PC;
  wire reset_PC;
  wire BEQ;
  wire BNE;
  wire BGE;
  wire BLT;
  wire BGEU;
  wire BLTU;
   

  uc auc(
  .clk(clk),
  .BEQ(BEQ),
  .BNE(BNE),
  .BGE(BGE),
  .BLT(BLT),
  .BGEU(BGEU),
  .BLTU(BLTU),
  .saida_IR(saida_IR),
  .saida_MI(saida_MI),

  .decisor0(decisor0),
  .decisor1(decisor1),
  .decisor2(decisor2),
  .decisor3(decisor3),
  .decisor4(decisor4),
  .decisor5(decisor5),
  .decisor6(decisor6),
  .reset_PC(reset_PC),
  .we(we), .we_ram(we_ram), .we_mi(we_mi), .we_pc(load_PC), .we_ir(load_IR),
  .somador_subtrator(decisor_somador_subtrator),
  .PCres(PCres),
  .somador_PC(somador_PC),
  .imm_PC(imm_PC),
  .Rw(Rw),
  .Ra(Ra), .Rb(Rb),
  .entrada_mux_add_sub(entrada_mux_add_sub) );
  
  datapath dph(
  .we(we), .clk(clk), .we_ram(we_ram), .we_mi(we_mi),
  .load_PC(load_PC), .load_IR(load_IR), .PCres(PCres),
  .Rw(Rw), .Ra(Ra), .Rb(Rb),
  .reset_PC(reset_PC),
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
  $dumpfile("test.vcd");
    $dumpvars(0);
 //32'b0000000 00100 00010 000 00110 0110011; //r-format Add: X6 <= X2 + X4 mem(0)
    clk = 1'b0;
    
    #10 $display("valor de X6 após add (X6 <= X4 + X2) = %d", douta_saida);

    $stop;
  
end


always #5 clk = ~clk;

endmodule