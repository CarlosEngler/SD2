
//---------------------------RF_components---------------------

module RF(we, clk, Rw, Ra, Rb, din, douta, doutb);
    input [4:0] Rw, Ra, Rb; 
    input we, clk; //we -> decide se vai ser registrado depois lido, ou so lido
    input [63:0] din;
    output [63:0] douta, doutb;
  
    wire [63:0] reg_outputs [0:31];

    // banco de registradores
    registrador X0(.clk(clk), .load((we && Rw == 0)), .entrada(din), .saida(reg_outputs[0]));
    registrador X1(.clk(clk), .load((we && Rw == 1)), .entrada(din), .saida(reg_outputs[1]));
    registrador X2(.clk(clk), .load((we && Rw == 2)), .entrada(din), .saida(reg_outputs[2]));
    registrador X3(.clk(clk), .load((we && Rw == 3)), .entrada(din), .saida(reg_outputs[3]));
    registrador X4(.clk(clk), .load((we && Rw == 4)), .entrada(din), .saida(reg_outputs[4]));
    registrador X5(.clk(clk), .load((we && Rw == 5)), .entrada(din), .saida(reg_outputs[5]));
    registrador X6(.clk(clk), .load((we && Rw == 6)), .entrada(din), .saida(reg_outputs[6]));
    registrador X7(.clk(clk), .load((we && Rw == 7)), .entrada(din), .saida(reg_outputs[7]));
    registrador X8(.clk(clk), .load((we && Rw == 8)), .entrada(din), .saida(reg_outputs[8]));
    registrador X9(.clk(clk), .load((we && Rw == 9)), .entrada(din), .saida(reg_outputs[9]));
    registrador X10(.clk(clk), .load((we && Rw == 10)), .entrada(din), .saida(reg_outputs[10]));
    registrador X11(.clk(clk), .load((we && Rw == 11)), .entrada(din), .saida(reg_outputs[11]));
    registrador X12(.clk(clk), .load((we && Rw == 12)), .entrada(din), .saida(reg_outputs[12]));
    registrador X13(.clk(clk), .load((we && Rw == 13)), .entrada(din), .saida(reg_outputs[13]));
    registrador X14(.clk(clk), .load((we && Rw == 14)), .entrada(din), .saida(reg_outputs[14]));
    registrador X15(.clk(clk), .load((we && Rw == 15)), .entrada(din), .saida(reg_outputs[15]));
    registrador X16(.clk(clk), .load((we && Rw == 16)), .entrada(din), .saida(reg_outputs[17]));
    registrador X17(.clk(clk), .load((we && Rw == 17)), .entrada(din), .saida(reg_outputs[17]));
    registrador X18(.clk(clk), .load((we && Rw == 18)), .entrada(din), .saida(reg_outputs[18]));
    registrador X19(.clk(clk), .load((we && Rw == 19)), .entrada(din), .saida(reg_outputs[19]));
    registrador X20(.clk(clk), .load((we && Rw == 20)), .entrada(din), .saida(reg_outputs[20]));
    registrador X21(.clk(clk), .load((we && Rw == 21)), .entrada(din), .saida(reg_outputs[21]));
    registrador X22(.clk(clk), .load((we && Rw == 22)), .entrada(din), .saida(reg_outputs[22]));
    registrador X23(.clk(clk), .load((we && Rw == 23)), .entrada(din), .saida(reg_outputs[23]));
    registrador X24(.clk(clk), .load((we && Rw == 24)), .entrada(din), .saida(reg_outputs[24]));
    registrador X25(.clk(clk), .load((we && Rw == 25)), .entrada(din), .saida(reg_outputs[25]));
    registrador X26(.clk(clk), .load((we && Rw == 26)), .entrada(din), .saida(reg_outputs[26]));
    registrador X27(.clk(clk), .load((we && Rw == 27)), .entrada(din), .saida(reg_outputs[27]));
    registrador X28(.clk(clk), .load((we && Rw == 28)), .entrada(din), .saida(reg_outputs[28]));
    registrador X29(.clk(clk), .load((we && Rw == 29)), .entrada(din), .saida(reg_outputs[29]));
    registrador X30(.clk(clk), .load((we && Rw == 30)), .entrada(din), .saida(reg_outputs[30]));
    registrador X31(.clk(clk), .load((we && Rw == 31)), .entrada(din), .saida(reg_outputs[31]));

    //retorna o valor referente ao endereco Ra e Rb
    assign douta = (Ra == 0) ? 64'd0 : reg_outputs[Ra];
    assign doutb = (Rb == 0) ? 64'd0 : reg_outputs[Rb];

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
      mem[0] <= 32'b00000000000000000011000000000011; //l-format Load: 
      mem[1] <= 32'b00000000000000000011000000100011; //s-format Store:
      mem[2] <= 32'b00000000010000010000001100110011; //r-format Add: X6 <= X2 + X4
      mem[3] <= 32'b01000000001100110000001110110011; //r-format Sub: X7 <= X6 - X3
      mem[4] <= 32'b00000000100100010000000010010011; //l-format Addi: X1 <= X2 + 9
      mem[5] <= 32'b00000000101000011000001010010011; //l-format Subi: X5 <= X3 - 10 nota: nossa ula inverte o número passado pra ela dentro de seu próprio funcionamento, portanto passamos o valor do immediate em módulo.
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
  	output reg [63:0] saida
);
  	// na primeria interacao retorna 64'bX, pois nada foi setado, ate aquele momento

  	always @(posedge clk) begin
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
  output overflow
);
wire [`NBITS-1:0] B_final;
wire [`NBITS-1:0] lista_carryOut;

assign B_final = (B != `NBITS'b0 && input_subtrator == 1) ? ~B : B;


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
  input [63:0] PCres,
  input [4:0] Rw,
  input [4:0] Ra, Rb,
  input [63:0] entrada_mux_add_sub,
  input decisor0,
  input decisor1,
  input decisor2,
  input somador_subtrator,
  output [63:0] douta_saida,
  output [63:0] data_ram,
  output [31:0] saida_IR
);

  wire [63:0] din;
  wire [63:0] dout_ram;
  wire [31:0] fio_MI;
  wire [63:0] enderecoIM;

  wire [63:0] douta;
  wire [63:0] doutb;

  wire [63:0] entrada_ula_load_store;
  wire [63:0] entrada_ula_add;

  wire [63:0] saida_ula;

  wire overflow;

  registrador PC(.load(load_PC), .clk(clk), .entrada(PCres), .saida(enderecoIM));

  InstructionMemory MI(.we(we_mi), .entrada(32'd0), .address(enderecoIM), .saida(fio_MI), .clk(clk));

  registrador32b IR(.load(load_IR), .clk(clk), .entrada(fio_MI), .saida(saida_IR));
  
  
  RF rf( .we(we), .Rw(Rw), .Ra(Ra), .Rb(Rb), .din(din), .douta(douta), .doutb(doutb), .clk(clk) ); //instancia o RF

  Mux_2 mux_add_sub(.S0(doutb), .S1(entrada_mux_add_sub), .decisor(decisor0), .S(entrada_ula_add)); // decisor = 1 devolve entrada da uc, enquanto decisor = 0 devolve doutB

  Mux_2 mux_load_store(.S0(doutb), .S1(douta), .decisor(decisor1), .S(entrada_ula_load_store)); // decisor = 1 devolve douta, enquanto decisor = 0 devolve doutb

  ULA soma(.A(entrada_ula_load_store), .B(entrada_ula_add), .input_subtrator(somador_subtrator), .resultado(saida_ula), .overflow(overflow)); //instancia a ULA

  ram RAM(.we(we_ram), .entrada(douta), .address(saida_ula), .saida(dout_ram), .clk(clk)); //instancia a memória

  Mux_2 mux_din(.S0(saida_ula), .S1(dout_ram), .decisor(decisor2), .S(din));

  assign douta_saida = douta;
  assign data_ram = dout_ram;

endmodule

// Resumo das operações:
// Load: mux_add_sub==mux_load_store==mux_din==1: Ra contém endereço do registrador (conteúdo expicitado em dout) que contém o endereço desejado da RAM. DoutA entra na ula e é somado com o valor definido pela UC, para definir qual endereço da ram será acessado. O resultado da ULA é passado como o input address da RAM, que é o endereço do elemento de memória que vai ser acessado. O conteúdo do endereço é selecionado pelo mux_din e é passado ao registrador do RF definido por Rw.  (o que é feito utilizando Ra podia ser feito utilizando Rb, apenas fazer mux_load_store=0);
//Store: 


//---------testbench-datapath-------------------------



module testbench;
  reg we, clk, we_ram, we_mi;
  reg load_PC;
  reg load_IR;
  reg [63:0] PCres;
  reg [4:0] Rw;
  reg [4:0] Ra, Rb; //Ra, rb são utilizados pra acessar o endereço dos registradores do RF, cuja conteúdo é exposto por douta e doutb
  reg [63:0] entrada_mux_add_sub;
  reg decisor0;
  reg decisor1;
  reg decisor2;
  reg decisor_somador_subtrator;
  wire [63:0] douta_saida; //contém conteúdo do registrador acessado por Ra
  wire [63:0] data_ram; //contém conteúdo do elemento de memória acessado pela saída da ULA, que é o resultado da soma
  wire [31:0] saida_IR;
    
  datapath dph(
  .we(we), .clk(clk), .we_ram(we_ram), .we_mi(we_mi),
  .load_PC(load_PC), .load_IR(load_IR), .PCres(PCres),
  .Rw(Rw),
  .Ra(Ra), .Rb(Rb),
  .entrada_mux_add_sub(entrada_mux_add_sub),
  .decisor0(decisor0),
  .decisor1(decisor1),
  .decisor2(decisor2),
  .somador_subtrator(decisor_somador_subtrator),
  .douta_saida(douta_saida),
  .data_ram(data_ram),
  .saida_IR(saida_IR)
);

    initial begin
      //Set valores iniciais
      clk = 0;
      we = 0;
      we_ram = 0;
      Ra = 5'd0;
      Rw = 5'd0;
      Rb = 5'd0;
      decisor_somador_subtrator = 0;
      entrada_mux_add_sub = 64'd0;
      decisor0 = 0;
      decisor1 = 0;
      decisor2 = 0;
      
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
      
      //---------------Exemplo de ADDI e SUBI---------------
      $display("\n---------ADDI START---------");
      
      //descobrir IR
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
      
      $display("------------ADDI END------------\n");
      
      
      
      //decidindo ir
      $display("-----------SUBI START---------");
      
      #10;
      load_PC = 1;
      PCres = 64'd5;
      #10;
      load_PC = 0;
      
      #10;
      load_IR = 1;
      #10;
      load_IR = 0;
      
      //-------subi X3 <= X4 - 10 = 11 - 10------
      #10;
      Ra = 5'd4;
      Rw = 5'd3;
      
      entrada_mux_add_sub = saida_IR[31:20];
      $display("valor do immediate = %d", entrada_mux_add_sub);
      
      #10 decisor_somador_subtrator = 1;
      
      #10 //ajuste dos MUX
      decisor0 = 1;
      decisor1 = 1;
      decisor2 = 0;
      
      #10 we = 1;
      #10 we = 0;
      #10 Ra = 5'd3;
      #10 $display("valor de X3 após subi (X3 <= X4 - 10) = %d", douta_saida);
      
      $display("------------SUBI END----------\n");
      
      //--------------Exemplo de ADD,SUB e STORE--------------
      //-------Add X6 <= X4 + X2 = 12 + 11---------------
      $display("\n---------ADD START---------");
      Ra = 5'd2;
      Rb = 5'd4;
      Rw = 5'd6;
      decisor_somador_subtrator = 0;
      
      //ajuste dos MUX
      decisor0 = 0;
      decisor1 = 1;
      decisor2 = 0;
      
      #10 we = 1;
      #10 we = 0;
      #10 Ra = 5'd6;
      #10 $display("valor de X6 após add (X6 <= X4 + X2) = %d", douta_saida);
      
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
      $display("---------STORE END---------\n");
      
      $finish;
    end
    
    always #5 clk = ~clk;

endmodule


