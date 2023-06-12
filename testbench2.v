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
  wire [31:0] saida_PC;
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
    #50;
    #10; $display("valor de X6 após add (X6 <= X4 + X2) = %d", douta_saida);

    $stop;
  
end


always #5 clk = ~clk;

endmodule