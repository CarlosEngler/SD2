//--------------ULA_components--------

  //-----------------Somador-componentes---------------
`define NBITS 64

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

`define NBITS 64

module full_adder ( // somador de nbits
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
  // NBITS = 64
  input [`NBITS-1:0] A,
  input [`NBITS-1:0] B,
  input [3:0] lists,
  output [`NBITS-1:0] resultado,
  output [3:0] alu_flags
);

wire [`NBITS-1:0] B_final;
wire [`NBITS-1:0] resultadoR; //utilizado para instrucoes do tipo R/U/UJ
wire subtrador;
wire Zero; // vale 1 se resultado e igual a zero
wire [`NBITS-1:0] lista_carryOut;

assign subtrador = (lists == 4'd1 && ) ? 1 : 0;
assign Zero = (A == B) ? 1 : 0;
assign B_final = (B != `NBITS'b0 && subtrador == 1) ? ~B : B; // inverte bits para a subtracao

// somador da ULA
full_adder UUT(
    .NA(A),
    .NB(B_final),
    .carryIn(subtrador),
    .soma_total(resultadoR),
    .lista_carryOut(lista_carryOut)
  );
//--------------------------------

assign resultado = (lists == 4'd2) ? (A & B) : (lists == 4'd3) ? (A | B) : resultadoR;
//falta o U e o UJ, mas eles recebem outras entradas

assign alu_flags = {1'b0, lista_carryOut[`NBITS-1],  resultado[63], Zero}; //descobre as flags
                    //0   //overflow                 //MSB          //Zero

endmodule


module testbench_ula;
  reg [`NBITS-1:0] A,
  reg [`NBITS-1:0] B,
  reg [3:0] lists,
  wire [`NBITS-1:0] resultado,
  wire [3:0] alu_flags

 ULA ULA (
  // NBITS = 64
  .A(A),
  .B(B),
  .lists(lists),
  .resultado(resultado),
  .alu_flags(alu_flags)
);

initial begin
  A = 64'd3;
  B = 64'd2;
  lists = 4'd0;
  $display("%d %b", resultado, alu_flags);
  #10;
  A = 64'd3;
  B = 64'd2;
  lists = 4'd1;
  $display("%d %b", resultado, alu_flags);
  #10;
  A = 64'd3;
  B = 64'd2;
  lists = 4'd2;
  $display("%d %b", resultado, alu_flags);
  #10;
  A = 64'd3;
  B = 64'd2;
  lists = 4'd3;
  $display("%d %b", resultado, alu_flags);
  #10;
end

endmodule