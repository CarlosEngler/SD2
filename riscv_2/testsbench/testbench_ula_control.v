  module Ula_control (
    input [3:0] alu_cmd,
    input [31:0] instruction,
    output reg [3:0] lists
  );
    assign lists = (alu_cmd = 4'b0000) ? ((instruction[14:12] == 3'b000) ? ((instruction[31:25] == 7'd0) ? 4'd0 : 4'd1) : ((instruction[14:12] == 3'b111) ? 4'd2 : ((instruction[14:12] == 3'b110) ? 4'd3 : 4'dx))) : // and e or
                   (alu_cmd = 4'b0001) ? 4'd0 : ((alu_cmd == 4'b0010) ? 4'd0 : ((alu_cmd == 4'b0011) ? 4'd2 : 4'd0));                      

    // 4'd0 = add; 4'd1 = sub; 4'd2 = and; 4'd3 = or;

  endmodule

module Ula_control (
    input [3:0] alu_cmd,
    input [31:0] instruction,
    output reg [3:0] lists
  );
                   
    always @* begin
    case(alu_cmd)
        4'b000: begin
            if(instruction[14:12] == 3'b000) begin
                if(instruction[31:25] == 7'd0) lists <= 4'd0;
                else lists <= 4'd1;
            end
            else if(instruction[14:12] == 3'b111) lists <= 4'd2;
            else if(instruction[14:12] == 3'b110) lists <= 4'd3;
            else lists <= 4'dx;
        end
        4'b0001: lists <= 4'd0;
        4'b0010: lists <= 4'd0;
        4'b0011: lists <= 4'd2;
        default: lists <= 4'd0;
    endcase
end

endmodule

  //---------testbench-datapath-------------------------

module testbench;
    reg [3:0] alu_cmd;
    reg [31:0]instruction;
    wire [3:0] lists;

    Ula_control control(.alu_cmd(alu_cmd), .instruction(instruction), .lists(lists));

    initial begin
        alu_cmd = 4'b0000;
        instruction = 32'b00000000000000000010000000000000;
        $display(" lists %d", lists);
        #10;
        alu_cmd = 4'b0000;
        instruction = 32'b10000000000000000110000000000000;
        $display(" lists %d", lists);
        #10;
        alu_cmd = 4'b0000;
        instruction = 32'b00000000000000000111000000000000;
        $display(" lists %d", lists);
        #10;
        alu_cmd = 4'b0000;
        instruction = 32'b00000000000000000000000000000000;
        $display(" lists %d", lists);
        #10;
        alu_cmd = 4'b0000;
        instruction = 32'b10000000000000000000000000000000;
        $display(" lists %d", lists);
        #10;
        alu_cmd = 4'b0001;
        instruction = 32'b10000000000000000000000000000000;
        $display(" lists %d", lists);
        #10;
        alu_cmd = 4'b0010;
        $display(" lists %d", lists);
        #10;
        alu_cmd = 4'b0011;
        $display(" lists %d", lists);
    end

    always #5 clk = ~clk;

endmodule