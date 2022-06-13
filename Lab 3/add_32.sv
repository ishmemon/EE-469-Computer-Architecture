//Ismail Memon
//4/7/22
//EE 469
//Lab 1 Task 3

//This is a 32 bit adder, it takes two 32 bit inputs a and b and outputs a 32 bit
//value out and has a 1 bit carry out that is also an output. This module is used by
//the ALU and uses 1 bit adders to make a 32 bit adder. We needed a 32 bit adder since 
//the ALU uses 32 bit values.

module add_32 (input logic [31:0] a, b, output logic [31:0] out, output logic final_c_out);
    //We will need a lot of 1 bit adders for this to work. First one has no carry in
	 //We have a logic to account for the carry out's 
	 logic [31:0] carry_outs;
	 fullAdder bit1 (.a(a[0]), .b(b[0]), .cin(1'b0), .sum(out[0]), .cout(carry_outs[0]));
	 
	 //This genvar statement creates 31 one bit adders (32 - the 1 from above) and uses
	 //the corresponding bits of a and b as inputs and sets the corresponding bit of the
	 //output. The carry out value is placed in the array and the carry in is the carry out
	 //of the previous addition.
	 genvar x;
	 generate
		  for (x = 1; x < 32; x++) begin : eachFullAdder
		      //I pass in the values the carry in is the carry out of the prev
		      fullAdder fa (.a(a[x]), .b(b[x]), .cin(carry_outs[x-1]), .sum(out[x]), .cout(carry_outs[x]));
		  end
	 
	 endgenerate
	 
	 //I assign the final carry out as the last value of the carry out array
	 assign final_c_out = carry_outs[31];
	 
endmodule 

//The testbench tests out a few combinations of a and b and sees if their addition works
//as expected. There is 1 case that causes overflow so we can test that too.
module add_32_testbench();
	logic [31:0] a, b, out;
	logic final_c_out;
	
	add_32 dut (a, b, out, final_c_out);
	
	initial begin
	    //I'm just going to try a few combos to see if the work
       //0 + 0
		 a = 32'b00000000000000000000000000000000; b = 32'b00000000000000000000000000000000; #10;
		 //16 + 16
		 a = 32'b00000000000000000000000000010000; b = 32'b00000000000000000000000000010000; #10;
		 //- 1 + 1 this should cause an overflow
		 a = 32'b11111111111111111111111111111111; b = 32'b00000000000000000000000000000001; #10;
		
	end  

endmodule 