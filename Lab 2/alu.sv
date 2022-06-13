//Ismail Memon
//4/7/22
//EE 469
//Lab 1 Task 3

//The ALU module takes in two 32 bit inputs a and b, a 2 bit control that chooses
//which of 4 operations (and, or, add, subtract) to pursue, a 32 bit result (output) that 
//gives the result of the pursued operation, and a 4 bit flag (also output) that is
//true for given special cases of the output. This module is a critical part of a CPU
//and is an implementation of the ALU covered in lecture.
module alu ( input logic [31:0] a, b,
				 input logic [1:0] ALUControl,
				 output logic [31:0] Result,
				 output logic [3:0] ALUFlags);

	
	//All the 4 possible outputs
	logic [31:0] and_out;
	logic [31:0] or_out;
	logic [31:0] add;
	logic [31:0] sub;
	
	assign and_out = a & b;  //These are bitwise so the and_out should give the and of a and b
	assign or_out = a | b;   //These are bitwise so the or_out should give the or of a and b
	
	logic [31:0] inverted_b;
	logic final_c_out_1;
	logic final_c_out_2;
	logic carry_out;
	assign inverted_b = (~b+32'b00000000000000000000000000000001);
	//I use the add_32 module to add and subtract the a and b.
	//I have two final c outs, one for add one for sub. I cant have just once bec then 
	//there is an error. If there is a subtraction, I pass inverted b instead of b as
	//the second input.
	
	/*logic [32:0] temp1, temp2, temp3, temp4;
	assign temp1[31:0] = a;
	assign temp2[31:0] = b;
	
	assign temp3 = temp1 + temp2;
	assign temp4 = temp1 - temp2;
	
	assign add = temp3[31:0];
	assign sub = temp4[31:0];
	assign final_c_out_1 = temp3[32];
	assign final_c_out_2 = temp4[32];*/
	
	add_32 add1 (.a, .b, .out(add), .final_c_out(final_c_out_1));
   add_32 sub1 (.a, .b(inverted_b), .out(sub), .final_c_out(final_c_out_2));
	

   //This logic sum will help in setting the overflow flag. Just a 32 bit variable
	//that is either the add or subtract depending on what's selected.
	logic [31:0] sum;
				 

	//This block choses the corresponding operation based on the value of the ALU Control
   //signal. It acts as the selector and assigns Result the corresponding value.	
	always_comb begin
		case (ALUControl)	
	    //If 11 then we do OR
	    2'b11: Result = or_out;
		 //If 10 then we do and
		 2'b01: Result = sub;
		 //If 01 then we do subtract
		 2'b10: Result = and_out; 
		 2'b00: Result = add; 
		 endcase 
	end
	
	//This block assigns sum the value of add or subtract depending on what ALUControl[1]
	//is. This allows for sum to reflect the sum if addition is selected or the difference
	//if subtraction is selected, allowing it to be used for the flag setting operations
	always_comb begin
		case (ALUControl[1])
			1'b0: sum = add;
			1'b1: sum = sub;
		
		endcase
	end
	
	//This always comb block assigns the carry out. The carry out is the add carry out
	//if the chosen operation is add and the subtract carry out if the chosen operation
	//is subtract. This carry out is also used in the flag setting operations.
	always_comb begin
		case (ALUControl[1])
			1'b0: carry_out = final_c_out_1;
			1'b1: carry_out = final_c_out_2;
		
		endcase
	end

	//Now I will have to assign the flags. 3 is N 2 is Z 1 is C and 0 is V using the
	//letters in class
   //All the formulas are from lecture, they are just implemented here in Boolean equations	
	assign ALUFlags[3] = Result[31];
	assign ALUFlags[2] = (Result == 32'b00000000000000000000000000000000);
	//These formulas I got from lecture
   assign ALUFlags[1] = carry_out & (~ALUControl[1]);
 	assign ALUFlags[0] = (~(ALUControl[0] ^ a[31] ^ b[31])) & (a[31] ^ Result[31]) & (~ALUControl[1]);
				 
				 
endmodule 


//The alu testbench tests against the table of vectors completed and placed in the alu.tv
//file. They are loaded into the testvectors array and each of them are tested.
module alu_testbench();
	logic [31:0] a, b;
	logic [1:0] ALUControl;
	logic [31:0] Result;
	logic [3:0] ALUFlags;
	
	alu dut (a, b, ALUControl, Result, ALUFlags);
	
	logic [103:0] testvectors [1000:0];
	logic clk;
	// Set up a simulated clock
	parameter CLOCK_PERIOD=100;  
	initial clk = 1;
   always begin   
		#(CLOCK_PERIOD/2);
		clk <= ~clk; // Forever toggle the clock 
	end 
	
	initial begin
		$readmemh("alu.tv", testvectors);
		
		for (int i = 0; i < 20; i = i+1) begin
			{ALUControl, a, b, Result, ALUFlags} = testvectors[i]; @(posedge clk);
		end
	
	end

endmodule 