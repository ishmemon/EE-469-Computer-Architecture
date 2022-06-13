//Attribution: This file was contributed by the UW Electrical Engineering Faculty.
//Ismail Memon
//4/7/22
//EE 469
//Lab 1 Task 3

//This module was copied from the course website when setting up the initial project.

/* A 1-bit adder with carry-in and carry-out.
 * 
 * Inputs:
 *   a    - 1-bit input #1
 *   b    - 1-bit input #2
 *   cin  - 1-bit carry-in (connected to cout of another fullAdder)
 *
 * Outputs:
 *   sum  - LSB of a+b+cin
 *   cout - MSB of a+b+cin (connected to cin of another fullAdder) 
 */

 module fullAdder (a, b, cin, sum, cout);

	input  logic a, b, cin;
	output logic sum, cout;
	
	assign  sum = a ^ b ^ cin;
	assign cout = (a & b) | (cin & (a ^ b));

endmodule  // fullAdder


/* testbench for the fullAdder */
module fullAdder_testbench();

	logic a, b, cin, sum, cout;
	
	// using Verilog's positional port connections (not recommended)
	fullAdder dut (a, b, cin, sum, cout);
	
	integer i;
	initial begin
	
		// ** syntax is "to the power of"
		for (i = 0; i < 2**3; i++) begin
			{a, b, cin} = i; #10;
		end  // for
		
	end  // initial

endmodule  // fullAdder_testbench
