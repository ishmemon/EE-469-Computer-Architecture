//Ismail Memon
//4/30/22
//EE 469
//Lab 3

/* Pipe_reg takes acts as a flip flop register that makes q = d at the edge of the clock.

Inputs are d, one bit signal, clk, the clock, flush, which , and stall, and reset.
The ouput is 1 bit q that is synchronized to the clock. 
These are individual registers (flip flops) that will combine to make those giant
wall registers we take care of everything one bit at a time. 
All the registers will have the flush and stall we can always disable those signals 
if we dont want them this helps so we dont have to make multiple flip flops. 
*/
module pipe_reg (rst, q, d, clk, flush, stall);
	input logic rst, d, clk, flush, stall;
	output logic q;
	
	always_ff @(posedge clk) begin
		  if (rst | flush)   
			  q <= 0;
			//If stall we keep the old q we do not look at the input. 
		  else if (stall)  
			  q <= q;
	     else  
		     q <= d;
	end
endmodule 

//Testbench to test the pipe_reg. Goes through a few clock edges to see whether its action
//is as expected and whether the flush and stall signals work.
module pipe_reg_testbench(); 
	logic d, rst, clk, flush, stall, q;
		
	pipe_reg dut (.*);

	// Set up a simulated clk.   
	parameter CLOCK_PERIOD=100;  
	initial begin   
		clk <= 0;  
		forever #(CLOCK_PERIOD/2) clk <= ~clk; // Forever toggle the clk 
	end 
	
	initial begin  
	//I flush first and then pass in data and change it and then I stall to keep the 
   //data locked and then I unstall and change the data and flush.
   //The simulation should answer whether this module works as expected. 	
								@(posedge clk);   
		flush = 1; stall = 0;   @(posedge clk);
	@(posedge clk);
		flush = 0; d = 1; @(posedge clk);
		@(posedge clk);
		d = 0; 
		@(posedge clk);
		d = 1; stall = 1; @(posedge clk);
		@(posedge clk);
		d = 0; @(posedge clk);
		@(posedge clk);
		stall = 0; @(posedge clk);
		@(posedge clk);
		d = 1; @(posedge clk);
		flush = 1; @(posedge clk);
		@(posedge clk);
		@(posedge clk);

		$stop; // End the simulation.  
   end  
endmodule 