//Ismail Memon
//4/7/22
//EE 469
//Lab 1 Task 2

//The reg_file takes in a clock a write enable signal (which if true gets the module
//to write to memory the given write_data) and two read adresses. The outputs are two 
//read_data lines that correspond to each of the read adresses. This module serves
//as our register and impelents a register which will be very useful for upcoming labs
module reg_file (input logic clk, wr_en,
						input logic [31:0] write_data,
						input logic [3:0] write_addr,
						input logic [3:0] read_addr1, read_addr2,
						output logic [31:0] read_data1, read_data2);
						
						
	//Array of memory
	logic [15:0][31:0] memory;
	
	//If write is enabled we write to the given address in memory.
	//We do this on the clockedge.
	always_ff @(posedge clk) begin
		if (wr_en) begin
			memory[write_addr] <= write_data;
		end

	end
	
	//The read always happens since there are two independent reads I do it
   //separately
	assign read_data1 = memory[read_addr1];
	assign read_data2 = memory[read_addr2];
		
endmodule 


//The reg_file testbench tests certain cases of the register file. We write data to
//an adress and then read it and then we read with the write asserted simultaneosuly. 
module reg_file_testbench();
	logic clk, wr_en;
	logic [31:0] write_data;
	logic [3:0] write_addr;
	logic [3:0] read_addr1, read_addr2;
	logic [31:0] read_data1, read_data2;
	
	reg_file dut (clk, wr_en, write_data, write_addr, read_addr1, read_addr2, read_data1, read_data2);
   
	// Set up a simulated clock.   
	parameter CLOCK_PERIOD=100;  
	initial begin  
		clk <= 0;  
		forever #(CLOCK_PERIOD/2) clk <= ~clk; // Forever toggle the clock 
	end  
	
	
	initial begin   	   
	   //First i initialize everythign to 0.
		wr_en <= 0; write_data <= 0; write_addr <= 0; read_addr1 <= 0; read_addr2 <= 0; @(posedge clk);   
                     @(posedge clk);   
                      @(posedge clk);
		//Here I write to adress 0001
		write_addr <= 4'b0001; write_data <= 32'b00000000000000000000000000000010; wr_en <= 1; @(posedge clk);   
                     @(posedge clk);   
                      @(posedge clk);
		//I turn off the write enable and read from adress 0001 using read_addr1 
		wr_en <= 0; read_addr1 <= 4'b0001; @(posedge clk);   
                     @(posedge clk);   
                      @(posedge clk); 
	   //I then write to adress 0010
	   write_addr <= 4'b0010; write_data <= 32'b00000000000000000000000000000100; wr_en <= 1; @(posedge clk);   
                     @(posedge clk);   
                      @(posedge clk);
		//I then turn off the write enable and read from adress 0010 using read_addr2
		wr_en <= 0; read_addr2 <= 4'b0010; @(posedge clk);   
                     @(posedge clk);   
                      @(posedge clk);
		//I then read with the write asserted using read_addr1. at adress 0011
		write_addr <= 4'b0011; write_data <= 32'b00000000000000000000000000001000; wr_en <= 1; read_addr1 <= 4'b0011; @(posedge clk);   
                     @(posedge clk);   
                      @(posedge clk);
	   //I then read with the write asserted using read_addr2. at adress 0100
		write_addr <= 4'b0100; write_data <= 32'b00000000000000000000000000000101; wr_en <= 1; read_addr2 <= 4'b0100; @(posedge clk);   
                     @(posedge clk);   
                      @(posedge clk);
		$stop; // End the simulation.  
   end 
	
endmodule
	