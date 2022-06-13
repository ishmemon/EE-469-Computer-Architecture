/* top is a structurally made toplevel module. It consists of 3 instantiations, as well as the signals that link them. 
** It is almost totally self-contained, with no outputs and two system inputs: clk and rst. clk represents the clock 
** the system runs on, with one instruction being read and executed every cycle. rst is the system reset and should 
** be run for at least a cycle when simulating the system.
*/

// clk - system clock
// rst - system reset. Technically unnecessary
module top(
    input logic clk, rst
);
    
    // processor io signals
    logic [31:0] InstrF;
    logic [31:0] ReadDataM;
    logic [31:0] WriteDataM;
    logic [31:0] PCF, ALUResultE;
    logic        MemWriteM;
	 
	 logic [31:0] InstrF_up, ReadDataM_up;
	 assign InstrF_up = rst? 32'h00000000 : InstrF;
	 assign ReadDataM_up = rst? 32'h00000000 : ReadDataM;

    // our single cycle arm processor
    arm processor (
        .clk        (clk        ), 
        .rst        (rst        ),
        .InstrF      (InstrF_up      ),
        .ReadDataM   (ReadDataM_up   ),
        .WriteDataM  (WriteDataM  ), 
        .PCF         (PCF         ), 
        .ALUResultM  (ALUResultE  ),
        .MemWriteM   (MemWriteM   )
    );

    // instruction memory
    // contained machine code instructions which instruct processor on which operations to make
    // effectively a rom because our processor cannot write to it
    imem imemory (
        .addr   (PCF     ),
        .instr  (InstrF )
    );

    // data memory
    // containes data accessible by the processor through ldr and str commands
    dmem dmemory (
        .clk     (clk       ), 
        .wr_en   (MemWriteM  ),
        .addr    (ALUResultE ),
        .wr_data (WriteDataM ),
        .rd_data (ReadDataM  )
    );
	 
	 //The reset signal was not working so these two logics will make sure that if we 
	 //have a reset the value of these are 0.


endmodule 

/* testbench is a simulation module which simply instantiates the processor system and runs 50 cycles 
** of instructions before terminating. At termination, specific register file values are checked to
** verify the processors’ ability to execute the implemented instructions.
*/
/* testbench is a simulation module which simply instantiates the processor system and runs 50 cycles 
** of instructions before terminating. At termination, specific register file values are checked to
** verify the processors’ ability to execute the implemented instructions.
*/
module top_testbench();

    // system signals
    logic clk, rst;

    // generate clock with 100ps clk period 
    initial begin
        clk = '1;
        forever #50 clk = ~clk;
    end

    // processor instantion. Within is the processor as well as imem and dmem
    top cpu (.clk(clk), .rst(rst));

    initial begin
        // start with a basic reset
        rst = 1; @(posedge clk);
        rst <= 0; @(posedge clk);

        // repeat for 50 cycles. Not all 50 are necessary, however a loop at the end of the program will keep anything weird from happening
        repeat(50) @(posedge clk);

        // basic checking to ensure the right final answer is achieved. These DO NOT prove your system works. A more careful look at your 
        // simulation and code will be made.

        // task 1:
        assert(cpu.processor.u_reg_file.memory[8] == 32'd11) $display("Task 1 Passed");
        else                                                 $display("Task 1 Failed");

        // task 2:
        //assert(cpu.processor.u_reg_file.memory[8] == 32'd1)  $display("Task 2 Passed");
        //else                                                 $display("Task 2 Failed");

        $stop;
    end

endmodule 