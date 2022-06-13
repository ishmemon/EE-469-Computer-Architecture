//Ismail Memon
//4/30/22
//EE 469
//Lab 3

/* arm is the spotlight of the show and contains the bulk of the datapath and control logic. This module is split into two parts, the datapath and control. 
In Lab 3, our job is to take the single cyle arm processor we made and were given in
lab 2 and pipeline it and take care of pipelining hazards to make it perform better.
The pipelining is 5 stage so 4 registers will be used.

*/

// clk - system clock
// rst - system rst
// Instr - incoming 32 bit instruction from imem, contains opcode, condition, addresses and or immediates
// ReadData - data read out of the dmem
// WriteData - data to be written to the dmem
// MemWrite - write enable to allowed WriteData to overwrite an existing dmem word
// PC - the current program count value, goes to imem to fetch instruciton
// ALUResult - result of the ALU operation, sent as address to the dmem

module arm (
    input  logic        clk, rst,
    input  logic [31:0] InstrF,
    input  logic [31:0] ReadDataM,
    output logic [31:0] WriteDataM, 
    output logic [31:0] PCF, ALUResultM,
    output logic MemWriteM
);
	 //These two incoming lines represent some more extra logic I had to declare for the
	 //diff stages/
	 logic MemWriteD, MemWriteE;
			logic [31:0] ALUOutM; 
	 //logic [31:0] ALUResultE;
	 logic [31:0] ALUResultE, ALUOutW, WriteDataE;
	 logic [31:0] InstrD, ReadDataW;
    // datapath buses and signals
    logic [31:0] PCPrime, PCPlus4F, PCPlus8D, PCPlus8E; // pc signals
    logic [ 3:0] RA1D, RA2D, RA1E, RA2E;                  // regfile input addresses
    logic [31:0] RD1D, RD2D, RD1E, RD2E;       // raw regfile outputs
    logic [ 3:0] ALUFlags;                  // alu combinational flag outputs
    logic [31:0] ExtImmD, ExtImmE, SrcAE, SrcBE;        // immediate and alu inputs 
    logic [31:0] ResultW;                    // computed or fetched value to be written into regfile or pc

    // control signals
    logic PCSrcD, MemtoRegD, ALUSrcD, RegWriteD, BranchD;
    logic [1:0] RegSrcD, ImmSrcD, ALUControlD;
	 //More control logic that has to be declared, it represents the control logic in
	 //the diff stages.
	 logic PCSrcE, MemtoRegE, ALUSrcE, RegWriteE, BranchE;
    logic [1:0] RegSrcE, ALUControlE;
	 logic PCSrcM, MemtoRegM, RegWriteM;
    logic PCSrcW, MemtoRegW, RegWriteW;
    //This logic is for the writing address that we have to take with us to deal with
	 //the hazard that what if we want to read when the load instruction isnt done yet.
	 //The one in the decode is just the instruction. 
	 logic [3:0] WA3D, WA3E, WA3M, WA3W;
	 assign WA3D = rst? 4'h0 : InstrD[15:12];
	 //This logic is for the conditional execution
	 logic [3:0] CondD, CondE;
	 //The D is the instruction and the E is pipelined.
	 assign CondD = rst? 4'h0 : InstrD[31:28];
	 //Control unit implemenator is the enable_path
	 logic enable_path;
	 
	 //These are all the signals taht are extra and are used for the pipeling and 
	 //hazard control. i had to declare them on top because otehrwise it would be an error
	 logic StallF, StallD, FlushD, FlushE, PCWrPendingF, Match_12D_E, ldrstall;
	 logic Match_1E_M, Match_2E_M, Match_1E_W, Match_2E_W;
	 logic [1:0] ForwardAE, ForwardBE;
	 logic BranchTakenE, BranchTakenF;
	 
	 

    /* The datapath consists of a PC as well as a series of muxes to make decisions about which data words to pass forward and operate on. It is 
    ** noticeably missing the register file and alu, which you will fill in using the modules made in lab 1. To correctly match up signals to the 
    ** ports of the register file and alu take some time to study and understand the logic and flow of the datapath.
    */
    //-------------------------------------------------------------------------------
    //                                      DATAPATH
    //-------------------------------------------------------------------------------


	 //Now there are two muxes to decide the pc thing because of the branch signal
	 //So I make a temp variable to store the output of the first mux.
	 logic [31:0] temp;
    assign temp = PCSrcW ? ResultW : PCPlus4F;  // mux, use either default or newly computed value
    assign PCPrime = BranchTakenE ? ALUResultE : temp;  //The second mux choses either temp or aluresult
	 assign PCPlus4F = PCF + 'd4;                  // default value to access next instruction
    //This needs to be changed there is no PCPlus8F it just goes to D.
	 //assign PCPlus8F = PCPlus4F + 'd4;             // value read when reading from reg[15]
    assign PCPlus8D = PCPlus4F;
    // update the PC, at rst initialize to 0.
	 //This is actually the register line. It is not a pipeline but it is a register
	 //and it also needs to respond to StallF.
    always_ff @(posedge clk) begin
        if (rst) PCF <= '0;
		  else if (StallF) PCF <= PCF;
        else     PCF <= PCPrime;
    end

    // determine the register addresses based on control signals
    // RegSrc[0] is set if doing a branch instruction
    // RefSrc[1] is set when doing memory instructions
    assign RA1D = RegSrcD[0] ? 4'd15        : InstrD[19:16];
    assign RA2D = RegSrcD[1] ? InstrD[15:12] : InstrD[ 3: 0];


    // TODO: insert your reg file here
    // TODO: The reg_file is instantiated with its parameters to create the register 
	 //part of the CPU
    
	 logic [31:0] useless, useless2, interR1, interR2;
	 reg_file u_reg_file (.clk, .wr_en(RegWriteW), .write_data(ResultW), 
	 .write_addr(WA3W), .read_addr1(RA1D), .read_addr2(RA2D), .read_data1(useless), 
	 .read_data2(useless2));
	 
	 always_comb begin
		if (rst) begin
			RD1D = 32'h00000000;
			RD2D = 32'h00000000;
		end else begin
			if (RA1D == 4'hf) begin
				RD1D = PCPlus8D;
			end else if (useless[31] == 1 | useless[31] == 0) begin
				RD1D = useless; 
			end else begin
				RD1D = 32'h00000000;
			end
			
			if (RA2D == 4'hf) begin
				RD2D = PCPlus8D;
			end else if (useless2[31] == 1 | useless2[31] == 0) begin
				RD1D = useless2; 
			end else begin
			   RD2D = 32'h00000000;
			end
		end
	 end
	 
	 //assign RD1D = rst? 32'h00000000 : ((RA1D == 4'hf)? PCPlus8D : useless);    
	 //assign RD2D = rst? 32'h00000000 : ((RA2D == 4'hf)? PCPlus8D : useless2);

        
	 
    // two muxes, put together into an always_comb for clarity
    // determines which set of instruction bits are used for the immediate
    always_comb begin
        if      (ImmSrcD == 'b00) ExtImmD = {{24{InstrD[7]}},InstrD[7:0]};          // 8 bit immediate - reg operations
        else if (ImmSrcD == 'b01) ExtImmD = {20'b0, InstrD[11:0]};                 // 12 bit immediate - mem operations
        else                     ExtImmD = {{6{InstrD[23]}}, InstrD[23:0], 2'b00}; // 24 bit immediate - branch operation
    end

    // WriteData and SrcA are direct outputs of the register file, wheras SrcB is chosen between reg file output and the immediate
    //I am NOT SURE about this I think PCPlus8 should be PCPlus8E.
	 logic [31:0] temp2;
	 logic [31:0] temp3;
	 //I dont thin that one is right it should be the output of the mux for B
	 //assign WriteDataE = (RA2E == 'd15) ? PCPlus8E : temp2;           // substitute the 15th regfile register for PC 
    //Source A and Source B have diff values depending on the 2 input mux that 
	 //the hazard unit uses so I just have a case statement for it. The one that goes in
	 //B goes through a second mux so I have a second temp variable to store the value
	 //that needs to be muxed again later. i also hve a temp variable for A as it also 
	 //goes through a decision.
	 always_comb begin
	    if (ForwardAE[0]&(~ForwardAE[1]))
			temp3 = ResultW;
		 else if ((~ForwardAE[0])&(ForwardAE[1]))
			temp3 = ALUOutM;
		 else 
			temp3 = RD1E;
	 end
	 always_comb begin
	    if (ForwardBE[0]&(~ForwardBE[1]))
			temp2 = ResultW;
		 else if ((~ForwardBE[0])&(ForwardBE[1]))
			temp2 = ALUOutM;
		 else 
			temp2 = RD2E;
	 end
	 
	 
	 
	 //i am not sure about these 2 muxes.
	 //assign SrcA      = (RA1E == 'd15) ? PCPlus8E : temp3;           // substitute the 15th regfile register for PC 
    //assign SrcB      = ALUSrcE      ? ExtImmE  : WriteDataE;     // determine alu operand to be either from reg file or from immediate
    assign SrcAE = rst? 32'h00000000 : temp3;
	 assign SrcBE = rst? 32'h00000000 : (ALUSrcE   ?   ExtImmE : temp2);
	 assign WriteDataE = temp2;
	 //assign writeDataE = rst? 32'h00000000 : temp2;
	 //assign writeDataE = 32'h00000008;
	 
    // TODO: insert your alu here
    // The ALU is instantiated with its parameters to create that unit for the processor
    //The ports come form the diagram of the processor 
	 //From Modelsim I see that SrcA and SrcB are the correct inputs and they behave 
	 //as expected, however, the ALUControl is not working. To fix that I created a logic
	 //controller that tells us the ALUControl early so we can use it to initialize the module
	 
    ///We assign the alu output to a temp variable so we can toggle between choosing 
	 //0 for the reset or that variable.
	 logic [31:0] tempALU;
	 alu u_alu (
        .a          (SrcAE), 
        .b          (SrcBE),
        .ALUControl (ALUControlE),
        .Result     (tempALU),
        .ALUFlags   (ALUFlags)
    );
	 assign ALUResultE = rst? 32'h00000000 : tempALU;

    // determine the result to run back to PC or the register file based on whether we used a memory instruction
    assign ResultW = MemtoRegW ? ReadDataW : ALUOutW;    // determine whether final writeback result is from dmemory or alu
 
	 //The flags holder needed for lab 2 and another one for lab3
	 //The flagsreg is what comes out of ALU and then FlagsRegE is what is used to define
	 //the condition.
	 logic [3:0] FlagsReg;
	 logic FlagWriteD, FlagWriteE;


	     
	     /* The hazard control stage defines and assigns the Flush and Stall logics that 
		  we use in the pipe lining stage that comes right after. The values we assign 
		  the logics are from the lecture
    */
    //-------------------------------------------------------------------------------
    //                                      HAZARD CONTROL
    //-------------------------------------------------------------------------------
	 //Here we assign the variables based on the slides in lecture
	 assign BranchTakenE = BranchE & enable_path;
	 assign BranchTakenF = BranchTakenE;
	 
	 assign Match_1E_M = rst? 1'b0 : (RA1E == WA3M);
	 assign Match_2E_M = rst? 1'b0 : (RA2E == WA3M);
	 assign Match_1E_W = rst? 1'b0 : (RA1E == WA3W);
	 assign Match_2E_W = rst? 1'b0 : (RA2E == WA3W);
	 
	 //This assigns the forwardae values and we use & instead of the * in lecture.
	 always_comb begin
	    if (Match_1E_M & RegWriteM) 
			ForwardAE = 2'b10;
		 else if (Match_1E_W & RegWriteW)
			ForwardAE = 2'b01;
		 else 
			ForwardAE = 2'b00;
	 end
	 //This assigns the forwardbe values and we use & instead of the * in lecture.
	 always_comb begin
	    if (Match_2E_M & RegWriteM) 
			ForwardBE = 2'b10;
		 else if (Match_2E_W & RegWriteW)
			ForwardBE = 2'b01;
		 else 
			ForwardBE = 2'b00;
	 end
	 
	 //assign ForwardAE = 2'b00;
	 //assign ForwardBE = 2'b00;
	 
	 assign Match_12D_E = rst? 1'b0 : ((RA1D == WA3E) + (RA2D == WA3E));
	 assign ldrstall = rst? 1'b0 : (Match_12D_E & MemtoRegE);
	 assign StallF = rst? 1'b0 : (ldrstall + PCWrPendingF);
	 assign StallD = rst? 1'b0 : ldrstall;
	 assign FlushE = rst? 1'b0 : (ldrstall + BranchTakenF);
	 assign FlushD = rst? 1'b0 : (PCWrPendingF + PCSrcW + BranchTakenF);
	 
	 assign PCWrPendingF = rst? 1'b0 : (PCSrcD + PCSrcE + PCSrcM);
	 
	 
	 //assign Match_12D_E = 1'b0;
	 //assign ldrstall = 1'b0;
	 //assign StallF = 1'b0;
	 //assign StallD = 1'b0;
	 //assign FlushE = 1'b0;
	 //assign FlushD = 1'b0;
	 //assign PCWrPendingF = 1'b0;
	 
	 
	 
	 
	 
	 
		  /* The pipelining stage has 4 large register columns created from 1 bit instantiations
		  of the pipe_reg file. We will first rename the signals used in the rest of the code to 
		  the apprporiate names, for example, the alu should not get SrcA and SrcB but instead
		  SrcAE and SrcBE so we will rename signals based on what they should be and then in this
		  stage we will use the registers to create those signals.
    */
    //-------------------------------------------------------------------------------
    //                                      PIPELINING
    //-------------------------------------------------------------------------------
    

	 
	 //This is the first pipe register from the left. It takes InstrF and flops it to
	 //InstrD. The flush and stall are FlushD and StallD. TO simplify I just use a genvar statement.
	 genvar y;
	 generate
		  for (y = 0; y < 32; y++) begin : firstpipe
		      //I pass in the values to the pipe_reg modules.
		      pipe_reg p2 (.rst, .q(InstrD[y]), .d(InstrF[y]), .clk, .flush(FlushD), .stall(StallD));
		  end
	 endgenerate
	 
	 //This is for the second long pipeline, each of the controlsignals are passed one by one
	 //The datapath signals use genvar statements.
	 //FlushE is the flush but there is no stall so that is 0.
	 pipe_reg p3 (.rst, .q(PCSrcE), .d(PCSrcD), .clk, .flush(FlushE), .stall(1'b0));
	 pipe_reg p4 (.rst, .q(RegWriteE), .d(RegWriteD), .clk, .flush(FlushE), .stall(1'b0));
	 pipe_reg p5 (.rst, .q(MemtoRegE), .d(MemtoRegD), .clk, .flush(FlushE), .stall(1'b0));
	 pipe_reg p6 (.rst, .q(MemWriteE), .d(MemWriteD), .clk, .flush(FlushE), .stall(1'b0));
	 pipe_reg p7 (.rst, .q(ALUControlE[0]), .d(ALUControlD[0]), .clk, .flush(FlushE), .stall(1'b0));
	 pipe_reg p7_2 (.rst, .q(ALUControlE[1]), .d(ALUControlD[1]), .clk, .flush(FlushE), .stall(1'b0));
	 pipe_reg p8 (.rst, .q(BranchE), .d(BranchD), .clk, .flush(FlushE), .stall(1'b0));
	 pipe_reg p9 (.rst, .q(ALUSrcE), .d(ALUSrcD), .clk, .flush(FlushE), .stall(1'b0));
	 pipe_reg p10 (.rst, .q(FlagWriteE), .d(FlagWriteD), .clk, .flush(FlushE), .stall(1'b0));
	 pipe_reg p11 (.rst, .q(CondE[0]), .d(CondD[0]), .clk, .flush(FlushE), .stall(1'b0));
	 pipe_reg p12 (.rst, .q(CondE[1]), .d(CondD[1]), .clk, .flush(FlushE), .stall(1'b0));
	 pipe_reg p13 (.rst, .q(CondE[2]), .d(CondD[2]), .clk, .flush(FlushE), .stall(1'b0));
	 pipe_reg p14 (.rst, .q(CondE[3]), .d(CondD[3]), .clk, .flush(FlushE), .stall(1'b0));
	 
	 
    //Now we do the datapath side
	 pipe_reg p17 (.rst, .q(WA3E[3]), .d(WA3D[3]), .clk, .flush(FlushE), .stall(1'b0));
	 pipe_reg p18 (.rst, .q(WA3E[2]), .d(WA3D[2]), .clk, .flush(FlushE), .stall(1'b0));
	 pipe_reg p19 (.rst, .q(WA3E[1]), .d(WA3D[1]), .clk, .flush(FlushE), .stall(1'b0));
	 pipe_reg p20 (.rst, .q(WA3E[0]), .d(WA3D[0]), .clk, .flush(FlushE), .stall(1'b0));
	 //We also have to flop RA1D to RA1E and RA2D to RA2E so we can use them for the 
	 //hazard unit.
	 pipe_reg p47 (.rst, .q(RA1E[3]), .d(RA1D[3]), .clk, .flush(FlushE), .stall(1'b0));
	 pipe_reg p48 (.rst, .q(RA1E[2]), .d(RA1D[2]), .clk, .flush(FlushE), .stall(1'b0));
	 pipe_reg p49 (.rst, .q(RA1E[1]), .d(RA1D[1]), .clk, .flush(FlushE), .stall(1'b0));
	 pipe_reg p50 (.rst, .q(RA1E[0]), .d(RA1D[0]), .clk, .flush(FlushE), .stall(1'b0));
	 pipe_reg p57 (.rst, .q(RA2E[3]), .d(RA2D[3]), .clk, .flush(FlushE), .stall(1'b0));
	 pipe_reg p58 (.rst, .q(RA2E[2]), .d(RA2D[2]), .clk, .flush(FlushE), .stall(1'b0));
	 pipe_reg p59 (.rst, .q(RA2E[1]), .d(RA2D[1]), .clk, .flush(FlushE), .stall(1'b0));
	 pipe_reg p60 (.rst, .q(RA2E[0]), .d(RA2D[0]), .clk, .flush(FlushE), .stall(1'b0));
	 //THis genvar statement flops RD1, RD2, and ExtImm. And PCPlus8.
	 genvar z;
	 generate
		  for (z = 0; z < 32; z++) begin : secondpipe
		      //I pass in the values to the pipe_reg modules.
		      pipe_reg p21 (.rst, .q(RD1E[z]), .d(RD1D[z]), .clk, .flush(FlushE), .stall(1'b0));
			   pipe_reg p22 (.rst, .q(RD2E[z]), .d(RD2D[z]), .clk, .flush(FlushE), .stall(1'b0)); 
				pipe_reg p23 (.rst, .q(ExtImmE[z]), .d(ExtImmD[z]), .clk, .flush(FlushE), .stall(1'b0)); 
				pipe_reg p23_1 (.rst, .q(PCPlus8E[z]), .d(PCPlus8D[z]), .clk, .flush(FlushE), .stall(1'b0)); 	
		  end
	 endgenerate

	 
	 //Third pipeline passes 4 signals they are also anded with the enable_path
	 //which is the conditioanl unit. No flush or stall so they are both 0. 
	 pipe_reg p25 (.rst, .q(PCSrcM), .d(PCSrcE&enable_path), .clk, .flush(1'b0), .stall(1'b0));
	 pipe_reg p26 (.rst, .q(RegWriteM), .d(RegWriteE&enable_path), .clk, .flush(1'b0), .stall(1'b0));
	 pipe_reg p27 (.rst, .q(MemtoRegM), .d(MemtoRegE), .clk, .flush(1'b0), .stall(1'b0));
	 pipe_reg p28 (.rst, .q(MemWriteM), .d(MemWriteE&enable_path), .clk, .flush(1'b0), .stall(1'b0));
	 pipe_reg p29 (.rst, .q(WA3M[3]), .d(WA3E[3]), .clk, .flush(1'b0), .stall(1'b0));
	 pipe_reg p29_2 (.rst, .q(WA3M[2]), .d(WA3E[2]), .clk, .flush(1'b0), .stall(1'b0));
	 pipe_reg p29_3 (.rst, .q(WA3M[1]), .d(WA3E[1]), .clk, .flush(1'b0), .stall(1'b0));
	 pipe_reg p29_4 (.rst, .q(WA3M[0]), .d(WA3E[0]), .clk, .flush(1'b0), .stall(1'b0));
	 //This genvar statement flops ALUResult and WriteData
	 genvar m;
	 generate
	 for (m = 0; m < 32; m++) begin : thirdpipe
		   //I pass in the values to the pipe_reg modules.
         pipe_reg p29_5 (.rst, .q(ALUResultM[m]), .d(ALUResultE[m]), .clk, .flush(1'b0), .stall(1'b0));
			pipe_reg p29_6 (.rst, .q(WriteDataM[m]), .d(WriteDataE[m]), .clk, .flush(1'b0), .stall(1'b0));
	  end
	 endgenerate

	 //This is the last pipeline it only passes 4 signals. There is no flush or stall so 
	 //they are both 0.
	 pipe_reg p30 (.rst, .q(PCSrcW), .d(PCSrcM), .clk, .flush(1'b0), .stall(1'b0));
	 pipe_reg p31 (.rst, .q(RegWriteW), .d(RegWriteM), .clk, .flush(1'b0), .stall(1'b0));
	 pipe_reg p32 (.rst, .q(MemtoRegW), .d(MemtoRegM), .clk, .flush(1'b0), .stall(1'b0));
	 pipe_reg p33 (.rst, .q(WA3W[3]), .d(WA3M[3]), .clk, .flush(1'b0), .stall(1'b0));
	 pipe_reg p34 (.rst, .q(WA3W[2]), .d(WA3M[2]), .clk, .flush(1'b0), .stall(1'b0));
	 pipe_reg p35 (.rst, .q(WA3W[1]), .d(WA3M[1]), .clk, .flush(1'b0), .stall(1'b0));
	 pipe_reg p36 (.rst, .q(WA3W[0]), .d(WA3M[0]), .clk, .flush(1'b0), .stall(1'b0));
	 
	 //This genvar statement flops ALUOut and ReadData but first I must assign ALUOut
	 //to ALUResult
	 assign ALUOutM = ALUResultM;
	 genvar n;
	 generate
	 for (n = 0; n < 32; n++) begin : fourpipe
		   //I pass in the values to the pipe_reg modules.
         pipe_reg p37 (.rst, .q(ALUOutW[n]), .d(ALUOutM[n]), .clk, .flush(1'b0), .stall(1'b0));
			pipe_reg p38 (.rst, .q(ReadDataW[n]), .d(ReadDataM[n]), .clk, .flush(1'b0), .stall(1'b0));
	  end
	 endgenerate
	 
	 
    /* The control conists of a large decoder, which evaluates the top bits of the instruction and produces the control bits 
    ** which become the select bits and write enables of the system. The write enables (RegWrite, MemWrite and PCSrc) are 
    ** especially important because they are representative of your processors current state. 
    */
    //-------------------------------------------------------------------------------
    //                                      CONTROL
    //-------------------------------------------------------------------------------
    
	 //Here we assign FlagWrite to be if Instr[20] is 1 because that is the set flags bit
	 assign FlagWriteD = rst? 1'b0 : InstrD[20]; 
	 //This block changes the value of FlagsReg is FlagWriteE is asserted.
	 always_ff @(FlagWriteE) begin
		if (rst) begin
			FlagsReg <= 4'b0000;
		end else begin
			FlagsReg <= ALUFlags;
		end
	 end
	 
	 //Some logic to set the condition for the branch depending on the flags
	 //I have an enable logic that allows the stuff from the branch below to happen
	 //if the conditions are met otherwise nothing happens. That logic is declard above
	 //it is enable_path
	 always_comb begin
		  case (CondE)
		      4'b1110: begin
					enable_path = 1;
				end
				4'b0000: begin
					//If 0000 we are checking for equality so the zero flag must be true
					enable_path = ((FlagsReg[2]));
				end
		  		4'b0001: begin
					//This is the not equal so the zero flag must be false
					enable_path = (~(FlagsReg[2]));
				end
		  		4'b1010: begin
				   //This is greater or equal so not less, so the negative flag cant be true
					enable_path = (~(FlagsReg[3]));
				end
				4'b1100: begin
					//This is greater than so the negative flag cant be true and the zero cant
					//be true
					enable_path = (~(FlagsReg[3]) & (~(FlagsReg[2])));
				end
				4'b1101: begin
					//This is less than or equal so we have negative flag is true or zero 
					enable_path = ((FlagsReg[3]) | ((FlagsReg[2])));
				end
				4'b1011: begin
					//This is less than so negative has to be true and zero cant be true
					enable_path = ((FlagsReg[3]) & (~(FlagsReg[2])));
				end
				//The default case is 0 I disable the path bec an invalid condition code
				//is sent. 
				default: begin
					enable_path = 0;
				end
		  endcase 
	 end

    //These cases have to be only made true if enable_path. Only if the enable_path
	 //is true can they be executed. So enable_path is bascally the conditional unit
	 //and that will be implemented in the pipeline part. 
	 //I am PRETTY SURE that since these look at the condition these should be D values.
	 //If the Branch is true then the BranchD is true.
	 
	 always_comb begin
        casez (InstrD[27:20])
            // ADD (Imm or Reg)
            8'b00?_0100_0 : begin   // note that we use wildcard "?" in bit 25. That bit decides whether we use immediate or reg, but regardless we add 
					 BranchD = 0;
					 PCSrcD    = 0;
                MemtoRegD = 0; 
                MemWriteD = 0; 
                ALUSrcD   = InstrD[25]; // may use immediate
                RegWriteD = 1;
                RegSrcD   = 'b00;
                ImmSrcD   = 'b00; 
                ALUControlD = 'b00;
            end

            // SUB (Imm or Reg)
            8'b00?_0010_0 : begin   // note that we use wildcard "?" in bit 25. That bit decides whether we use immediate or reg, but regardless we sub
				    BranchD = 0;
					 PCSrcD    = 0; 
                MemtoRegD = 0; 
                MemWriteD = 0; 
                ALUSrcD   = InstrD[25]; // may use immediate
                RegWriteD = 1;   //I dont know about this one
                RegSrcD   = 'b00;
                ImmSrcD   = 'b00; 
                ALUControlD = 'b01;
            end
				
				
            // CMP (Imm or Reg)
            8'b00?_0010_1 : begin   // note that we use wildcard "?" in bit 25. That bit decides whether we use immediate or reg, but regardless we sub
				    BranchD = 0;
					 PCSrcD    = 0; 
                MemtoRegD = 0; 
                MemWriteD = 0; 
                ALUSrcD   = InstrD[25]; // may use immediate
                RegWriteD = 1;
                RegSrcD   = 'b00;
                ImmSrcD   = 'b00; 
                ALUControlD = 'b01;
            end

            // AND
            8'b000_0000_0 : begin  
					 BranchD = 0;
					 PCSrcD    = 0; 
                MemtoRegD = 0; 
                MemWriteD = 0; 
                ALUSrcD   = 0; 
                RegWriteD = 1;
                RegSrcD   = 'b00;
                ImmSrcD   = 'b00;    // doesn't matter
                ALUControlD = 'b10;	 
            end

            // ORR
            8'b000_1100_0 : begin  
					 BranchD = 0;
					 PCSrcD    = 0; 
                MemtoRegD = 0; 
                MemWriteD = 0; 
                ALUSrcD   = 0; 
                RegWriteD = 1;
                RegSrcD   = 'b00;
                ImmSrcD   = 'b00;    // doesn't matter
                ALUControlD = 'b11;
            end

            // LDR
            8'b010_1100_1 : begin  
					 BranchD = 0;
					 PCSrcD    = 0; 
                MemtoRegD = 1; 
                MemWriteD = 0; 
                ALUSrcD   = 1;
                RegWriteD = 1;
                RegSrcD   = 'b10;    // msb doesn't matter
                ImmSrcD   = 'b01; 
                ALUControlD = 'b00;  // do an add
            end

            // STR
            8'b010_1100_0 : begin 
					 BranchD = 0;
					 PCSrcD    = 0; 
                MemtoRegD = 0; // doesn't matter
                MemWriteD = 1; 
                ALUSrcD   = 1;
                RegWriteD = 0;
                RegSrcD   = 'b10;    // msb doesn't matter
                ImmSrcD   = 'b01; 
                ALUControlD = 'b00;  // do an add
            end

            // B
            8'b1010_???? : begin    
						  BranchD = 1;
						  PCSrcD    = 1; 
                    MemtoRegD = 0;
                   MemWriteD = 0; 
                    ALUSrcD   = 1;
                    RegWriteD = 0;
                    RegSrcD   = 'b01;
                    ImmSrcD   = 'b10; 
                    ALUControlD = 'b00;  // do an add
            end

			default: begin
					PCSrcD    = 0; 
                    BranchD = 0;
						  MemtoRegD = 0; // doesn't matter
                    MemWriteD = 0; 
                    ALUSrcD   = 0;
                    RegWriteD = 0;
                    RegSrcD   = 'b00;
                    ImmSrcD   = 'b00; 
                    ALUControlD = 'b00;  // do an add
			end
        endcase
    end


endmodule 