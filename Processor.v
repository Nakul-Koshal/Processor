`timescale 1ns / 1ps


// Using Define Directive (use GLOBAL INCLUDE for other files to acess the define directives used here)
// Fields of IR
`define oper_type IR[31:27]     //Operation type 
`define rdst      IR[26:22]     //Destination Register
`define rsrc1     IR[21:17]     //Source Register 1
`define imm_mode  IR[16]        //Mode to specify if its Immediate or Regiter type
`define rsrc2     IR[15:11]     //Source Register 2
`define isrc      IR[15:0]      //Immediate Value

// ----------------------------INSTRUCTION SET------------------------------
// Moving Operations
`define movsgpr        5'b00000 //Move the value to SGPR
`define mov            5'b00001 //Move the value to specified GPR

// Arithmetic Operations
`define add            5'b00010 //Add
`define sub            5'b00011 //Subtract
`define mul            5'b00100 //Multiply

// Logical Operations
`define ror            5'b00101 //OR 
`define rand           5'b00110 //AND
`define rxor           5'b00111 //XOR
`define rxnor          5'b01000 //XNOR
`define rnand          5'b01001 //NAND
`define rnor           5'b01010 //NOR
`define rnot           5'b01011 //NOT

// Load & Store Instructions
`define storereg       5'b01101 //Store content of register in data memory
`define storedin       5'b01110 //Store content of din bus in data memory
`define senddout       5'b01111 //Send data from data memory to dout bus
`define sendreg        5'b10001 //Send data from data memory to register

// Jump and Branch Instructions
`define jump           5'b10010 //Jump to address
`define jcarry         5'b10011 //Jump if carry flag is HIGH
`define jnocarry       5'b10100 //Jump if carry flag is LOW
`define jsign          5'b10101 //Jump if sign flag is HIGH
`define jnosign        5'b10110 //Jump if sign flag is LOW
`define jzero          5'b10111 //Jump if zero flag is HIGH
`define jnozero        5'b11000 //Jump if zero flag is LOW
`define joverflow      5'b11001 //Jump if overflow flag is HIGH
`define jnooverflow    5'b11010 //Jump if overflow flag is LOW

// Halt 
`define halt           5'b11011 //Pause the operation of the processor (will be in waiting state) -> Apply sys_rst to come out of the HALT state







module Processor(
    input clk, sys_rst,
    input [15:0] din,
    output reg [15:0] dout
);

// Adding program and data memory
reg [31:0] inst_mem [15:0]; //Instruction/Program Memory
reg [15:0] data_mem [15:0]; //Data Memory
reg [15:0] GPR [31:0];      //32 General Purpose Registers of size 16 bits
reg [15:0] SGPR;            //MSB of multiplication -> Special General Purpose Register
reg [31:0] IR;              //Instruction Register   <--ir[31:27]--><--ir[26:22]--><--ir[21:17]--><--ir[16]--><--ir[15:11]--><--ir[10:0]-->
                            //fields                 <---  oper  --><--   rdest --><--   rsrc1 --><--modesel--><--  rsrc2 --><--unused  -->             
                            //fields                 <---  oper  --><--   rdest --><--   rsrc1 --><--modesel--><--  immediate_date      -->    
reg [31:0] mul_res; //temporary ragister to store multiplication result
reg sign = 0, zero = 0, overflow = 0, carry = 0; // condition flags
reg [16:0] temp_sum; //temporary ragister to store addition result
reg jmp_flag = 0, stop = 0; //stores the value as high if there is a VALID jump, stop is used for halt instruction 



// Logic for Instructions
task decode_inst();
begin
    jmp_flag = 1'b0;
    stop = 1'b0;
    case(`oper_type)
        
////////////////////////////////////////////////////////////////////////////
//--------------------------------MOVE OPERATIONS---------------------------

        `movsgpr: GPR[`rdst] = SGPR;
        
////////////////////////////////////////////////////////////////////////////

        `mov: begin
            if (`imm_mode)
                GPR[`rdst] = `isrc;
            else
                GPR[`rdst] = GPR[`rsrc1];
        end
       
////////////////////////////////////////////////////////////////////////////
//----------------------------ARITHMETIC OPERATIONS-------------------------

        `add: begin
            if (`imm_mode)
                GPR[`rdst] = GPR[`rsrc1] + `isrc;
            else
                GPR[`rdst] = GPR[`rsrc1] + GPR[`rsrc2];
        end
        
////////////////////////////////////////////////////////////////////////////

        `sub: begin
            if (`imm_mode)
                GPR[`rdst] = GPR[`rsrc1] - `isrc;
            else
                GPR[`rdst] = GPR[`rsrc1] - GPR[`rsrc2];
        end

////////////////////////////////////////////////////////////////////////////

        `mul: begin
            if (`imm_mode)
                mul_res = GPR[`rsrc1] * `isrc;
            else
                mul_res = GPR[`rsrc1] * GPR[`rsrc2];
            GPR[`rdst] = mul_res[15:0];
            SGPR = mul_res[31:16];
        end
       
////////////////////////////////////////////////////////////////////////////
//------------------------------LOGICAL OPERATIONS--------------------------

        `ror: begin
            if (`imm_mode)
                GPR[`rdst] = GPR[`rsrc1] | `isrc;
            else
                GPR[`rdst] = GPR[`rsrc1] | GPR[`rsrc2];
        end
        
//////////////////////////////////////////////////////////////////////////// 
       
        `rand: begin
            if (`imm_mode)
                GPR[`rdst] = GPR[`rsrc1] & `isrc;
            else
                GPR[`rdst] = GPR[`rsrc1] & GPR[`rsrc2];
        end
        
//////////////////////////////////////////////////////////////////////////// 
       
        `rxor: begin
            if (`imm_mode)
                GPR[`rdst] = GPR[`rsrc1] ^ `isrc;
            else
                GPR[`rdst] = GPR[`rsrc1] ^ GPR[`rsrc2];
        end
        
////////////////////////////////////////////////////////////////////////////  
      
        `rxnor: begin
            if (`imm_mode)
                GPR[`rdst] = GPR[`rsrc1] ~^ `isrc;
            else
                GPR[`rdst] = GPR[`rsrc1] ~^ GPR[`rsrc2];
        end
        
////////////////////////////////////////////////////////////////////////////   
     
        `rnand: begin
            if (`imm_mode)
                GPR[`rdst] = ~(GPR[`rsrc1] & `isrc);
            else
                GPR[`rdst] = ~(GPR[`rsrc1] & GPR[`rsrc2]);
        end
        
////////////////////////////////////////////////////////////////////////////
        
        `rnor: begin
            if (`imm_mode)
                GPR[`rdst] = ~(GPR[`rsrc1] | `isrc);
            else
                GPR[`rdst] = ~(GPR[`rsrc1] | GPR[`rsrc2]);
        end
        
//////////////////////////////////////////////////////////////////////////// 
       
        `rnot: begin
            if (`imm_mode)
                GPR[`rdst] = ~(`isrc);
            else
                GPR[`rdst] = ~(GPR[`rsrc1]);
        end
        
////////////////////////////////////////////////////////////////////////////  
//---------------------------LOAD & STORE INSTRUCTIONS----------------------      

        `storedin: data_mem[`isrc] = din;
        `storereg: data_mem[`isrc] = GPR[`rsrc1];
        `senddout: dout = data_mem[`isrc];
        `sendreg : GPR[`rdst] = data_mem[`isrc];
        
////////////////////////////////////////////////////////////////////////////
//--------------------------JUMP & BRANCH INSTRUCTIONS----------------------   

        `jump       : jmp_flag = 1'b1;
        `jcarry     : jmp_flag = (carry == 1'b1);
        `jsign      : jmp_flag = (sign == 1'b1);
        `jzero      : jmp_flag = (zero == 1'b1);
        `joverflow  : jmp_flag = (overflow == 1'b1);
        `jnocarry   : jmp_flag = (carry == 1'b0);
        `jnosign    : jmp_flag = (sign == 1'b0);
        `jnozero    : jmp_flag = (zero == 1'b0);
        `jnooverflow: jmp_flag = (overflow == 1'b0);

////////////////////////////////////////////////////////////////////////////
//--------------------------HALT INSTRUCTION--------------------------------

        `halt: stop = 1'b1;
        
////////////////////////////////////////////////////////////////////////////

    endcase
end
endtask





// Logic for condition flags
task decode_condflag();
begin

////////////////////////////////////////////////////////////////////////////
//--------------------------------SIGN FLAG---------------------------------

    if (`oper_type == `mul)
        sign = SGPR[15];
    else
        sign = GPR[`rdst][15];

////////////////////////////////////////////////////////////////////////////
//-------------------------------CARRY FLAG---------------------------------

    if (`oper_type == `add) begin
        if (`imm_mode) begin
            temp_sum = GPR[`rsrc1] + `isrc;
            carry = temp_sum[16];
        end else begin
            temp_sum = GPR[`rsrc1] + GPR[`rsrc2];
            carry = temp_sum[16];
        end
    end else
        carry = 1'b0;

////////////////////////////////////////////////////////////////////////////
//--------------------------------ZERO FLAG---------------------------------

if(`oper_type == `mul)
 zero = ~((|SGPR[15:0]) | (|GPR[`rdst]));
else
 zero = ~(|GPR[`rdst]);

////////////////////////////////////////////////////////////////////////////
//----------------------------OVERFLOW FLAG---------------------------------

    if (`oper_type == `add) begin
        if (`imm_mode)
            overflow = (~GPR[`rsrc1][15] & ~IR[15] & GPR[`rdst][15]) | (GPR[`rsrc1][15] & IR[15] & ~GPR[`rdst][15]);
        else
            overflow = (~GPR[`rsrc1][15] & ~GPR[`rsrc2][15] & GPR[`rdst][15]) | (GPR[`rsrc1][15] & GPR[`rsrc2][15] & ~GPR[`rdst][15]);
    end else if (`oper_type == `sub) begin
       if (`imm_mode)
            overflow = (~GPR[`rsrc1][15] & IR[15] & GPR[`rdst][15]) | 
                        (GPR[`rsrc1][15] & ~IR[15] & ~GPR[`rdst][15]);
        else
            overflow = (~GPR[`rsrc1][15] & GPR[`rsrc2][15] & GPR[`rdst][15]) | 
                        (GPR[`rsrc1][15] & ~GPR[`rsrc2][15] & ~GPR[`rdst][15]);
    end else
        overflow = 1'b0;
end
endtask





// Reading program
initial begin
$readmemb("D:/Processor/inst_data.mem",inst_mem);
end

// Reading instructions one after another
reg [2:0] count = 0;
integer PC = 0; //Program Counter (stores address of next instruction)

// FSM states
parameter idle = 0, fetch_inst = 1, dec_exec_inst = 2, next_inst = 3, sense_halt = 4, delay_next_inst = 5;
//idle          : check reset state
//fetch_inst    : load instrcution from Program memory
//dec_exec_inst : execute instruction + update condition flag
//next_inst     : next instruction to be fetched
reg [2:0] state = idle, next_state = idle;
// FSM states
// Reset decoder
always@(posedge clk)
begin
 if(sys_rst)
   state <= idle;
 else
   state <= next_state; 
end
 
// Next state decoder + output decoder
always@(*)
begin
  case(state)
   idle: begin
     IR         = 32'h0;
     PC         = 0;
     next_state = fetch_inst;
   end
    
  fetch_inst: begin
    IR          =  inst_mem[PC];   
    next_state  = dec_exec_inst;
  end
  
  dec_exec_inst: begin
    decode_inst();
    decode_condflag();
    next_state  = delay_next_inst;   
  end
  
  delay_next_inst:begin
  if(count < 4)
       next_state  = delay_next_inst;       
     else
       next_state  = next_inst;
  end
  
  next_inst: begin
      next_state = sense_halt;
      if(jmp_flag == 1'b1)
        PC = `isrc;
      else
        PC = PC + 1;
  end
   
 sense_halt: begin
    if(stop == 1'b0)
      next_state = fetch_inst;
    else if(sys_rst == 1'b1)
      next_state = idle;
    else
      next_state = sense_halt;
 end
  
  default : next_state = idle; 
  endcase 
end

// Count update 
 
always@(posedge clk)
begin
case(state)
 idle : begin
    count <= 0;
 end
 fetch_inst: begin
   count <= 0;
 end 
 dec_exec_inst : begin
   count <= 0;    
 end  
 delay_next_inst: begin
   count  <= count + 1;
 end
  next_inst : begin
    count <= 0;
 end
  sense_halt : begin
    count <= 0;
 end
 default : count <= 0; 
endcase
end

endmodule
