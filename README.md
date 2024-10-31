# Single cycle RISC-V
### Single Cycle RISC-V Processor Description
A single-cycle RISC-V processor executes each instruction in a single clock cycle. This simplified processor design includes the following key components:<br/>
1.Program Counter (PC):<br/>
Function: Holds the address of the current instruction. It increments by 4 to point to the next instruction or updates to a new address in case of a branch.<br/>
2.Instruction Memory:<br/>
Function: Stores the program's instructions. It outputs the instruction at the address specified by the PC.<br/>
3.Register File:<br/>
Function: Contains 32 registers for storing intermediate values. It supports reading two registers and writing to one register per clock cycle.<br/>
4.ALU (Arithmetic Logic Unit):<br/>
Function: Performs arithmetic and logical operations based on control signals. It takes two input operands and produces an output result.<br/>
5.Data Memory:<br/>
Function: Stores and retrieves data required by load and store instructions. It supports read and write operations based on control signals.<br/>
6.Control Unit:<br/>
Function: Generates control signals based on the instruction opcode to coordinate the operation of other components. It determines the type of instruction and controls the data paths accordingly.<br/>
<br/>
## Instruction Execution
In a single clock cycle, the following sequence occurs:<br/>
1.The PC provides the address to the instruction memory.<br/>
2.The instruction memory outputs the instruction.<br/>
3.The instruction is decoded, and control signals are generated.<br/>
4.Register operands are read from the register file.<br/>
5.Immediate values are generated if needed.<br/>
6.The ALU performs the specified operation.<br/>
7.The result is written back to the register file or memory.<br/>
8.The PC is updated for the next instruction.<br/>
