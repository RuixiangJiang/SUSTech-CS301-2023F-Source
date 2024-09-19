**Catalogue**

[TOC]

## Lecture 1: Intruduction

### Embedded System

-   An embedded system is an application that contains at least one programmable computer.
-   Embedded systems are information processing systems that are embedded into an enclosing product.
-   An embedded system may or may not have OS.

<img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240112115050024.png" alt="image-20240112115050024" style="zoom:10%;" />

### Value Notations (32 bit processor)

<img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240112115428126.png" alt="image-20240112115428126" style="zoom:20%;" />

### Little Endian vs Big Endian

-   Little endian: LSB of a word is at the least memory address.
-   Big endian: MSB of a word is at the least memory address.

<img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240112165112110.png" alt="image-20240112165112110" style="zoom:15%;" />



------



## Lecture 2: STM32 MCU & GPIO

### CPU

<img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240112114439918.png" alt="image-20240112114439918" style="zoom:10%;" />

-   ALU
    -   operands (stored in registers and data memory)
    -   operation
    -   flag (stored in Program Status Register): **Z**ero, **N**egative, o**V**erflow, **C**arry
    -   result
-   Register File
    -   low register (r0 - r7): can be accessed by any instruction
    -   high register (r8 - r12): can only be accessed by some instructions
    -   stack pointer (r13)
    -   link register (r14): stores the return address for function calls
    -   program counter (r15): memory address of to be executed instruction
    -   program status register
-   Control Unit

### GPIO

-   Processor can directly access processor registers; accesses peripheral registers via memory
    mapped I/O.

-   Each register has a specific memory address, Register Mapping assigned a name to each register address.

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240112133207873.png" alt="image-20240112133207873" style="zoom:10%;" />

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240112133343256.png" alt="image-20240112133343256" style="zoom:10%;" />

-   GPIO Mode

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240112133458794.png" alt="image-20240112133458794" style="zoom:10%;" />

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240112133835113.png" alt="image-20240112133835113" style="zoom:20%;" />

    -   default: floating input
    -   input: pull-up/pull-down
    -   output: push-pull

-   Programming GPIO
    -   configure the GPIO mode: set CRL/CRH to configure input/output mode
    -   set the output status: set ODR to configure output status
    -   read the input status: set ODR to configure input with pull-up/pull-down; read from IDR to get input status

-   GPIO Registers (each group GPIO ports has 7 registers)

    -   CRL/CRH (32-bit configuration register): every 4 bits combine to represent a pin

        <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240112135512526.png" alt="image-20240112135512526" style="zoom:20%;" />

        common settings for each pin (default = 0x4):

        -   0x3 = 0011: CNF = 00 (pushpull), MODE = 11 (output)
        -   0x4 = 0100: CNF = 01 (Hiz), MODE = 00 (input)
        -   0x8 = 1000: CNF = 10 (pull-up/pull-down), MODE = 00 (input)

        example:

        ```objective-c
        GPIOC -> CRH &= 0xFFF00FFF; // clear settings of PC11 and PC12
        GPIOC -> CRH |= 0x00038000; // set PC11, PC12 as input and output, respectively
        GPIOC -> ODR |= 1 << 11; // set PC11 as input with pull-up
        ```

    -   IDR/ODR (32-bit input/output data register): the high 16 bits are reserved, each one of the low 16 bits represents a pin

        <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240112151813656.png" alt="image-20240112151813656" style="zoom:25%;" />

        

        <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240112151840753.png" alt="image-20240112151840753" style="zoom:25%;" />

        for LED, set ODR bit to **low** to **turn on** the LED.

        for KEY0/KEY1, set **pull-up** input mode.

        for KEY-WK, set **pull-down** input mode.
    
    -   BSRR (32-bit set/reset register)
        
        <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240112151937234.png" alt="image-20240112151937234" style="zoom:25%;" />
        
    -   BRR (32-bit reset register)
        
        <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240112152342517.png" alt="image-20240112152342517" style="zoom:25%;" />
    	
    -   LCKR (32-bit locking register)
        
        <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240112152634763.png" alt="image-20240112152634763" style="zoom:35%;" />

------



## Lecture 3: ARM Assembly

### Flags

-   Numbers are stored in 2's complement representation.

-   CPU doesn't know whether the number is signed or unsigned.

-   Condition flags in PSR:

    -   **N**egative: N = 1 if most significant bit of result (do not consider the potential carry bit) is 1

    -   **Z**ero: Z = 1 if all bits of result (do not consider the potential carry bit) are 0
    -   **C**arry (treat as unsigned numbers):
        -   for unsigned additoin, C = 1 if carry takes place
        -   for unsigned subtraction, C = 0 if borrow takes place
        -   for shift/rotation, C = last bit shifted out
    -   o**V**erflow (treat as signed numbers):
        -   V = 1 if adding two same-signed numbers produces a result with  the opposite sign
        -   non-arithmetic operations doesn't touch V bit

-   Most instructions update NZCV flags only if 'S' suffix is present (e.g. ADD - unchanged, ADDS - changed), while some instructions always update them (e.g. CMP).

### Logic Instructions

-   ```AND r0, r1, r2```: bitwise and, ```r0 = r1 and r2```
-   ```ORR r0, r1, r2```: bitwise or, ```r0 = r1 or r2```
-   ```EOR r0, r1, r2```: bitwise exclusive or, ```r0 = r1 xor r2```
-   ```BIC r0, r1, r2```: bit clear, ```r0 = r1 and ~r2```
-   ```LSL r1, r2, num```: logical shift left, ```r1 = r2 << num```
-   ```LSR r1, r2, num```: logical shift right, ```r1 = r2 >> num```
-   ```ROR r1, r1, num```: rotate right

### Addressing Mode

-   Immediate addressing
-   Register addressing
-   Register indirect addressing (indexed): base + offset
    -   ```LDR r1, [r2, num]``` (pre-index): load r2[num] to r1
    -   ```LDR r1, [r2, num]!``` (pre-index): r2 += num, and then load r2[0] to r1
    -   ```LDR r1, [r2], num``` (post-index): load r2[num] to r1, and then r2 += num

------



## Lecture 4: Embedded C

### Operation

-   Bit manipulation

    -   &: clear a bit
    -   |: set a bit
    -   ^: toggle a bit
    -   shift: mul/div

-   if-else/switch-case

    switch-case runs faster, because it will generate a forwarding table, in order not to go through all the branches.

-   macro

    -   preprocessor is executed before the compilation, for file inclusion, macro substitution, conditional compilation, and so on
    -   macro is a fragment of code as a replacement in the source code

### Data Type

-   pointer
-   array
-   string
-   struct
-   union: members occupy the same memory space

### Storage

-   static variable

    -   declared within body of a function: maintain the value between function calls, i.e., when calling a function secondly, the value is the same as which at the end of the first-called function
    -   declared within a module but outside the body of any function: accessible by all functions within that module; independent with variables in other modules, even if they have the same name

-   volatile variable

    the value may change outside the normal program flow, possibly, via an interrupt service routine or a hardware action

-   malloc/free

    costly in time and space

------



## Lecture 5: Interrupt

### Subroutine

-   call a subroutine: BL (branch and link)

    ```c++
    LR = PC + 4 // save the return address in the link register (LR)
    PC = label
    ```

-   back to caller: BX (branch with exchange)

    ```C++
    PC = LR
    ```

-   stack

    -   descending/ascending

        <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240112231648136.png" alt="image-20240112231648136" style="zoom:15%;" />

    -   full/empty

        <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240112231800723.png" alt="image-20240112231800723" style="zoom:15%;" />

    -   Cortex-M uses full descending stack

### Interrupt

-   Process
    -   finish processing current instruction
    -   save return address and register context to stack
    -   run interrupt service routine
    -   restore return address and register context from stack
    -   resume main program

-   Each interrupt has an ISR (Interrupt Service Routine)
-   As for Cortex-M, an interrupt vector table in memory contains addresses of ISR
    -   table elements located at fixed address
    -   table elements store function pointers (address of ISR)

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240113171350570.png" alt="image-20240113171350570" style="zoom:10%;" />

-   Interrupts are managed by Nested Vectored Interrupt Controller (NVIC)

    -   AIRCR register: set priority group
    -   IPRx register: set priority value
        -   smaller value means higher priority
        -   fixed priority for Reset, HardFault and NMI; adjustable for the other interrupts

    -   ISER/ICER registers: enable/disable interrupts

-   Priority

    -   preempt priority (first)
    -   sub priority (second)
    -   natural priority (third)

-   Principle

    -   higher preempt priority can interrupt ongoing lower preempt priority
    -   same preempt priority
        -   higher sub priority can not interrupt ongoing lower sub priority
        -   when two interrupts occur at the same time, the one with higher sub priority is executed first
        -   same sub priority: depend on which interrupt occurs first

-   BL vs Interrupt

<img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240113172441509.png" alt="image-20240113172441509" style="zoom:15%;" />

------



## Lecture 6: Serial Communication - UART

### Communication Interfaces

-   UART/USART
-   SSI/SPI
-   I2C
-   USB
-   Ethernet
-   CAN

### Serial/Parallel

<img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240113173818036.png" alt="image-20240113173818036" style="zoom:15%;" />

### Synchronous/Asynchronous

-   Synchronous Communication (e.g. I2C, SPI, USART)
    -   shares the same clock signal which is sent along with data
    -   the device that generates the clock is called the master and other devices are slaves
-   Asynchronous Communication (e.g. UART, USART)
    -   sender provides no clock signal to receiver
    -   relies on synchronous signals like start and stop bits within the data signal

### Direction

-   simplex: data can only be transmitted in one direction always
-   half-duplex: data can be transmitted in both directions but need time division (e.g. I2C)
-   full-duplex: data can be simultaneously transmitted in both directions (e.g. UART, SPI)

### UART

-   frame format

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240113174940789.png" alt="image-20240113174940789" style="zoom:15%;" />

    -   ST: start bit = 0
    -   SP: stop bit =1 to make sure that the next start bit makes a falling edge

-   transmission speed of actual data = baud rate / number of bits in each frame

-   synchronization of two ends: each clock of the end should be accurate to within about ±2.5%

-   bit level reorganization: oversampling with receiver sample clock being 16x faster than baud rate

    <img src="https://tse4-mm.cn.bing.net/th/id/OIP-C.kQjSqI0BT3PgKB6cu4_DywAAAA?rs=1&pid=ImgDetMain" alt="See the source image" style="zoom:70%;" />

    -   at START, discover 1 -> 0
    -   after 8 clock cycles, reach the first MIDBIT, do sampling (and find that the first 8 bits are 0, which should be a start signal)
    -   after 16 clock cycles, reach the second MIDBIT, do sampling (the first data bit of frame)

-   error detection: add a parity bit

-   RS232 defines the electrical and mechanical characteristics

-   STM32 USART

    Baud rate register (USART_BRR): the baud rate for the receiver and transmitter (Rx and Tx) are both set to the same value as programmed in the Mantissa and Fraction values of USARTDIV
    $baud = \frac {f_{USART\ input\ clk}}{16\times USARTDIV},USARTDIV = DIV\_Mantissa + \frac{DIV\_Fraction}{16}$

------



## Lecture 7 & 8: Timer

### Clock Tree

-   clock source

    -   Quartz Crystal Oscillators
        -   high accuracy
        -   stable frequency
    -   RC/LC/RLC oscillators
        -   simplicity
        -   low cost
    -   PLL

-   STM32F103 clock config

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240113201223729.png" alt="image-20240113201223729" style="zoom:20%;" />

### STM32 Timers

-   basic block diagram

    -   prescaler: divide counter clock frequency by any factor between 1~65536
    -   counter: counts from 0 to the value stored in ARR register, then restarts from 0 and generates an overflow event

-   timer register

    -   TIMx_CNT (16-bit counter)

    -   TIMx_ARR (16-bit auto-reload register)

    -   TIMx_PSC (16-bit prescaler value)

        $f_{CK\_CNT} = \frac{f_{CK\_PSC}}{TIMx\_PSC+1}$, $T=\frac{TIMx\_ARR+1}{f_{CK\_CNT}}=\frac{(TIMx\_ARR+1)\times(TIMx\_PSC+1)}{f_{CK\_PSC}}$, $f_{CK\_OV}=\frac{1}{T}$

    -   TIMx_CR1 (16-bit control register, but the most significant 6 bits are reserved)

    -   TIMx_SR (16-bit status register, but totally 6 bits are reserved)

    -   TIMx_DIER (16-bit DMA/Interrupt enable register, but totally 4 bits are reserved)

-   counting mode

    -   counting up
    -   counting down
    -   counting up and down

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240113204159723.png" alt="image-20240113204159723" style="zoom:15%;" />

### Output Compare

<img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240113205232508.png" alt="image-20240113205232508" style="zoom:15%;" />

<img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240113205308771.png" alt="image-20240113205308771" style="zoom:15%;" />

UIF: update interrupt flag

CCnIF: capture & compare channel n interrupt flag

-   set mode (OCnM = 001)

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240113205415836.png" alt="image-20240113205415836" style="zoom:15%;" />

-   clear mode (OCnM = 010)

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240113205618491.png" alt="image-20240113205618491" style="zoom:15%;" />

-   toggle mode (OCnM = 011)

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240113205647768.png" alt="image-20240113205647768" style="zoom:15%;" />

### PWM

-   $frequency = 1/T_s=\frac{f_{CK\_PSC}}{(TIMx\_ARR+1)\times(TIMx\_PSC+1)}$

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240113210231882.png" alt="image-20240113210231882" style="zoom:10%;" />

-   $duty\ cycle = T_{on}/T_s=T_{on}/(T_{on}+T_{off})$

    $duty\ cycle=\frac{TIMx\_CCR}{TIMx\_ARR+1},\ PW1\ mode\\=1-\frac{TIMx\_CCR}{TIMx\_ARR+1},\ PW2\ mode$

-   mode

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240113210316075.png" alt="image-20240113210316075" style="zoom:10%;" />

-   $period=(1+TIMx\_ARR)\times T_s$

### Time Span

<img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240113211947815.png" alt="image-20240113211947815" style="zoom:20%;" />

### System Timer (SysTick)

-   internel interrupt

-   24-bit downcounter from RELOAD value to 0
-   SysTick_LOAD: SysTick reload value register (similar to TIMx_ARR)
-   SysTick_VAL: SysTick current value register, has random value on reset (similar to TIMx_CNT)
-   time interval between two SysTick interrupts = (RELOAD + 1)  * SysTick_Clock_Period

------



## Lecture 9: I2C and SPI

### I2C (Inter Integrated Circuit)

-   characteristics
    -   inter integrated circuit
    -   two bidirectional open-drain lines, plus ground (Serial Data Line + Serial Clock Line)
    -   serial
    -   half-duplex
    -   synchrnous
    -   byte-oriented, MSB first, (8 bits + 1 ack bit) per data frame
    -   multi-master, multi-slave
    -   no global master to generate clock, i.e., each master drives both SCL and SDL

-   communication steps

    -   master sends start condition: SDA changes from 1 to 0 when SCL = 1

    -   master sends address of the slave and the direction (is also data), and then receive an ack bit

        -   7-bit address

            each device listens to address and switches state if its address matches

        -   1-bit direction (0: master writes data; 1: master receives data)

    -   slave send ack to response master

    -   repeat:

        -   transmitter sends a byte of data which will be put on SDA

            when SCL = 0, transmitter sends a bit of data onto SDA (SDA allowed to change)

            when SCL = 1, receiver reads a bit of data from SDA (SDA must remain stable)

        -   receiver sends ack (ACK - pull down; NACK - leave high)

        <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240113221053445.png" alt="image-20240113221053445" style="zoom:10%;" />

    -   master sends stop condition: SDA changes from 0 to 1 when SCL = 1

-   master vs slave
    -   each device can be master or slave
    -   master should do
        -   begins the communication
        -   chooses the slave
        -   makes the clock
        -   sends/receives data
    -   slave should do
        -   has a unique 7-bit address
        -   responds to the master

-   working modes

    blue blocks: data transmitted from the master to the slave

    white blocks: data transmitted from the slave to the master

    -   master send data to slave

        <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240113222201837.png" alt="image-20240113222201837" style="zoom:15%;" />

    -   slave sends data to master

        <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240113222234322.png" alt="image-20240113222234322" style="zoom:15%;" />

    -   repeated start

        <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240113222351180.png" alt="image-20240113222351180" style="zoom:15%;" />

### SPI (Serial Peripheral Interface)

-   characteristics

    -   serial
    -   synchronous
    -   full-duplex
    -   single-master, multi-slave
    -   no start/stop or slave ack, master sets corresponding slave select (SS) signal instead

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240113224700074.png" alt="image-20240113224700074" style="zoom:10%;" />

-   SPI timing diagram
    -   MSB first (but can be set to LSB first)
    -   transmit data at rising/falling edge of SPICLK; the same bit data is read at the following falling/rising edge

	<img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240113225836259.png" alt="image-20240113225836259" style="zoom:10%;" />

	-   if the master only write to the slave, it can ignore data received from the slave

	    if the master needs to read from the slave, it should send an empty data to trigger the slave to send data
	
	<img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240113225053654.png" alt="image-20240113225053654" style="zoom:10%;" />

-   SPI clock phase (CPHA) & polarity (CPOL)

    -   polarity: whether the clk signal is high or low when it is idle

        -   0: sclk = 0 when idle

        -   1: sclk = 1 when idle

    -   phase: whether the data is sampled on the rising or falling edge of clk

        -   0: sample at 1st edge
        -   1: sample at 2nd edge

    -   CPOL and CPHA together determine the clock edge for transmitting and receiving

    -   SCLK is pushed to low during idle when CPOL = 0, otherwise pulled to high during idle

        <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240113230448213.png" alt="image-20240113230448213" style="zoom:15%;" />

        <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240113231822477.png" alt="image-20240113231822477" style="zoom:10%;" />

### I2C vs SPI

-   commonality
    -   both serial and synchronous
    -   both for short distance communication
    -   similar application scenarios
    -   both in master-slave configuration
-   difference
    -   I2C by Philips while SPI by Motorola
    -   I2C is half-duplex while SPI is full-duplex
    -   I2C uses ack but SPI doesn't
    -   I2C addresses 
    -   devices by broadcasting slave addresses onto the bus while SPI addresses devices by sending an chip select signal to the corresponding slave
    -   I2C has fixed clock polarity and clock phase while SPI allows adjustable clock polarity and clock phase

------



## Lecture 10: Bus

### Definition

-   a shared communication link between subsystems
-   multiple components can connect to the bus and exchange data through set of wires
-   each wire of a bus can contain a single binary digit at any time
-   at a given time slot, only one component can send data to the bus

### Classification

-   address bus (unidirectional)

    CPU accesses memory by addressing a unique location

-   data bus (bidirectional)

    carry data between src and dst

-   control bus (bidirectional)

    containing signal requests and acks which indicate what type of information is on the data lines

<img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114125843944.png" alt="image-20240114125843944" style="zoom:10%;" />

### Performance

-   width: the max number of data bits transmitted simultaneously (e.g. 32-bit)
-   bandwidth (i.e., transmission rate): the max amount of data transmitted on a bus within a specific time (e.g. MB/s)
-   frequency: speed at which the bus operates

-   bandwidth = width * frequency

    note that: width is represented in bit but not byte; the frequency is not the bus clk frequency

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114133810966.png" alt="image-20240114133810966" style="zoom:15%;" />

### Architecture

-   single level bus

    -   one common bus is used to communicate between peripherals and microprocessors
    -   easy to add or remove IO devices
    -   conflicts 
    -   occur when multiple components need to use the bus, and priority needs to be determined
    -   not support burst transfer (which refers to the consecutive transfer of multiple data elements, initiated by a single address, i.e., the data flow looks like: address, data, data, data, ...)

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114132829475.png" alt="image-20240114132829475" style="zoom:10%;" />

-   two level bus

    -   CPU centralized: easy to add or remove IO devices

        <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114133038395.png" alt="image-20240114133038395" style="zoom:10%;" />

    -   Memory centralized: Memory bus (M-bus) operates at a high speed, reducing the burden on the system bus

        <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114133112566.png" alt="image-20240114133112566" style="zoom:10%;" />

-   multi-level bus

    -   Memory centralized with DMA: information exchange between IO devices and main memory doesn't require CPU intervention

        <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114133330441.png" alt="image-20240114133330441" style="zoom:10%;" />

    -   more advanced: allowing various components to connect efficiently while maintaining high performance and data integrity

        <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114133427602.png" alt="image-20240114133427602" style="zoom:10%;" />

### Timing

-   bus transaction step

    -   request: a device that wishes to initiate a transaction on the bus sends a request signal to the bus controller or arbiter
    -   addressing: master sends address and control signal to the bus
    -   transfer: master and slave perform data trainsmission
    -   complete: master removes relevant information from the bus and release the control from the bus

-   coordination of events on bus

    -   synchronous: controlled by a clock

        e.g. CPU (Master) reads/writes data from/to memory (Slave)

        <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114135432317.png" alt="image-20240114135432317" style="zoom:15%;" />

    -   asynchronous: handled by well-defined specifications, i.e., a response is delivered within a specified time after a request

        e.g. CPU (master) reads data from memory (slave)

        <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114135552684.png" alt="image-20240114135552684" style="zoom:15%;" />

        e.g. CPU (master) writes data to memory (slave)

        <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114135646827.png" alt="image-20240114135646827" style="zoom:15%;" />

### Arbitration

-   chooose a mster among multiple requests

-   centralized: single hardware device controls bus access, which may be part of CPU or separate

    -   serial arbitration (daisy chain): devices connected in a chain sequentially select the bus controller

        -   all bus master use the same line for bus request
        -   the bus controller gives the bus grant signal if the bus busy line is inactive
        -   bus grant signal is propagated serially through all masters starting from nearest one

        <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114141028500.png" alt="image-20240114141028500" style="zoom:15%;" />

    -   parallel arbitration (independent request): devices independently request bus access

        -   all bus masters have individual request and grant lines
        -   the bus controller knows which master has requested so bus is granted to that master
        -   priorities of the masters are predefined so on simultaneous bus requests, the bus is granted based on the priority

        <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114141430973.png" alt="image-20240114141430973" style="zoom:20%;" />

-   distributed: there are control logics (arbiters) on all modules and each module may claim the bus

    -   the bus device arbiter sends its own arbitration number onto the shared bus, compares the  arbitration number received from bus with its own number to determine priority, and withdraws its arbitration number if its own priority is lower
    -   finally the highest-priority arbiter remaining on the bus gains control of the bus
    -   one device failure won't affect others

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114141909833.png" alt="image-20240114141909833" style="zoom:10%;" />

### SoC AMBA bus

-   advantage

    -   enable IP reuse
    -   flexibility
    -   compatibility
    -   well-supported

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114142541569.png" alt="image-20240114142541569" style="zoom:18%;" />

-   AHB (Advanced High-performance Bus)
    -   high performance
    -   connects CPUs, High-Speed Memory, DMA Controllers
    -   high bandwidth data transfer
    -   multiple masters supported
    -   ideal for high-performance components
-   APB (Advanced Peripheral Bus)
    -   low power
    -   connects low-speed peripheral devices to system bus
    -   suitable for Timers, GPIO Controllers, UART, etc.

------



## Lecture 11: DMA

### Transfer Direction

-   external device to memory
-   memory to external device
-   memory to memory
-   external device to another external device (potentially)

no continuous intervention by the processor, use DMAC (an peripheral control unit of DMA) to move data

<img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114151240642.png" alt="image-20240114151240642" style="zoom:10%;" />

### Configuration

-   single bus, detached DMA controller

    -   each transfer uses bus twice (IO to DMAC and then DMAC to memory)
    -   CPU is suspended twice

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114151810490.png" alt="image-20240114151810490" style="zoom:10%;" />

-   single bus, integrated DMA controller

    -   controller may support more than one device
    -   each transfer uses bus once (DMAC to memory)
    -   CPU is suspended once

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114151923454.png" alt="image-20240114151923454" style="zoom:10%;" />

-   separate IO bus

    -   bus supports all DMA enabled devices
    -   each transfer uses bus once (DMAC to memory)
    -   CPU is suspended once

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114152105042.png" alt="image-20240114152105042" style="zoom:10%;" />

-   separate DMA bus

    -   without CPU suspended
    -   conflict when both CPU and DMA access memory

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114152312769.png" alt="image-20240114152312769" style="zoom:10%;" />

### Transfer Mode

-   burst mode

    entire block of data transferred in one continuous operation, during which CPU remains inactive, ensuring high efficiency

-   interleaving mode

    data transferred only when there is no conflict with CPU for system bus or memory access, ensuring no interference

-   cycle stealing mode

    firstly, DMA controller takes control; once one word of data transferred, DMAC releases control back to the CPU, and then repeat this process, ensuring that CPU can continue exexuting its instructions between data transfers

### DMAC Registers

-   address register: contains the address to specify the desired location in memory
-   word count register: contains the number of words to be transferred
-   control register: specifies the transfer mode

<img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114153612313.png" alt="image-20240114153612313" style="zoom:18%;" />

### Operation

-   interaction with CPU
    -   CPU sets up registers configuration
        -   data transfer direction between memory and IO (Read/Write)
        -   IO devices address
        -   starting address of memory block for data
        -   amount of data to be transferred
    -   CPU carries on with other work
    -   DMAC deals with transfer
    -   DMAC send interrupt when finished
-   interaction with peripheral
    -   when each word of data is available, the IO device places a signal on the DMA request wire
    -   the signal causes DMAC to seize the memory bus, in order to place the desired address on the memory address wire and a signal on the DMA acknowledge wire
    -   when IO device receives the DMA acknowledge signal, the transfer between external device and DMA starts

### Sequence

-   I/O device sends a DMA request to DMAC
-   DMAC sends a bus request signal to CPU for memory access
-   CPU sends bus grant signal to DMAC, and release the bus control authority to DMAC
-   DMAC sends acknowledgement signal to I/ O device
-   DMAC transfers data between I/O device and Memory according to preset address and word count
-   After completing of transfer, DMAC cancels the bus request signal
-   DMAC sends interrupt signal to CPU if necessary

<img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114155113312.png" alt="image-20240114155113312" style="zoom:15%;" />

------



## Lecture 12: Embedded Storage Management

### Type

-   volatile: loses its content when power-off
    -   SRAM (Static Random Access Memory)
        -   low density
        -   high power
        -   expensive but fast
        -   content last until power-off
        -   often used for cache
    -   DRAM (Dynamic Random Access Memory)
        -   high density
        -   low power
        -   cheap but slow
        -   refresh regularly
        -   often used for main memory
-   non-volatile: remain its content even when power-off
    -   ROM (Read-Only Memory)
        -   permanent, pre-programmed data
        -   retains information across power cycles
    -   EPROM/Flash (Erasable Programmable ROM)
        -   rewritable
        -   often used used for firmware, configuration and data storage
        -   EPROM can be erased using UV light, while EEPROM can be erased electrically

​				Note that erasing means changing from 0 to 1, while writing means from 1 to 0

<img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114170728730.png" alt="image-20240114170728730" style="zoom:10%;" />

### Flash

-   sector
    -   refers to a fixed-size, contiguous block of storage
    -   the smallest addressable unit for reading/writing data and the smallest erasable unit

-   interface: SPI
-   application
    -   SD Card
    -   USB Flash Drive
    -   SSD

### SD Card

-   interface

    -   SD mode
        -   clk
        -   4 data lines
    -   SPI mode
        -   clk
        -   card select
        -   2 data lines

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114172916472.png" alt="image-20240114172916472" style="zoom:10%;" />

-   driver: Read, Write, Initial, GetStatus

### FAT File System

-   type

    -   FAT
    -   NTFS
    -   ext2, ext3, ext4
    -   CDFS

-   cluster

    -   1 cluster = $2^n$ sectors (e.g. FAT 12: 12-bit cluster address)
    -   $File\ System\ Size = num_{clusters}\times size_{cluster}=2^{cluster\ address\ width}\times size_{cluster}$

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114181147609.png" alt="image-20240114181147609" style="zoom:10%;" />

------



## Lecture 13: ADC

### Sampling

-   rate: how often analog signal is measured (samples per second, Hz)

-   resolution: number of bits to represent each sample (bit depth, bit)

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114214950229.png" alt="image-20240114214950229" style="zoom:10%;" />

### Output

for a n-bit ADC:

-   $ numOfSteps = 2^n, stepSize = \frac{V_{ref}}{numOfSteps} $
-   $Output = round(\frac{V_{input}}{stepSize})=round(2^n \times \frac{V_{input}}{V_{ref}})$

### Architecture

-   Successive-approximation ADC (SAR): use binary search to determine the closest digital representation of the input signal
-   Flash ADC: directly convert an analog input signal into a digital output by using a set of comparators to rapidly compare the input against multiple reference voltages

<img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114220656457.png" alt="image-20240114220656457" style="zoom:10%;" />

### STM32 ADC

-   12-bit
-   16-channel regular grpup and 4-channel injected group (higher priority)
-   single conversion mode

<img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114221147045.png" alt="image-20240114221147045" style="zoom:10%;" />

-   continuous conversion mode

<img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114221216584.png" alt="image-20240114221216584" style="zoom:10%;" />

-   continued scan mode

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114221403969.png" alt="image-20240114221403969" style="zoom:10%;" />

-   discontinuous mode

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114221437547.png" alt="image-20240114221437547" style="zoom:10%;" />

-   $t_{ADC}=t_{sampling}+t_{conversion}$
    -   $t_{sampling}$ is defined in ADCx_SMPR register (1.5 ~ 239.5)
    -   $t_{conversion}=12.5\times t_{ADC\ clk\ cycle}$

------



## Lecture 14: Arithmetic

### Barrel Shifter

-   LSL (logical shift left): bits are shifted to the right and the new bits added in at the left hand side are 0
-   LSR (logical shift right): bits are shifted to the left and the new bits added in at the right hand side are 0
-   ASR (arithmetic shift right): bits are shifted to the right and the new bits added in at the left hand side are 0 for positive value or 1 for negative value
-   ROR (rotate right): bits are rotated rightwards so that the bits shifted out at the right hand side reappear at the left hand side
-   RRX (rotate extended): bits are shifted right one place only and the carry flag is shifted into the new most significant bit. The least significant bit is shifted into the carry flag only if the mnemonic specifies an S

-   shifts with other instruction

    -   ```mov r0, r0, LSL #n```: $r0 = r0 * 2^n$

    -   ```mov r0, r0, LSR #n```: $r0 = r0 / 2^n$ (unsigned)

    -   ```mov r0, r0, ASR #n```: $r0 = r0 / 2^n$ (signed)

    -   ```mov r0, r0, ROR #n```: swap the first 32-n bits and the other n bits

    -   ```add r0, r0, r0, LSL #n```: $r0=r0+r0*2^n=r0*(1+2^n)$

    -   ```rsb r0, r0, r0, LSL #n```: $r0=r0*2^n-r0=r0*(2^n-1)$

### ALU Adder

-   ripple carry adder

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240114234241563.png" alt="image-20240114234241563" style="zoom:20%;" />

    each cell causes a propagation delay

-   lookahead carry adder

    Noticing $c_{i+1}$ is 1 IFF at least two bits among $x_i,\ y_i$ and $c_i$ are 1, we derive that $c_{i+1}=x_iy_i+y_ic_i+c_ix_i=x_iy_i+c_i(x_i\oplus y_i)$. What's more, $s_i=x_i\oplus y_i\oplus c_{i-1}$, so we can define $g_i=x_iy_i,\ p_i=x_i\oplus y_i$, and then $c_{i+1}=g_i+p_ic_i=g_i+p_i(g_{i-1}+p_{i-1}c_{i-1})=...$ till $c_0$ occurs.

    <img src="https://pic4.zhimg.com/v2-59fced804216b1c5896af68e5131570b_r.jpg" alt="img" style="zoom:50%;" />

    <img src="https://pic2.zhimg.com/80/v2-840aa3c8ea77faa742dd67643a29298d_1440w.webp" alt="img" style="zoom:35%;" />

-   select carry adder

    When calculating $s_i=x_i+y_i+c_i$, even if $c_i$ is unknown till now, we still know that $c_i$ must be 0 or 1. We can use a adder which produces two results, $x_i+y_i$ and $x_i+y_i+1$, which should be selected referring to $c_i$ when it is known.

-   propagation delays comparison

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240115001523638.png" alt="image-20240115001523638" style="zoom:10%;" />

### Multiplier

-   matrix multiplier

    -   Add each partial product into a total as it is formed, with carry save adder involved. Carry-save addition passes (saves) the carries to the output, rather than propagating them. With this technique, we can avoid carry propagation until final addition.

    -   When adding sets of numbers, carry-save can be used on all but the final sum. Standard adder (carry propagate) is used for final sum.

    -   Carry-save is fast (no carry propagation) and inexpensive (full adders)

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240115002552027.png" alt="image-20240115002552027" style="zoom:15%;" />

-   sequential multiplier

    In each step, one bit of the multiplier is selected. If the bit is logic 1, the multiplicand is shifted left to form a partial product, and it’s added to the partial sum.

    Only for unsigned multiplication. Sign bits are evaluated separately.

    Do a n-loop, for each loop, set ```AC = Q[0] == 1 ? AC + M : AC```, and then logically shift right {AC, Q}.

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240115003505306.png" alt="image-20240115003505306" style="zoom:15%;" />

-   Booth's multiplication algorithm

    Use +1 and -1 to replace continuous 1s, e.g., use ```0 +1 0 0 0 -1 0``` to represent ```30 = 0011110 = 0100000 - 0000010```.

    Can perform negative number multiplication. Sometimes worse than normal algorithm.

    Do a n-loop, for each loop, set ```{Q[0]Q[-1]} == 01: AC += M; else {Q[0]Q[-1]} == 10: AC += M' + 1; else AC = AC```, and then arithmetically shift right {AC, Q[ : -1]}.

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240115004800326.png" alt="image-20240115004800326" style="zoom:15%;" />

-   Booth2 multiplication algorithm

    <img src="/Users/jrx/Library/Application Support/typora-user-images/image-20240115010201169.png" alt="image-20240115010201169" style="zoom:15%;" />

------



Editted by Ruixiang Jiang.