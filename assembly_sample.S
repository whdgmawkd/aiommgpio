.section .init
.globl _start
_start:

.equ PWMCTL, 0x2020c000
.equ PWMSTA, 0x2020c004
.equ PWMDMAC, 0x2020c008
.equ PWMRNG1, 0x2020c010
.equ PWMDAT1, 0x2020c014
.equ PWMFIF1, 0x2020c018
.equ PWMRNG2, 0x2020c020
.equ PWMDAT2, 0x2020c024

.equ CM_PWMCTL, 0x201010a0
.equ CM_PWMDIV, 0x201010a4
.equ PASSWD, 0x5a000000

ldr r0,=0x20200000 // This command loads the physical address of the GPIO region into r0.  

// CLOCK MASTER SETUP
ldr r1, =CM_PWMCTL
ldr r3, =PASSWD
ldr r2, =0x00FFFFEF
ldr r5, [r1]
AND r2, r2, r5   // now r2 has whatever was in CM_PWMCTL but the first byte and bit 4 is 0
ORR r2, r2, r3     // OR in the password 
str r2, [r1]       // request to stop the clock

wait1:
 ldr r5, [r1]      // load the contents of the clock master pwm control register into r5
 AND r5, r5, #0x80 //mask to isolate the clock generating bit. (i.e. BUSY flag)
 CMP r5, #0x80     // compare the masked with the value that would be expected if the clock was still running
 BEQ wait1         // if the clock is still running go back and check again. i.e. when BUSY == 0 move on. 

ldr r5, =32       
lsl r5, r5, #12    // shift the 32 over to bits 23-12, integer portion of divisor in CM PWMDIV register
ORR r5, r5, r3     // OR the password
ldr r2, =CM_PWMDIV
str r5, [r2]       //set integer portion of divisor to (decimal) 32 

ldr r2, [r1]       // load contents of CM_PWMCTL into r2
ORR r2, #0x11      // set source to oscillator (i.e. 19.2MHz can't find it documented but several examples suggest this is right), and Request a clock start 
ORR r2, r2, r3     // OR with password
str r2, [r1]       // With a divisor of 32 the final output freq should be 600kHz 

wait2:
 ldr r5, [r1]      // load contents of clock master pwm control register into r5
 AND r5, r5, #0x80 // mask off the, clock generating bit. 
 CMP r5, #0x0      // check to see if the bit is cleared
 BEQ wait2         // if the clock isn't running wait until it starts.  

//PWM SETUP 
LDR r1, =PWMRNG1  // this is the S in the M/S clock cycle
ldr r2, =6        // 600kHz/6 should be 10us per write from data
str r2, [r1]  

ldr r1, =PWMDAT1 // This is the M in the M/S when MESN=1 
ldr r2, =3
str r2, [r1]    // I assume this will give me a 50% duty cycle

ldr r0, =0x20200004 // gpio function select register 1.
ldr r1, =0x02040000 // mask for bits 26-24 to put gpio 18 alt function 5 (i.e. PWM0) and gpio 16 (output OK LED)
str r1,[r0] // set gpio 18 (pin 12) to alt function defined above 

// Enable PWM
ldr r1, =PWMCTL
ldr r2, =0x81
str r2, [r1] // MSES1=1 Channel 1 PWM M/S mode, and enable PWM1.

/*
* Set the 16th bit of r1.
*/
mov r1,#1
lsl r1,#16

ldr r0, =0x20200000 // gpio base
/*
* Set GPIO 16 to low, causing the LED to turn on.
*/
str r1,[r0,#40]