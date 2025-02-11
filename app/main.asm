;-------------------------------------------------------------------------------
; MSP430 Assembler Code Template for use with TI Code Composer Studio
;
;
;-------------------------------------------------------------------------------
            .cdecls C,LIST,"msp430.h"       ; Include device header file
            
;-------------------------------------------------------------------------------
            .def    RESET                   ; Export program entry-point to
                                            ; make it known to linker.
;-------------------------------------------------------------------------------
            .text                           ; Assemble into program memory.
            .retain                         ; Override ELF conditional linking
                                            ; and retain current section.
            .retainrefs                     ; And retain any sections that have
                                            ; references to current section.

;-------------------------------------------------------------------------------
RESET       mov.w   #__STACK_END,SP         ; Initialize stackpointer
StopWDT     mov.w   #WDTPW|WDTHOLD,&WDTCTL  ; Stop watchdog timer


;-------------------------------------------------------------------------------
; Main loop here
;-------------------------------------------------------------------------------

init:
		; Set up SDA
		bis.b   #BIT5,&P1OUT            ; Set P1.5 output (SDA)
        bis.b   #BIT5,&P1DIR            ; P1.5 output

		; Set up SCL
        bis.b   #BIT6,&P1OUT            ; Set P1.6 output (SCL)
        bis.b   #BIT6,&P1DIR            ; P1.6 output

		; Configure P4 (S1) for digital I/O
		mov.b	#000h, &P4SEL0
		mov.b	#000h, &P4SEL1

		; Set up registers
        mov.b	#0d, R4					; Clearing R4 to use as Clock Counter
        mov.b	#0d, R5					; Clearing R5 to use as state ( 0 = FREE / 1 = TX / 2 = RX / 3 = STOP / 4 = TX MODE ACK / 5 = RX MODE ACK )
        mov.b	#0d, R6					; Clearing R6 to use as buffer
        mov.b	#0d, R7					; Clearing R7 to use for message byte size.
        mov.w	#0d, R8					; Clearing R8 to use as a WORD to point to memory address

        ; Set up timer B0

		bis.w	#TBCLR, &TB0CTL			;Clear timer and dividers
		bis.w	#TBSSEL__SMCLK, &TB0CTL ; Set SMCLOCK
        bis.w	#MC__STOP, &TB0CTL 		; STOP COUNTING

        mov.w	#100d, &TB0CCR0 		; Initialize CCR0 = Clock Pulse
        mov.w	#50d, &TB0CCR1 			; Initialize CCR1 = SDA pulse

        bis.w	#CCIE, &TB0CCTL0 		; Enabled capture/compare IRQ
        bic.w	#CCIFG, &TB0CCTL0 		; Clear interrupt flag

        bis.w	#CCIE, &TB0CCTL1 		; Enabled capture/compare IRQ
        bic.w	#CCIFG, &TB0CCTL1 		; Clear interrupt flag

		NOP
        bis.w	#GIE, SR 				; Enable global maskable interrupts
        NOP

        bic.w   #LOCKLPM5,&PM5CTL0 		; Unlock I/O pins

main:
		call 	#i2c_init

		mov.w	#DataBlock1, R8			; DataBlock I'm using for TX
		mov.b	#3d, R7					; We are transmitting 3 bytes

		call	#i2c_tx_array			; Transmit array

		NOP								; Breakpoint

continuous:

		mov.w	#DataBlock2, R8			; Datablock I'm using for RX
		mov.b	#3d, R7					; We are receiving 3 bytes

		call	#i2c_rx_array			; Receive to array

		jmp continuous

;-------------------------------------------------------------------------------
; Subroutines
;-------------------------------------------------------------------------------

; /---- I2C TX Array Subroutine
i2c_tx_array:

		add.b	#2d, R7				; Since we send the address and one register, add 2 more bytes

		call 	#i2c_start

		mov.b	@R8+, R6 			; Setting TX buffer to what should be the address.
		rla.b	R6					; Rotate TX buffer left so LSB is 0, which results in a WRITE bit
		call 	#i2c_tx_byte

		mov.b	@R8+, R6			;Set TX buffer to second array element, which should be the REGISTER ADDRESS
		call 	#i2c_tx_byte

i2c_tx_array_loop:
		tst.b	R7					; Check bytes remaining
		jz		i2c_tx_array_return ; Return if zero

		mov.b	@R8+, R6			; Continue to iterate down array
		call	#i2c_tx_byte		; Transmit array data

		jmp		i2c_tx_array_loop

i2c_tx_array_return:

		call 	#i2c_stop

		ret

; /-- I2C RX Array Subroutine
i2c_rx_array:

		add.b	#3d, R7				; Since we send two addresses and one register, add 3 more bytes

		call 	#i2c_start

		mov.b	@R8+, R6			; Setting TX buffer to what should be the address.
		rla.b	R6					; Rotate TX buffer left so LSB is 0, which results in a WRITE bit
		call 	#i2c_tx_byte

		mov.b	@R8+, R6			; Set TX buffer to second array element, which should be the REGISTER ADDRESS
		call 	#i2c_tx_byte

		call 	#i2c_stop			; Stop TXing

		call 	#i2c_start

		mov.b	@R8+, R6			; Setting TX buffer to what should be the address.
		rla.b	R6					; Rotate TX buffer left so LSB is 0
		bis.b	#BIT0, R6			; Set bit to READ
		call 	#i2c_tx_byte

i2c_rx_array_loop:
		tst.b	R7					; Check bytes remaining
		jz		i2c_rx_array_return	; Return if zero

		mov.b	#0d, R6				; Clear buffer
		call	#i2c_rx_byte		; Receive the byte
		;mov.w	R6, &202Ah			FUCK

		jmp		i2c_rx_array_loop

i2c_rx_array_return:

		call 	#i2c_stop

		ret


; /---- I2C Init
; Will bring SDA and SCL HIIGH

i2c_init:
		bis.b   #BIT5,&P1OUT            ; SET 1.5 (SDA) HIGH
		bis.b   #BIT6,&P1OUT            ; SET 1.6 (SCL) HIGH

		ret



; /---- I2C Start
; Will bring SDA LOW while keeping clock HIGH

i2c_start:
		mov.w	#0d, TB0R				; Ensure timer is at 0
		bic.b   #BIT5,&P1OUT            ; BRING 1.5 (SDA) LOW

		bis.w	#MC__UP, &TB0CTL		; START COUNTING UP

		ret



; /---- I2C TX bye
; - Will initialize TXing a byte:

; - Set R5 state to 1 (TX mode)
; - Set cycle/SDA counter to 8 iterations (8 bits)
; - Make sure pin is configured to OUTPUT mode
; - Loop doing nothing until all 8 bits are transmitted and R5 state is brought to 0 (Free state)

i2c_tx_byte:

		mov.b	#1d, R5			; Set R5 state to TX
		mov.b	#8d, R4			; We want to iterate 8 bits

		dec.b	R7				; Decrement byte counter

		bic.b	#BIT5, &P1REN	; Disable pull up/down resistor on P1.5
		bis.b	#BIT5, &P1DIR	; Set P1.5 as an output

i2c_tx_trap:
		cmp.b	#0d, R5			; Check R5 state
		jne		i2c_tx_trap		; Keep looping to check R5 state until R5 = 0 (FREE)

		ret



; /---- I2C RX bye
; - Will initialize RXing a byte:

; - Set R5 state to 2 (RX mode)
; - Set cycle/SDA counter to 8 iterations (8 bits)
; - Make sure pin is configured to INPUT mode
; - Loop doing nothing until all 8 bits are received and R5 state is brought to 0 (Free state)

i2c_rx_byte:

		mov.b	#2d, R5			; Set R5 state to RX
		mov.b	#8d, R4			; We want to iterate 8 bits

		dec.b	R7				; Decrement byte counter

		bic.b	#BIT5, &P1DIR	; Set P1.5 as an input.
		bis.b	#BIT5, &P1REN	; Enable pull up/down resistor on P1.5
		bis.b	#BIT5, &P1OUT	; Make the resistor a pull-up

i2c_rx_trap:
		cmp.b	#0d, R5			; Check R5 state
		jne		i2c_rx_trap		; Keep looping to check R5 state until R5 = 0 (FREE)

		ret



; /---- I2C STOP
; - Makes sure SDA is set to low, then bring SCL high, then bring SDA high, and stop the clocks

i2c_stop:
		cmp.b	#0d, R5			; Check if R5 is FREE (0)
		jne		i2c_stop

		mov.b	#3d, R5			; Set R5 state to 3 (STOP)

i2c_stop_trap:

		cmp.b	#0d, R5			; Check if R5 is FREE (0)
		jne		i2c_stop_trap	; We don't want to leave stop until it's actually finished

		ret



;-------------------------------------------------------------------------------
; Interrupt Service Routines
;-------------------------------------------------------------------------------

; SCL Timer
ISR_TB0_CCR0:

clock_run:
		xor.b	#BIT6, &P1OUT		; Flip Clock

clock_return:

		bic.w	#CCIFG, &TB0CCTL0 	; Clear interrupt
		reti

; SDA Timer
ISR_TB0_CCR1:
;-- First we start with a state check

		cmp.b	#0d, R5				; If FREE mode
		jeq		ret_CCR1			; Just repeat the state check

		; We're first going to check stop mode, since it doesn't require the clock to be low to change SDA state
		cmp.b	#3d, R5				; If STOP mode
		jeq		sda_stop			; Jump to STOP mode

		; Check TX ACK
		cmp.b	#4d, R5				; If ACK mode
		jeq		check_ACK			; Jump to CHECK ACK

		; Check RX ACK
		cmp.b	#5d, R5				; If RX ACK mode
		jeq		set_ACK				; Jump to SET ACK

		; Now we check clock state, SDA should only change state when clock is low.
		bit.b	#01000000b, P1OUT 	; Compare Clock Bit 1.6, if HIGH, return, since we shouldn't change SDA
		jnz		ret_CCR1			; Return

		; --- Next we check the TX, RX, or ACK states

		; Check TX
		cmp.b	#1d, R5				; If TX mode
		jeq		sda_tx				; Jump to TX

		; Check RX
		cmp.b	#2d, R5				; If RX mode
		jeq		sda_rx				; Jump to RX

		jmp ret_CCR1				; This should never be reached.

;--- END STATE CHECK

sda_stop:							; We do this IF state is in R5 state STOP (3), bring SDA to HIGH, turn off timer
		bis.b	#BIT5, &P1DIR		; Set P1.5 as an output
		bic.b	#BIT5, &P1REN		; Disable pull up/down resistor on P1.5

		bic.b	#BIT5, &P1OUT		; Bring SDA LOW

		bit.b   #01000000b, P1OUT	; Check if SCL is HIGH
		jz		ret_CCR1			; Jump to return.

		bis.b	#BIT5, &P1OUT		; Set SDA to HIGH
		bic.w	#MC__UP, &TB0CTL 	; STOP COUNTING
		mov.b	#0d, R5				; Set STATE to FREE
		bic.w	#CCIFG, &TB0CCTL0 	; Clear SCL interrupt
		jmp		ret_CCR1			; Jump to return

;----------- TX method block
sda_tx:
		tst.b	R4					; Check bit counter
		jz		sda_tx_ACK			; If we've pushed all bits, set R5 state to TX ACK (As in, RECEIVE THE ACK)

		bit.b	#10000000b, R6		; Check bit 7 of buffer
		jnz		set_SDA				; If bit 0 is 1, set SDA HIGH
		jmp		clear_SDA			; else, jump to clear SDA

set_SDA:
		bis.b	#BIT5, &P1OUT		; Set SDA HIGH
		jmp		roll_left_Buffer	; Jump to Roll buffer

clear_SDA:
		bic.b	#BIT5, &P1OUT		; Set SDA to LOW, will roll right after.
		jmp		roll_left_Buffer	; Jump to Roll Buffer

;----------- End TX method block

;----------- RX Method Block
sda_rx:
		tst.b	R4					; Check bit counter
		jz		sda_rx_ACK			; If we've pushed all bits, set R5 state to RX ACK (As in, TRANSMIT THE ACK)

		bit.b	#BIT5, &P1IN		; Check SDA
		jnz		set_buffer			; If SDA is 1, set buffer LSB HIGH
		jmp		clear_buffer		; Otherwise, clear buffer LSB

set_buffer:
		bis.b	#BIT0, R6			; Set buffer LSB HIGH
		jmp		roll_left_Buffer	; Jump to Roll Buffer

clear_buffer:
		bic.b	#BIT7, R6			; Set buffer LSB LOW
		jmp		roll_left_Buffer	; Jump to Roll Buffer

;----------- End RX Method Block

roll_left_Buffer:
		rla.b	R6					; Roll buffer left so new bit to TX is the one to be read
		dec.b	R4					; Buffer rolled, decrement our bit counter by 1
		jmp 	ret_CCR1

sda_tx_ACK:
		mov.b	#4d, R5				; Set state to TX ACK mode, as in we should be RECEIVING the ACK between TX PACKETS
		bic.b	#BIT5, &P1DIR		; Set P1.5 as an input.
		bis.b	#BIT5, &P1REN		; Enable pull up/down resistor on P1.5
		bis.b	#BIT5, &P1OUT		; Make the resistor a pull-up

		jmp		ret_CCR1


sda_rx_ACK:
		mov.b	#5d, R5				; Set state to RX ACK mode, as in we should be TRANSMITTING the ACK between RX PACKETS
		bic.b	#BIT5, &P1REN		; Disable pull up/down resistor on P1.5
		bis.b	#BIT5, &P1DIR		; Set P1.5 as an output


		rrc.b	R6					; Allign Buffer
		bic.b	#BIT7, R6			; Allign Buffer



		tst.b	R7					; Check Byte Counter
		jz		sda_rx_NACK			; Return as ACK if byte counter isn't 0, otherwise, we send a NACK.
		bic.b	#BIT5, &P1OUT		; Otherwise, Bring SDA low (ACK)
		jmp		ret_CCR1

sda_rx_NACK:
		bis.b	#BIT5, &P1OUT		; Bring SDA HIGH (NACK)
		jmp		ret_CCR1


check_ACK:
		bit.b	#BIT5, &P1IN		; Check SDA
		jmp		free_SDA			; If ZERO, that means we've received an ACK, so we can enter our free state for the next packet.
									; Jump regardless, in the future I'll make this stop if we don't get an ACK, but who gives a shit.

set_ACK:

		jmp		free_SDA			; Free


free_SDA:
		mov.b	#0d, R5				; Set R5 state to FREE


ret_CCR1:
 		bic.w	#CCIFG, &TB0CCTL1 	; Clear interrupt
		reti

;-------------------------------------------------------------------------------
; Memory Allocation
;-------------------------------------------------------------------------------

		.data				; Allocate variables in data memory
		.retain				; Keep these statements even if not used

		; Format for arrays: Address, Register, Data*
		; When setting byte size in R7, set to the number of DATA packets

DataBlock1: .short 0068h, 1132h, 0012h 	; TX Data (Address 68, Register 0, 32 Seconds, 11 Minutes, 13 Hours)

DataBlock2: .short 0068h, 0068h, 0000h	; RX Data (Address 68, Register 0, Free, Free, Free)

;-------------------------------------------------------------------------------
; Stack Pointer definition
;-------------------------------------------------------------------------------
            .global __STACK_END
            .sect   .stack
            
;-------------------------------------------------------------------------------
; Interrupt Vectors
;-------------------------------------------------------------------------------
            .sect   ".reset"                ; MSP430 RESET Vector
            .short  RESET

            .sect	".int43"				; TB0 CCR0 Vector
            .short	ISR_TB0_CCR0

            .sect	".int42"				; TB0 CCR1 Vector
            .short	ISR_TB0_CCR1