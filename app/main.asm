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
		bic.b   #BIT5,&P1OUT            ; Clear P1.5 output (SDA)
        bis.b   #BIT5,&P1DIR            ; P1.5 output

        bic.b   #BIT6,&P1OUT            ; Clear P1.6 output (SCL)
        bis.b   #BIT6,&P1DIR            ; P1.6 output

        mov.b	#0d, R4					; Clearing R4 to use as Clock Counter
        mov.b	#0d, R5					; Clearing R5 to use as state ( 0 = FREE / 1 = TX / 2 = RX / 3 = STOP / 4 = TX MODE ACK / 5 = RX MODE ACK )
        mov.b	#0d, R6					; Clearing R6 to use as TX buffer

        ; Set up timer B0

		bis.w	#TBCLR, &TB0CTL	;Clear timer and dividers
		bis.w	#TBSSEL__SMCLK, &TB0CTL ; Set SMCLOCK
        bis.w	#MC__STOP, &TB0CTL ; STOP COUNTING

        mov.w	#100d, &TB0CCR0 ; Initialize CCR0 = Clock Pulse
        mov.w	#50d, &TB0CCR1 ; Initialize CCR1 = SDA pulse

        bis.w	#CCIE, &TB0CCTL0 ; Enabled capture/compare IRQ
        bic.w	#CCIFG, &TB0CCTL0 ; Clear interrupt flag

        bis.w	#CCIE, &TB0CCTL1 ; Enabled capture/compare IRQ
        bic.w	#CCIFG, &TB0CCTL1 ; Clear interrupt flag

        mov.b	#11010000b, R6 ; Setting TX buffer to 0x68, the clock address

		NOP
        bis.w	#GIE, SR ; Enable global maskable interrupts
        NOP

        bic.w   #LOCKLPM5,&PM5CTL0 ; Unlock I/O pins

main:
		call 	#i2c_init

		call 	#i2c_start

		call 	#i2c_tx_byte

		mov.b	#00000000b, R6		; SECONDS Register
		call 	#i2c_tx_byte

		mov.b	#32h, R6			; 32 Seconds
		call 	#i2c_tx_byte

		mov.b	#11h, R6			; 11 Minutes
		call 	#i2c_tx_byte

		mov.b	#13h, R6				; 13 Hours
		call 	#i2c_tx_byte

		call 	#i2c_stop

trap:

		jmp trap

;-------------------------------------------------------------------------------
; Subroutines
;-------------------------------------------------------------------------------

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

		ret



; /---- I2C TX bye
; - Will initialize TXing a byte:

; - Set R5 state to 1 (TX mode)
; - Set cycle/SDA counter to 8 iterations (8 bits)
; - Make sure pin is configured to OUTPUT mode
; - Start the clock
; - Loop doing nothing until all 8 bits are transmitted and R5 state is brought to 0 (Free state)

i2c_tx_byte:

		bis.w	#MC__UP, &TB0CTL	; START COUNTING UP

		mov.b	#1d, R5			 	; Set R5 state to TX
		mov.b	#8d, R4			 	; We want to iterate 8 bits

i2c_tx_trap:
		cmp.b	#0d, R5			; Check R5 state
		jne		i2c_tx_trap		; Keep looping to check R5 state until R5 = 0 (FREE)

		ret

; /---- I2C STOP
; - Work in Progress

i2c_stop:
		cmp.b	#0d, R5			; Check if R5 is FREE (0)
		jne		i2c_stop

		mov.b	#3d, R5			; Set R5 state to 3 (STOP)

		ret





;-------------------------------------------------------------------------------
; Interrupt Service Routines
;-------------------------------------------------------------------------------

; SCL Timer
ISR_TB0_CCR0:

		cmp.b	#3d, R5			; If STOP mode
		jeq		clock_stop		; JUMP to STOP mode
		jmp		clock_run		; Otherwise, run clock normally

clock_stop:
		bis.b	#BIT6, &P1OUT	; Set CLOCK HIGH
		jmp		clock_return

clock_run:
		xor.b	#BIT6, &P1OUT	; Flip Clock

clock_return:

		bic.w	#CCIFG, &TB0CCTL0 ; Clear interrupt
		reti

; SDA Timer
ISR_TB0_CCR1:

		cmp.b	#0d, R5			; If FREE mode
		jeq		ret_CCR1		; Just repeat the state check

		; We're first going to check stop mode, since it doesn't require the clock to be low to change SDA state
		cmp.b	#3d, R5				; If STOP mode
		jeq		sda_stop			; Jump to STOP mode

		; Check TX ACK
		cmp.b	#4d, R5				; If ACK mode
		jeq		check_ACK			; Jump to CHECK ACK

		; Now we check clock state, SDA should only change state when clock is low.
		bit.b	#01000000b, P1OUT 	; Compare Clock Bit 1.6, if HIGH, return, since we shouldn't change SDA
		jnz		ret_CCR1			; Return

		; --- Next we check the TX, RX, or ACK states

		; Check TX
		cmp.b	#1d, R5				; If TX mode
		jeq		sda_tx				; Jump to TX

		; Check RX
		; Todo

sda_stop:							; We do this IF state is in R5 state STOP (3), bring SDA to HIGH, turn off timer
		bis.b	#BIT5, &P1DIR		; Set P1.5 as an output
		bic.b	#BIT5, &P1REN		; Disable pull up/down resistor on P1.5

		bic.b	#BIT5, &P1OUT		; Bring SDA LOW

		bit.b   #01000000b, P1OUT	; Check if SCL is HIGH
		jz		ret_CCR1			; Jump to return.

		bis.b	#BIT5, &P1OUT		; Set SDA to HIGH
		bic.w	#MC__UP, &TB0CTL 	; STOP COUNTING
		jmp		ret_CCR1			; Jump to return

;----------- TX method block
sda_tx:
		tst.b	R4					; Check bit counter
		jz		sda_tx_ACK			; If we've pushed all bits, set R5 state to TX ACK (As in, RECEIVE THE ACK)

		bic.b	#BIT5, &P1REN		; Disable pull up/down resistor on P1.5
		bis.b	#BIT5, &P1DIR		; Set P1.5 as an output

		bit.b	#10000000b, R6		; Check bit 7 of buffer
		jnz		set_SDA				; If bit 0 is 1, set SDA HIGH
		jmp		clear_SDA			; else, jump to clear SDA

set_SDA:
		bis.b	#BIT5, &P1OUT		; Set SDA HIGH
		jmp		roll_right_Buffer	; Jump to Roll buffer

clear_SDA:
		bic.b	#BIT5, &P1OUT		; Set SDA to LOW, will roll right after.

roll_right_Buffer:
		rlc.b	R6					; Roll buffer left so new bit to TX is the one to be read
		dec.b	R4					; Buffer rolled, decrement our bit counter by 1
		jmp 	ret_CCR1
;----------- End TX method block

sda_tx_ACK:
		mov.b	#4d, R5				; Set state to TX ACK mode, as in we should be RECEIVING the ACK between TX PACKETS
		bic.b	#BIT5, &P1DIR		; Set P1.5 as an input.
		bis.b	#BIT5, &P1REN		; Enable pull up/down resistor on P1.5
		bis.b	#BIT5, &P1OUT		; Make the resistor a pull-up

		jmp		ret_CCR1

check_ACK:
		bit.b	#BIT5, &P1IN		; Check SDA
		jmp		free_SDA			; If ZERO, that means we've received an ACK, so we can enter our free state for the next packet.
									; Jump regardless, in the future I'll make this stop if we don't get an ACK, but who gives a shit.

free_SDA:
		mov.b	#0d, R5				; Set R5 state to FREE

ret_CCR1:
 		bic.w	#CCIFG, &TB0CCTL1 	; Clear interrupt
		reti

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
            
            .sect	".int43"
            .short	ISR_TB0_CCR0

            .sect	".int42"
            .short	ISR_TB0_CCR1
