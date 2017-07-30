; csc230_a2.asm
; CSC 230 - Summer 2017
; 
; Instructor - B. Bird - 06/01/2017
;
; Student - M. McKay - 06/24/2017
; Student Number: V00900866
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                        Constants and Definitions                            ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; Special register definitions
.def ZL = r30
.def ZH = r31
.def DELAY = r17
.def DIRECTION = r18
.def CURRENT_LED = r19
.def overflows = r20
.def temp = r21
.equ INVERTED = 0x200

; Stack pointer and SREG registers (in data space)
.equ SPH = 0x5E
.equ SPL = 0x5D
.equ SREG = 0x5F

; Initial address (16-bit) for the stack pointer
.equ STACK_INIT = 0x21FF

; Port and data direction register definitions (taken from AVR Studio; note that m2560def.inc does not give the data space address of PORTB)
.equ DDRB = 0x24
.equ PORTB = 0x25
.equ DDRL = 0x10A
.equ PORTL = 0x10B

; Definitions for the analog/digital converter (ADC) (taken from m2560def.inc)
; See the datasheet for details
.equ ADCSRA = 0x7A ; Control and Status Register
.equ ADMUX = 0x7C ; Multiplexer Register
.equ ADCL = 0x78 ; Output register (high bits)
.equ ADCH = 0x79 ; Output register (low bits)

; Definitions for button values from the ADC
; Some boards may use the values in option B
; The code below used less than comparisons so option A should work for both
; Option A (v 1.1)
;.equ ADC_BTN_RIGHT = 0x032
;.equ ADC_BTN_UP = 0x0FA
;.equ ADC_BTN_DOWN = 0x1C2
;.equ ADC_BTN_LEFT = 0x28A
;.equ ADC_BTN_SELECT = 0x352
; Option B (v 1.0)
.equ ADC_BTN_RIGHT = 0x032
.equ ADC_BTN_UP = 0x0C3
.equ ADC_BTN_DOWN = 0x17C
.equ ADC_BTN_LEFT = 0x22B
.equ ADC_BTN_SELECT = 0x316


; Definitions of the special register addresses for timer 0 (in data space)
.equ GTCCR = 0x43
.equ OCR0A = 0x47
.equ OCR0B = 0x48
.equ TCCR0A = 0x44
.equ TCCR0B = 0x45
.equ TCNT0  = 0x46
.equ TIFR0  = 0x35
.equ TIMSK0 = 0x6E

.cseg

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                          Reset/Interrupt Vectors                            ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
.org 0x0000 ; RESET vector
	jmp reset
	
.org 0x002e	; Timer overflow interrupt handler
	jmp overflow_interrupt

 reset:
	clr r16
    	ldi temp,  0x05
    	sts TCCR0B, temp        ; set the Clock Selector Bits CS00, CS01, CS02 to 101
                            	; this puts Timer Counter0, TCNT0 in to FCPU/1024 mode
                            	; so it ticks at the CPU freq/1024
    	ldi temp, 0x01
    	sts TIMSK0, temp        ; set the Timer Overflow Interrupt Enable (TOIE0) bit 
                            	; of the Timer Interrupt Mask Register (TIMSK0)

    	sei                     ; enable global interrupts -- equivalent to "sbi SREG, I" 

    	clr temp
    	sts TCNT0, temp         ; initialize the Timer/Counter to 0 
	ldi temp, 0x01
	sts TIFR0, temp
	ldi temp, 0x00
	sts INVERTED, temp

    ; Set up the ADC
	
	; Set up ADCSRA (ADEN = 1, ADPS2:ADPS0 = 111 for divisor of 128)
	ldi	r16, 0x87
	sts	ADCSRA, r16
	
	; Set up ADMUX (MUX4:MUX0 = 00000, ADLAR = 0, REFS1:REFS0 = 1)
	ldi	r16, 0x40
	sts	ADMUX, r16	
	
	; Now, check the button values until something below the highest threshold
	; of each respective button is returned from the ADC.
	
	; Store the button thresholds in r29:r22
	ldi	r22, low(ADC_BTN_UP)
	ldi	r23, high(ADC_BTN_UP)
	ldi	r24, low(ADC_BTN_DOWN)
	ldi	r25, high(ADC_BTN_DOWN)
	ldi	r26, low(ADC_BTN_RIGHT)
	ldi	r27, high(ADC_BTN_RIGHT)
	ldi	r28, low(ADC_BTN_LEFT)
	ldi	r29, high(ADC_BTN_LEFT)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                               Main Program                                  ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; According to the datasheet, the last interrupt vector has address 0x0070, so the first
; "unreserved" location is 0x0074
.org 0x0074
;============
   

; Initialize stack, variables and PORTB/L directions

main_begin:
	
	; Set our current LED to 0 and direction to forward
	ldi CURRENT_LED, 0x0
	; Set the initial direction of the LED to 1
	ldi DIRECTION, 0x01
	; Set the initial DELAY variable to 61
	ldi DELAY, 0x3d
	; Set the direction on ports B and L 
	ldi r16, 0xff
	sts DDRB, r16
	sts DDRL, r16

	; Initialize the stack
	ldi r16, high(STACK_INIT)
	sts SPH, r16
	ldi r16, low(STACK_INIT)
	sts SPL, r16

; This is the main loop	
start:

	; Wait for an ADC Conversion and a timer interrupt
        ; The conversion needs to effect the value in
	; Start an ADC conversion
	
	; Set the ADSC bit to 1 in the ADCSRA register to start a conversion
	lds	temp, ADCSRA
	ori	temp, 0x40
	sts	ADCSRA, temp
	
	; Wait for the conversion to finish
	wait_for_adc:
		lds		temp, ADCSRA
		andi	temp, 0x40
		brne	wait_for_adc
	
	; Load the ADC result into the Z pair (ZH:ZL). Note that ZH and ZL are defined above.
	lds	ZL, ADCL
	lds	ZH, ADCH
	
	; Right - r27:r26
	cp	ZL, r26 ; Low byte
	cpc	ZH, r27 ; High byte
	
	brlo uninvert ; If the ADC value was above the threshold, no button was pressed (so try again)
	
	; Up - r23:r22
	cp	ZL, r22 ; Low byte
	cpc	ZH, r23 ; High byte
	
	brlo speedup ; If the ADC value was above the threshold, no button was pressed (so try again)
	
	; Down -  r25:r24
	cp	ZL, r24 ; Low byte
	cpc	ZH, r25 ; High byte
	
	brlo speeddown	
	
	; Left -  r29:r28
	cp	ZL, r28 ; Low byte
	cpc	ZH, r29 ; High byte
	
	brlo invert
	rjmp start                    
uninvert:
	push temp
	ldi temp, 0x00
	sts INVERTED, temp
	pop temp
	rjmp start
	
invert:
	push temp
	ldi temp, 0x01
	sts INVERTED, temp
	pop temp
	rjmp start
	
speedup:
	push temp
	ldi temp,  0x04
	sts TCCR0B, temp    ; set the Clock Selector Bits CS00, CS01, CS02 to 101
                            ; this puts Timer Counter0, TCNT0 in to FCPU/256 mode
                            ; so it ticks at the CPU freq/256
	pop temp
	rjmp start

speeddown:
	push temp
	ldi temp,  0x05
        sts TCCR0B, temp    ; set the Clock Selector Bits CS00, CS01, CS02 to 101
                            ; this puts Timer Counter0, TCNT0 in to FCPU/1024 mode
                            ; so it ticks at the CPU freq/1024
	pop temp
	rjmp start	                

overflow_interrupt:
    	inc overflows                              ; add 1 to the overflows variable
    	cp overflows, DELAY                        ; compare with DELAY default value 61
    	brne PC+6                                  ; Program Counter + 6 (skip next line) if not equal
    	clr overflows                              ; if 61 overflows occured reset the counter to zero
    	call CLEAR_LEDS                            ; Clear all LEDs by calling CLEAR_LEDS
    	call SET_LED                               ; Call the subroutine SET_LED which uses the value in r16
    	ADD CURRENT_LED, DIRECTION                 ; Add the value in DIRECTION to CURRENT_LED
    	call check_direction	                   ; Check the direction of the program and switch if conditionals are met
    	reti                                       ; Return from interrupt

; Check the direction of the program
check_direction:
    	cpi DIRECTION, 1
    	breq forward
    	cpi CURRENT_LED, 0
    	breq change_direction
    	ret
    	forward:
    		cpi CURRENT_LED, 5
    		breq change_direction
    		ret	
    	change_direction:
   	 	neg DIRECTION
    		ret
		
; CLEAR_LEDS()
; Turn off all LEDs on Ports B and L
CLEAR_LEDS:
    	; This function uses r16, so we will save it onto the stack
    	; to preserve whatever value it already has.
    	push r16
		
    	; Set PORTL and PORTB to 0x00
    	ldi r16, 0x00
    	sts PORTL, r16
    	sts PORTB, r16
	
    	; Load the saved value of r16
    	pop r16
	
	; Return from function 
        ret

; SET_LED(CURRENT_LED: index)
; This function takes an argument in r16. The argument will
; be an index between 0 and 2, giving the LED to light:
;   CURRENT_LED = 0 - Light the LED on Pin 52 (Port B Bit 1)
;   CURRENT_LED = 1 - Light the LED on Pin 50 (Port B Bit 3)
;   CURRENT_LED = 2 - Light the LED on Pin 48 (Port L Bit 1)
;   CURRENT_LED = 3 - Light the LED on Pin 46 (Port L Bit 3)
;   CURRENT_LED = 4 - Light the LED on Pin 44 (Port L Bit 5)
;   CURRENT_LED = 5 - Light the LED on Pin 42 (Port L Bit 7)

SET_LED:
	; Push registers onto the stack to preserve values and use the registers in here
	push r16
	push r17
	push temp
	clr r17

	; Load the boolean value of INVERTED into temp
	lds temp, INVERTED

	; A 6-case if-statement for the different index values
	cpi CURRENT_LED, 0
	breq SET_LED_idx0
	cpi CURRENT_LED, 1
	breq SET_LED_idx1
	cpi CURRENT_LED, 2
	breq SET_LED_idx2
	cpi CURRENT_LED, 3
	breq SET_LED_idx3
	cpi CURRENT_LED, 4
	breq SET_LED_idx4
	cpi CURRENT_LED, 5
	breq SET_LED_idx5
	rjmp SET_LED_done
	
SET_LED_idx0:
	; Set Port B, Bit 1
	; Load the existing value of PORTB
	lds r17, PORTB
	; Use Bitwise OR to set bit 1 to 1
	; (Note that bit 1 is the second bit from the right)
	ori r17, 0x02
	sts PORTB, r17
	rjmp SET_LED_done
SET_LED_idx1:
	; Set Port B, Bit 3
	; Load the existing value of PORTB
	lds r17, PORTB
	; Use Bitwise OR to set bit 3 to 1	
	ori r17, 0x08
	sts PORTB, r17
	rjmp SET_LED_done
SET_LED_idx2:	
	; Set Port L, Bit 1
	; Load the existing value of PORTL
	lds r17, PORTL
	; Use Bitwise OR to set bit 1 to 1	
	ldi r17, 0x02
	sts PORTL, r17
	rjmp SET_LED_done
SET_LED_idx3:	
	; Set Port L, Bit 3
	; Load the existing value of PORTL
	lds r17, PORTL
	; Use Bitwise OR to set bit 3 to 1	
	ori r17, 0x08
	sts PORTL, r17
	rjmp SET_LED_done
SET_LED_idx4:	
	; Set Port L, Bit 5
	; Load the existing value of PORTL
	lds r17, PORTL
	; Use Bitwise OR to set bit 5 to 1	
	ori r17, 0x20
	sts PORTL, r17
	rjmp SET_LED_done
SET_LED_idx5:	
	; Set Port L, Bit 7
	; Load the existing value of PORTL
	lds r17, PORTL
	; Use Bitwise OR to set bit 7 to 1	
	ori r17, 0x80
	sts PORTL, r17
	rjmp SET_LED_done
SET_LED_done:	
	; Check if the boolean flag is set and if it is ones complement bits in PORTB/L	
	cpi temp, 0x01
	breq PC + 2
	brne PC + 10
	lds r16, PORTB
	lds r17, PORTL
	com r16
	com r17
	sts PORTB, r16
	sts PORTL, r17
	pop temp
	pop r17
	pop r16
	ret

; Finish line
stop:
	rjmp stop
	


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                               Data Section                                  ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

.dseg
.org 0x200

