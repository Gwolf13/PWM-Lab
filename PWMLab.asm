;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Title: PWM1
; Team Members: Emmanuel Bastidas, Gwendolyn Wolf, Zahra Ziaee
;
; Desc: Loop dimming a LED in RA4 from full ON to full OFF
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; 
    
#include <xc.inc>
#include "pic16f18875.inc" 

; CONFIG1 
; __config 0xFFFF 
 CONFIG FEXTOSC=ECH
 CONFIG RSTOSC=EXT1X
 CONFIG CLKOUTEN=OFF
 CONFIG CSWEN=ON
 CONFIG FCMEN=ON 
 CONFIG MCLRE=ON
 CONFIG PWRTE=OFF
 CONFIG LPBOREN=OFF
 CONFIG BOREN=ON
 CONFIG BORV=LO
 CONFIG ZCD=OFF
 CONFIG PPS1WAY=ON
 CONFIG STVREN=ON 
 CONFIG WDTCPS=31
 CONFIG WDTE=OFF
 CONFIG WDTCWS=7
 CONFIG WDTCCS=SC 
 CONFIG WRT=OFF
 CONFIG SCANE=available
 CONFIG LVP=ON 
 CONFIG CP=OFF
 CONFIG CPD=OFF 
 
; Variables and Constants follow the format
; name EQU location in memory
 i EQU 70H
 j EQU 71H ; global data
 onCount EQU 72H ; Store the on time on memory 72
 offCount EQU 73H ; Store the off time on memory 73 
 
 PSECT res_vect, class=CODE, delta=2
res_vect:
 ;org 0x0000 
 goto Start
 ; learn later there is memory that can be used here

; this is the entry point of the program
Start:
    banksel 	OSCFRQ		; Select clock bank
    movlw	00000000B	; 1 Mhz clock move from l to w(working register)
    movwf  	OSCFRQ      	; Load new clock, move from working register to f
				; with f being OSCFRQ
    
    ; this configures all 'A' ports as digital
    banksel ANSELA   ; this is for analog select, each pin can be analog or digital
    clrf    ANSELA   ; clearing all                  
    
    ; this is more often used when pins are set to inputs
    ; however if the pin is set as output, the latch bit for the pin
    ; will change to match the port bit of the pin
    ; so we want to clear this when having pins set as outputs because it is not known
    ; what the previous port bit value is
    banksel PORTA    ; select the memory bank that contains the PORTA register
    clrf    PORTA    ; clear the PORTA register
		     ; this sets the pins RA0-RA7 as logic lows
   
    ; configures pins RA0-RA7 as output pins
    banksel TRISA ; move to the memory bank containing TRISA register
    clrf    TRISA ; clear this register to zero.
		  ; this makes it so all RA0-RA7 register are now configured as outputs
   
    ; Set all the pins RA0-RA7 as logic lows 
    banksel LATA  ; move to the data bank that contains LATA register
    clrf    LATA  ; clear all the bits in the register which sets RA0-RA7 as logic lows
    bcf    LATA, 4  ; set ra4 to high(sets selelct  pin to high)
  
    
BeginPWM:
    movlw 0xff	  ; The only way to do a load immediate is two steps
    movwf onCount ; Then move it to memory onCount
    
    movlw 0x01	  ; Now get the offCount
    movwf offCount; Prime the pump for the time off
    
LOOP: 
    banksel LATA ; Modify our light switches
    bsf LATA, 4; Turn on RA4 because I'm to lazy to wire it up
    ; Prime the pump 
    movf onCount, w ; Move onCount into the working Directory
    movwf j	    ; Now move it to j 
    call DELAY1
    
    bcf LATA, 4	; Turn of RA4
    
    ; Prime the pump
    movf offCount, w ; Move offCount to the woring Directory
    movwf j	     ; This is like putting it in $a0
    call DELAY1
    
    ; Update 
    incf offCount   ; This is offCount++
    decfsz onCount  ; This is onCount-- and if onCount==0, skip the line
    goto LOOP
    
; Brighten it back up 
    movlw 0xff	  ; The only way to do a load immediate is two steps
    movwf offCount ; Then move it to memory offCount
    
    movlw 0x01	  ; Now get the onCount
    movwf onCount; Prime the pump for the time off

    
LOOP2: 
    banksel LATA ; Modify our light switches
    bsf LATA, 4; Turn on RA4 because I'm to lazy to wire it up
    ; Prime the pump 
    movf onCount, w ; Move onCount into the working Directory
    movwf j	    ; Now move it to j 
    call DELAY1
    
    bcf LATA, 4	; Turn of RA4
    
    ; Prime the pump
    movf offCount, w ; Move offCount to the woring Directory
    movwf j	     ; This is like putting it in $a0
    call DELAY1
    
    ; Update 
    incf onCount   ; This is onCount++
    decfsz offCount  ; This is offCount-- and if offCount==0, skip the line
    goto LOOP2 
    
    goto BeginPWM
    
DELAY1:
    decfsz  j,	F ; Decrement and skip next instruction
    goto DELAY1	  ;  Delay loop
    return
   



