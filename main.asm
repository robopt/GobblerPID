;;;;;;;;;;;;;;;;;;;;
;; GobblerPID ASM ;;
;; Edward Mead    ;;
;;;;;;;;;;;;;;;;;;;;
; Assembly functions which I rewrote/created
	XDEF ad0_enable, ad0_enableLine, ad0convfast

; A/D converter PADD0-PADD7
; Sets Mode of A/D to Interrupt driven
; Uses ports AN0-3
ad0_enable:
			ldaa	#$82
			staa	ATD0CTL2
			ldaa	#$00
			staa	ATD0CTL3
			ldaa	#$E5
			staa	ATD0CTL4
			ldaa	#$B4
			staa	ATD0CTL5
			rts
; A/D converter PADD0-PADD7
; Sets Mode of A/D to non-Interrupt driven
; Uses ports AN4-7
ad0_enableLine:
			ldaa	#$80
			staa	ATD0CTL2
			ldaa	#$38
			staa	ATD0CTL3
			ldaa	#$80			;set ADPU & AFFC
			staa	ATD0CTL4
			ldaa	#$B0
			staa	ATD0CTL5  
			rts
			
; A/D Read (Single)
ad0convfast:
			andb	#$07
			orab	#$80
			stab	ATD0CTL5
			rts