;*********************************************************************************************************
;* ROBOCOMP19 - SuicideBot
;
;* Data:	    São José dos Campos, 23 de Outubro de 2017
;* Escuderia:	    SuicideBot
;* Integrantes:	    Lucas Jorge, Gabriela Lima, Talize Facó
;
;* esquerda -> '11000000', direita -> '11000001', STOP -> '11000010'
;
;* A comunicacao serial foi implementada com a seguinte configuracao:
;* 9600bps de taxa, 8 bits de dados, sem paridade, 1 stop bit
;* A rotina foi desenvolvida originalmente por: Marcio José Soares
;* Compilador MPLAB 6.32.00
;* PIC utilizado - PIC16F648A-I/P
;* Oscilador: interno a 4MHz
;***********************************************************************************************************
list p=16F648a				   	;modelo do microcontrolador para geracao do codigo
radix dec					;padrao decimal para valores nao identificados
include <P16F648A.INC>				;funcoes do microcontrolador selecionado

;* Para melhorar a eficiencia no momento da gravacao use a linha de configuracao dos FUSES a seguir:
__CONFIG _LVP_OFF & _MCLRE_OFF & _BODEN_OFF & _LVP_OFF & _CP_OFF & _PWRTE_OFF &  _WDT_OFF & _INTOSC_OSC_NOCLKOUT

;*****************************************************
;* Definições
;*****************************************************

#DEFINE bank0	BCF	STATUS,RP0		;seta banco 0 da memória
#DEFINE bank1	BSF	STATUS,RP0		;seta banco 1 da memória

PICRES      	equ 0x00
PICRAM	    	equ 0x20
M	    	equ 0x01
N	    	equ 0x00
DIR_STATUS	equ B'11000010'			;a seta o 0, b seta o 1
VEL_STATUS	equ B'00010010'			;1,2,3,4 setam 0,1,2,3
LOW_P		equ 0x23			;parte baixa do pulso
HI_P		equ 0x24			;parte alta do pulso
DELAY		equ 0x27	
;*****************************************************
;* Variáveis
;*****************************************************
		org	PICRAM			;inicio da RAM
T1		res	1			;variável para temporizador
T2		res	1			;variável para temporizador
T3		res	1			;variável para temporizador
T4		res	1			;variável para temporizador
AUX		res	1			;variável para dado recebido

;**************************************************************
;*define memoria de programa e vetor de reset
;**************************************************************

        org     PICRES      	;reset

meureset:

        goto    inicio      	;desvia do endereco 0x04 - interrupcao

;**************************************************************
;* endereço e rotina de interrupção
;**************************************************************
        org     4           	;toda interrupção aponta para este endereço

minhaint:

        goto	IntVector	;desvia para tratamento da INT

;
;**************************************************************
;* inicio do programa
;**************************************************************
	org 	0x30		;inicio da memoria de programa

inicio:
    	movlw   0x00        	;ajuste para os bits INTCON
    	movwf   INTCON
        
	clrf	PORTA
 	clrf	PORTB

    	bank1			;seleciona banco 1 para options e tris

   	movlw   0x00        	;ajusta como saída. 
   	movwf   TRISA			

   	movlw   0x02        	;ajusta os bits em B como saida. RB1 é entrada
    	movwf   TRISB			

    	bank0               	;volta ao banco 0... (padrão do reset)

;**************************************************************
;* configura o canal serial no PIC
;**************************************************************
	bank1				;seleciona banco 1
	movlw	0x19			;velocidade selecionada 9600
	movwf	SPBRG			;8 bits, sem paridade, 1 stop bit
	bsf	TXSTA,TXEN		;habilita transmissao
	bsf	TXSTA,BRGH		;seleciona baud rate alto
	bcf	TXSTA,SYNC		;deleciona modo assincrono
	bank0				;volta para banco 0
	bsf	RCSTA,SPEN		;habilita port serial
	bsf	RCSTA,CREN		;habilita recepcao continua
	bcf	PIR1,RCIF		;limpa flag de interrupcao RCIF
	bank1				;seleciona banco 1
	bsf	PIE1,RCIE		;seta interrupçao RCIE
	bank0				;volta para banco 0
	bsf	INTCON,PEIE		;habilita perifericos
	bsf	INTCON,GIE		;habilita interrupçoes (Global)

	movlw	0x07			;desliga os conversores AD internos
	movwf	CMCON

;**************************************************************
;* loop principal
;* pisca led enquanto aguarda dado via serial
;* trabalha com a INT de recepção
;**************************************************************
loop:
	nop				;falta implementar o pisca_led
	goto	PWM_LOOP
	goto	loop			;faz infinitamente

;***************************************************************
; subrotinas de interrupçoes
; salve o contexto geral se necessario - registros W, STATUS, etc
;***************************************************************
IntVector:
	btfss	PIR1,RCIF		;USART gerou int?
	goto	OutraInt		;nao, entao verifique outra int

	movlw	0x06			;mascara para bits
	andwf	RCSTA,W			;checa erros
	btfss	STATUS,Z		;encontrado erro???
	goto	RcvError		;sim, trata

	movf	RCREG,W			;Sem erros. Pega dado recebido
	movwf	AUX			;guarda caracter recebido
	call	enviaeco		;e envia mensagem e caracter de eco
	call	resolve			;resolve o caracter recebido
	goto	IntEnd			;fim da int. Restaure o contexto se necessario

RcvError:
	bcf	RCSTA,CREN		;limpa status de recebimento
	bsf	RCSTA,CREN
	goto	IntEnd			;fim da int. Restaure o contexto se necessario

OutraInt:
	goto	$			;não existe outra int. Apenas retorna

IntEnd:
	retfie				;retorna da INT

;***************************************************************
; subrotinas para enviar mensagens via RS232
;***************************************************************
enviaeco:
	movlw	'r'
	movwf	TXREG			;envia caracter
	call	TestTx
	movlw	'e'
	movwf	TXREG			;envia caracter
	call	TestTx
	movlw	'c'
	movwf	TXREG			;envia caracter
	call	TestTx
	movlw	'e'
	movwf	TXREG			;envia caracter
	call	TestTx
	movlw	'b'
	movwf	TXREG			;envia caracter
	call	TestTx
	movlw	'i'
	movwf	TXREG			;envia caracter
	call	TestTx
	movlw	'd'
	movwf	TXREG			;envia caracter
	call	TestTx
	movlw	'o'
	movwf	TXREG			;envia caracter
	call	TestTx
	movlw	'-'
	movwf	TXREG			;envia caracter
	call	TestTx
	movlw	'>'
	movwf	TXREG			;envia caracter
	call	TestTx
	movf	AUX,W			;envia caracter de eco
	movwf	TXREG			;envia caracter
	call	TestTx
	movlw	0x0D			;enter
	movwf	TXREG			;envia caracter
	call	TestTx
	movlw	0x0A			;Line feed
	movwf	TXREG			;envia caracter
	call	TestTx
	return

;***************************************************************
; subrotina para testar buffer de transmissão
;***************************************************************
TestTx:				
	bank1		
Tst:
	btfss	TXSTA,TRMT		;testa se buffer vazio
	goto	Tst			;não, aguarda
	bank0
	return				;buffer vazio, pronto para próximo TX
	
;**************************************************************
;* Rotina que resolve o caracter recebido
;**************************************************************
resolve:				; verifica qual caractere(byte) foi recebido:
	movf	AUX,W			
	xorlw 	B'11000000'		; 11000000 representa ESQUERDA
	btfsc	STATUS,Z
	call 	mliga_esq
	
	movf	AUX,W			
	xorlw 	B'11000001'		; 11000001 representa DIREITA
	btfsc	STATUS,Z
	call 	mliga_dir

	movf	AUX,W			
	xorlw 	B'11000010'		; 11000010 representa STOP
	btfsc	STATUS,Z
	call 	mdesl
	
	movf	AUX,W			
	andlw	B'11000000'		; se os 2 bits mais significativos forem
	xorlw	B'00000000'		; '0', então representa uma VELOCIDADE
	btfsc	STATUS,Z
	call	set_vel
	
	return
	
set_vel:
	movf	AUX,W			; guarda a velocidade na variável
	movwf	VEL_STATUS		; VEL_STATUS
	return
	
mliga_esq:
	movlw	B'11000000'		; guarda DIREITA na variável
	movwf	DIR_STATUS		; DIR_STATUS
	return

mliga_dir:
	movlw	B'11000001'		; guarda ESQUERDA na variável
	movwf	DIR_STATUS		; DIR_STAUS
	return
	
mdesl:
	movlw	B'11000010'		; guarda STOP na variável
	movwf	DIR_STATUS		; DIR_STATUS
	bcf 	PORTA,M			; desliga o motor ESQUERDO
	bcf 	PORTA,N			; desliga o motor DIREITO
	return	

;* Modulador PWM - Receive Interruption caracter

;**************************************************************
;* LOOP PWM 
;**************************************************************
PWM_LOOP:			; o motor direito gira mais que o esquerdo
	movf	DIR_STATUS,W	; (por detalhes de HARDWARE) para compensar 
	xorlw	B'11000000'	; isso, quando o operante for o motor esquerdo,
	btfsc	STATUS,Z	; chama-se a subrotina compensate_left para
	goto	compensate_left	; incrementar ligeiramente sua velocidade
			
	movf	VEL_STATUS,W		; DUTY CYCLE:
	movwf	HI_P			; duraçao_pulso_alto
	sublw	63
	movwf	LOW_P			; duraçao_pulso_baixo = 63 - (pulso alto)
    
gerar:	movf	DIR_STATUS,W
	xorlw	B'11000000'		
	btfsc	STATUS,Z		; se o DIR_STATUS for ESQUERDA
	bsf	PORTA,M			; liga o motor ESQUERDO
	
	movf	DIR_STATUS,W
	xorlw	B'11000001'
	btfsc	STATUS,Z		; se o DIR_STATUS for DIREITA
	bsf	PORTA,N			; liga o motor DIREITO
	
alta:	nop
	call	delay			; atrasa o pulso para manter sua
	decfsz	HI_P			; frequência em 100Hz, em concordância
	goto 	alta			; com o HARDWARE

	bcf	PORTA,M			; desliga o motor ESQUERDO
	bcf	PORTA,N			; desliga o motor DIREITO
    
baixa:	nop
	call	delay			; atrasa o pulso novamente
	decfsz	LOW_P			
	goto	baixa			

	goto	loop

delay:					;;;;;;;;;;;;;;;;;;;;;;;;;
	movlw	50			;;;;;;;;;;;;;;;;;;;;;;;;;
	movwf	DELAY			;;;;; DELAY para ;;;;;;;;
loop_delay:				;;;;; ajustar a ;;;;;;;;;
	decfsz	DELAY			;;;;; frequência ;;;;;;;;
	goto	loop_delay		;;;;;;;;;;;;;;;;;;;;;;;;;
	return				;;;;;;;;;;;;;;;;;;;;;;;;;

compensate_left:
	movf	VEL_STATUS,W
	addlw	1			; incrementa em 1 unidade a velocidade
	movwf	HI_P			; do motor esquerdo
	sublw	63			; (esse valor mostrou-se adequado experimentalmente)
	movwf	LOW_P
	goto gerar
	
	end
	
;**************************************************************
;* Temporiza com clock de 4MHz interno ou externo
;* Trata-se de um temporizador com espera ocupada
;**************************************************************
;_1ms:
;	movlw   0x02			;carrega W com 02
;	movwf	T1			;carrega T1 com W
;	movlw	0x00			;carrega T4 com 0
;	movwf	T4
;	goto	car_1
;
;_5ms:
;    	movlw   0x06			;carrega W com 06
;	movwf	T1			;carrega T1 com W
;	movlw	0x00			;carrega T4 com 0
;	movwf	T4
;	goto	car_1
;
;_20ms:
;    	movlw   0x18			;carrega W com 24
;	movwf	T1			;carrega T1 com W
;	movlw	0x00			;carrega T4 com 0
;	movwf	T4
;	goto	car_1
;
;_200ms:
;    	movlw   0x01			;carrega W com 1
;	movwf	T1			;carrega T1 com W
;	movlw	0x01			;carrega T4 com 1
;	movwf	T4
;	goto	car_1
;
;;aguarda 1 segundo com clock de 4MHz
;_1000ms:
;	movlw	0x06			;carrega W com 6
;	movwf	T3			;carrega T3 com 6
;	movlw	0x01			;carrega T4 com 1
;	movwf	T4
;	
;car:
;    	movlw   0xff        		;carrega W com 255
;	movwf	T1			;carrega T1 com W
;	btfsc	T4,0			;testa bit 0 de T4
;	decfsz	T3,F			;decrementa T3
;	goto 	car_1
;	return
;
;car_1:
;    	movlw   0xFF        		;carrega W com 255
;	movwf	T2			;carrega T2 com 255
;dec_1:
;	decfsz 	T2,1			;decrementa T2
;	goto	dec_1			;255 x T1 vezes
;	decfsz 	T1,1			;decrementa T1
;	goto 	car_1			;volta a carregar T2
;	btfsc	T4,0			;testa bit 0 de T4
;	goto	car			;retorna 0 em W
;	return
;
;
;		;END				;Fim do programa
;;**************************************************************
;;* fim do programa
;;**************************************************************
