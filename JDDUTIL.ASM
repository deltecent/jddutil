; JDDUTIL.ASM
;
; THE JDDUTIL PROGRAM MUST BE ASSEMBLED WITH THE TDL
; TDL ZASM ASSEMBLER ON A Z80 PROCESSOR
;
; VERSION  AUTHO	DATE		DESCRIPTION
; 1.0      P. LINSTRUTH	05/20/20	INITIAL VERSION
; 1.1      P. LINSTRUTH	05/27/20	ADDED MEM DUMP
;

;******************************************************
; TDL ZASM ASSEMBLER DIRECTIVES                       *
;******************************************************

	.PABS		;ABSOLUTE ADDRESSING.
	.PHEX		;INTEL HEX OBJECT FILE.
	.XLINK		;NO LINKAGE REQUIRED.

;******************************************************
; DCMLOAD MAY RUN UNDER CP/M OR STAND-ALONE.          *
;******************************************************

CPM.TPA	==	0100H		;TRANSIENT PROGRAM AREA
CPM.WB	==	0000H		;CP/M WARM BOOT VECTOR
JMP.INS	==	0C3h		;8080 JMP INSTRUCTION


	.LOC	CPM.TPA


;*******( START PROGRAM )******************************

	LXI     SP,STACK	;INIT STACK POINTER

	CALL	CK.CPM		;CHECK FOR CP/M
	CALL	IO.INI		;INITIALIZE I/O

	LXI	H,M.VER		;DISPLAY PROGRAM BANNER
	CALL	IO.MSG
	LXI	H,M.AUTH
	CALL	IO.MSG

;*******( COMMAND LOOP )******************************

CMD.LP:	LXI	H,M.CMD		;DISPLAY PROMPT
	CALL	IO.MSG
	LXI	H,CMD.LP	;CREATE RETURN ADDRESS FOR COMMANDS
	PUSH	H
	CALL	IO.GET		;COMMAND IN HL
	MOV	A,M
	INX	H		;POINT HL TO TOKEN PAST COMMAND CHARACTER
	CPI	'C'		;DISPLAY COMMAND BLOCK
	JZ	CMD.C
	CPI	'B'		;DISPLAY STORAGE LOCATIONS
	JZ	CMD.B
	CPI	'D'		;DUMP MEMORY
	JZ	CMD.D
	CPI	'E'		;EXECUTE
	JZ	CMD.E
	CPI	'F'		;FORMAT DISK
	JZ	CMD.F
	CPI	'I'		;INITIALIZE DD AND DCM
	JZ	CMD.I
	CPI	'M'		;SET MEM BANK
	JZ	CMD.M
	CPI	'S'		;DISPLAY STATUS PORT
	JZ	CMD.S
	CPI	'T'		;SHOW BOOT BLOCKS
	JZ	CMD.T
	CPI	'X'		;EXIT TO CP/M
	CZ	..DONE
	LXI	H,M.HELP	;INVALID COMMAND ENTERED, DISPLAY HELP
	JMP	IO.MSG

..DONE:	LDA	CPM.FLG		;RUNNING UNDER CP/M?
	ORA	A
	JNZ	CPM.EX		;YES, RETURN TO CP/M

..LOOP:	JMP	..LOOP		;NO, JUST LOOP

;******************************************************
; DISPLAY DCM BUFFERS
;******************************************************
CMD.B:	MVI	A,DC.MB0	;REQUEST BANK 0
	OUT	D.PORT

	LHLD	D.ADDR		;DD SYS ADDRESS.
	LXI	D,DD.DSL	;DCM STORAGE LOC.
	DAD	D

	XCHG			;DE PTS TO DCM STORAGE
	LXI	H,M.DSL
	CALL	IO.MSG

	LXI	H,M.DSL0	;RETRY ERROR COUNT
	CALL	IO.MSG
	LDAX	D
	CALL	PR.BYT
	CALL	PR.NL
	INX	D

	LXI	H,M.DSL1	;CL.CTL DRIVE BITS
	CALL	IO.MSG
	LDAX	D
	CALL	PR.BYT
	CALL	PR.NL
	INX	D

	LXI	H,M.DSL2	;CL.CTL DRIVE AND SIDE BITS
	CALL	IO.MSG
	LDAX	D
	CALL	PR.BYT
	CALL	PR.NL
	INX	D

	LXI	H,M.DSL3	;CL.CTL LAST ISSUED
	CALL	IO.MSG
	LDAX	D
	CALL	PR.BYT
	CALL	PR.NL
	INX	D

	LXI	H,M.DSL4	;FD179X-02 STATUS VALUE
	CALL	IO.MSG
	LDAX	D
	CALL	PR.BYT
	CALL	PR.NL
	INX	D

	LXI	H,M.DSL5	;PHYS TRAK NUMBER
	CALL	IO.MSG
	LDAX	D
	CALL	PR.BYT
	CALL	PR.NL

	LXI	H,M.SEC		;DISPLAY SECTOR BUFFER
	CALL	IO.MSG
	CALL	PR.SEC

	RET


;******************************************************
; DISPLAY DCM COMMAND BLOCK
;******************************************************
CMD.C:	CALL	GET.CB		;GET COMMAND BLOCK

	LXI	H,M.CB
	CALL	IO.MSG

	LXI	H,M.CB0
	CALL	IO.MSG
	LDA	DCM.CMD
	CALL	PR.BYT
	CALL	PR.NL

	LXI	H,M.CB1
	CALL	IO.MSG
	LDA	DCM.DRV
	CALL	PR.BYT
	CALL	PR.NL

	LXI	H,M.CB2
	CALL	IO.MSG
	LDA	DCM.TRK
	CALL	PR.BYT
	CALL	PR.NL

	LXI	H,M.CB3
	CALL	IO.MSG
	LDA	DCM.SEC
	CALL	PR.BYT
	CALL	PR.NL

	LXI	H,M.CB4
	CALL	IO.MSG
	LDA	DCM.STS
	CALL	PR.BYT
	CALL	PR.NL

	RET

;******************************************************
; DUMP MEMORY
;******************************************************
CMD.D:	MOV	A,M		;NEXT CHARACTER
	ORA	A
	JZ	..NADR

	CALL	G.ADR
	SHLD	D.ADR		;SAVE IN D.ADR

..NADR: CALL	PR.BUF
	RET

;******************************************************
; EXECUTE DCM COMMAND
;******************************************************
CMD.E:	CALL	G.TKN		;GET DCM CMD
	LXI	H,M.EXCH	;NO TOKEN
	JZ	IO.MSG		;DISP HELP - RETURN

	PUSH	PSW		;SAVE A
	CALL	GET.CB		;GET CMD BUFFER
	POP	PSW		;RESTORE A

	SUI	'0'		;CONVERT DEC TO VAL
	STA	DCM.CMD

	CALL	PUT.CB		;PUT CMD BUFFER

	LXI	H,M.EXC
	CALL	IO.MSG
	LDA	DCM.CMD
	CALL	PR.NIB
	CALL	PR.NL

	MVI	A,DC.EXC	;EXECUTE COMMAND
	OUT	D.PORT

	CALL	W.HLT

	RET

;******************************************************
; FORMAT DISK IN SINGLE DENSITY			      *
;******************************************************
CMD.F:	CALL	F.DSK
	LXI	H,M.RESP
	ORA	A
	CZ	IO.MSG

	RET

;******************************************************
; INITIALIZE DD AND LOAD DCM        		      *
;******************************************************
CMD.I:	CALL	INJECT

	MVI	A,DC.BGN	;BEGIN DD PROCESSOR.
	OUT	D.PORT		;ISSUE  COMMAND.

	LXI	H,M.BGN
	CALL	IO.MSG

	CALL	W.HLT		;WAIT FOR HALT

	RET

;******************************************************
; REQUEST DD MEMORY BANK
;******************************************************
CMD.M:	CALL	G.TKN		;GET COMMAND LINE TOKEN

	LXI	H,M.MB
	CALL	IO.MSG

	MVI	B,DC.MB1
	CPI	'1'
	JZ	..SET

	MVI	A,'0'
	MVI	B,DC.MB0	;BANK 0 IS DEFAULT

..SET:	SUI	'0'
	CALL	PR.NIB

	MOV	A,B
	OUT	D.PORT

	RET

;******************************************************
; DISPLAY DD STATUS REGISTER
;******************************************************
CMD.S:	LXI	H,M.PORT
	CALL	IO.MSG
	MVI	A,D.PORT
	CALL	PR.BYT
	CALL	PR.NL
	LXI	H,M.VAL
	CALL	IO.MSG
	IN	D.PORT
	CALL	PR.BYT
	CALL	PR.NL

	RET

;******************************************************
; DISPLAY BOOT TRACKS
;******************************************************
CMD.T:	MVI	C,0
	CALL	SETTRK
	MVI	C,1
	CALL	SETSEC

..LOOP: LDA	DCM.TRK
	CALL	PR.BYT
	LDA	DCM.SEC
	CALL	PR.BYT
	CALL	PR.NL

	CALL	RESEC		;READ SECTOR
	CALL	PR.SEC		;PRINT SECTOR

	CALL	IO.ST		;PAUSE OUTPUT?
	JZ	..NOPS

	CALL	IO.IN		;GET PAUSE CHARACTER
	CPI	03H		;CTRL-C
	RZ			;RETURN
	CALL	IO.IN		;WAIT FOR RESTART CHARACTER
	
..NOPS:	LDA	SPT		;SECTOR PER TRACK
	MOV	B,A
	LDA	DCM.SEC
	CMP	B
	JZ	..NXT		;NEXT TRACK

	INR	A
	STA	DCM.SEC
	JMP	..LOOP

..NXT:	LDA	DCM.TRK
	INR	A
	CPI	2		;RETURN IF TRACK 2
	RZ

	MOV	C,A
	CALL	SETTRK
	MVI	C,1
	CALL	SETSEC
	JMP	..LOOP


	.PAGE
	.SBTTL  "CP/M ROUTINES"
;******************************************************
; CK.CPM - CHECK IF RUNNING UNDER CP/M. CP/M FLAG IS  *
; SET TRUE (NON-ZERO) IF YES, CLEARED OTHERWISE.      *
;******************************************************

CPM.FLG:.BYTE	0

;*******( INITIALIZE ENTRIES FOR STAND-ALONE )*********

CK.CPM:	XRA	A
	STA	CPM.FLG		;CLEAR CP/M FLAG

;******************************************************
; DETERMINE IF WE'RE UNDER CP/M OR STANDALONE. CP/M   *
; IS ASSUMED IF A JUMP INSTRUCTION IS PRESENT AT THE  *
; CP/M WARM START LOCATION (0) AND FIVE MORE JUMPS    *
; (E.G., A JUMP TABLE) IS PRESENT AT THE JUMP-TO      *
; DESTINATION.                                        *
;******************************************************

	LDA	CPM.WB		;SEE IF JUMP INSTRUCTION PRESENT FOR CP/M
	CPI	JMP.INS
	RNZ			;NO, NOT CP/M

; A JUMP INSTRUCTION IS PRESENT AT THE CP/M WARM BOOT LOCATION (0),
; NOW SEE IF THAT JUMP POINTS TO FIVE MORE JUMPS. IF SO, ASSUME CP/M

	LXI	H,CPM.WB+1	;POINT TO LSB OF JUMP ADDRESS
	MOV	E,M		;E=LOW BYTE OF JUMP
	INX	H
	MOV	D,M		;DE=DESTINATION OF JUMP
	MVI	B,5		;LOOK FOR 5 MORE JUMPS (A JUMP TABLE)

..TEST:	LDAX	D		;A=OPCODE AT JUMP DESTINATION
	SUI	JMP.INS		;ANOTHER JUMP PRESENT?
	RNZ			;NO, NOT CP/M
	INX	D		;MOVE TO NEXT JUMP
	INX	D
	INX	D
	DCR	B
	JNZ	..TEST

; RUNNING UNDER CP/M. SET CP/M FLAG

	MVI	A,1
	STA	CPM.FLG		;SET CP/M FLAG TO NON-ZERO VALUE

	RET

;------------------------------------------------------------------------------
; CPM.EX - IF RUNNING UNDER CP/M, PROMPT USER TO INSERT THE CP/M
; DISK AND THEN WARM-START CP/M. OTHERWISE, JUST RETURN.
;------------------------------------------------------------------------------
CPM.EX:	LDA	CPM.FLG		;RUNNING UNDER CP/M?
	ORA	A		;TEST FOR ZERO
	RZ			;NO, NOT CP/M

	LXI	H,M.CPM		;DISPLAY "INSERT CP/M DISK"	
	CALL	IO.MSG
	CALL	IO.IN		;WAIT FOR A CHARACTER
	CALL	PR.NL
	JMP	CPM.WB		;WARM BOOT CP/M


;******************************************************
; SET DOUBLE D SYSTEM PORT ADDRESS		      *
;******************************************************

D.PORT	==	043H	;DOUBLE D PORT ADDRESS.

;*******( SET USER DOUBLE D BOARD REVISION )***********

TRUE	==	1	;SET TRUE TO LOGIC ONE.
FALSE	==	0	;SET FALSE TO LOGIC ZERO
REV.B	==	FALSE	;SET TRUE FOR REV B BOARDS.
REV.C	==	TRUE	;SET TRUE FOR REV C BOARDS.
MA10	==	FALSE	;TRUE IF MA10 JUMPED (REV-B).

;*******( DEFINE HALT MASK AND BASE ADDRESS)***********

	.IFN	REV.B,[
DS.HLT	==	002H	;STATUS PORT HALT INDICATOR.
DS.ASW	==	00CH	;STATUS PORT ADDR SW MASK.
D.BASE	==	0E400H	;SYSTEM WINDOW BASE ADDRESS
	]

	.IFN	MA10,[
D.BASE	==	0E000H	;SYSTEM WINDOW BASE ADDRESS
	]

	.IFN	REV.C,[
DS.HLT	==	001H	;STATUS PORT HALT INDICATOR.
DS.ASW	==	00EH	;STATUS PORT ADDR SW MASK.
D.BASE	==	0E000H	;SYSTEM WINDOW BASE ADDRESS
	]

;*******( BOOTSTRAP LINKAGE ADDRESS )******************

D.ADDR	==	0040H	;DOUBLE D ADDRESS POINTER.
D.MASK	==	0042H	;DOUBLE D HALT BIT ADDR.

;*******( DCM HARDWARE COMMANDS )**********************

DC.BGN	==	080H	;RESET Z80A AND EXECUTE.
DC.MRQ	==	001H	;REQUEST MEMORY WINDOW.
DC.MRT	==	000H	;RELEASE MEMORY WINDOW.
DC.MB0	==	001H	;SELECT MEMORY BANK 0.
DC.MB1	==	003H	;SELECT MEMORY BANK 1.
DC.EXC	==	002H	;ISSUE DOUBLE D INTERRUPT.

;*******( DCM MEMORY ADDRESSES DEFINED )***************

DD.CBT	==	0370H	;COMMAND BLOCK  (BANK 0).
DD.BUF	==	0380H	;SECTOR BUFFER  (BANK 0).
DD.BFL	==	128	;SECTOR BUFFER LENGTH.
DD.FBF	==	0300H	;FORMAT BUFFER	(BANK 1).
DD.FBL	==	0100H	;FORMAT BUFFER LENGTH.
DD.DPB	==	03A0H	;ID SEC DPB	(BANK 0).
DD.DDF	==	03B1H	;ID SEC FLAGS	(BANK 0).

;*******( DCM COMMANDS )*******************************

DCM.LOG	==	000H	;LOG ON DISKETTE.
DCM.RDS	==	001H	;READ SECTOR.
DCM.WRS	==	002H	;WRITE SECTOR.
DCM.FMT	==	003H	;FORMAT TRACK.
DCM.LST	==	005H	;LIST CHARACTER.
DCM.LCK	==	006H	;LIST STATUS.

;******************************************************
; GET DCM CONTROL BLOCK  			      *
;******************************************************

GET.CB:	MVI	A,DC.MB0	;REQUEST BANK 0
	OUT	D.PORT

	LXI	B,8		;NMBR BYTE TO MOVE.
	LHLD	D.ADDR		;DD SYS ADDRESS.
	LXI	D,DD.CBT	;COMMAND BYTE OFFSET.
	DAD	D		;HL NOW PTS CMND BLK.
	LXI	D,DCM.CMD	;JDDUTIL CMND BLOCK.
	CALL	BLK.MV		;PERFORM BLOCK MOVE.
	RET

;******************************************************
; PUT DCM CONTROL BLOCK  			      *
;******************************************************

PUT.CB:	MVI	A,DC.MB0	;REQUEST BANK 0
	OUT	D.PORT

	LXI	B,8		;NMBR BYTE TO MOVE.
	LHLD	D.ADDR		;DD SYS ADDRESS.
	LXI	D,DD.CBT	;COMMAND BYTE OFFSET.
	DAD	D
	XCHG			;DE NOW PTS CMND BLK.
	LXI	H,DCM.CMD	;JDDUTIL CMND BLOCK.
	CALL	BLK.MV		;PERFORM BLOCK MOVE.
	RET

;******************************************************
; SET COMMAND     				      *
;******************************************************

SETCMD:	MOV	A,C		;MOVE COMMAND.
	STA	DCM.CMD		;SAVE COMMAND.
	RET			;RETURN TO CALLER.

;******************************************************
; SET TRACK NUMBER				      *
;******************************************************

SETTRK:	MOV	A,C		;MOVE TRACK NUMBER.
	STA	DCM.TRK		;SAVE TRACK NUMBER.

	MVI	C,26		;26 SECTORS PER TRACK
	CPI	1		;TRACK 1?
	JNZ	..NOT1		;NOT TRACK 1
	MVI	C,50		;50 SECTORS PER TRACK

..NOT1:	MOV	A,C
	STA	SPT		;SAVE SECTORS PR TRACK
	RET			;RETURN TO CALLER.

;******************************************************
; SET SECTOR NUMBER				      *
;******************************************************

SETSEC:	MOV	A,C		;MOVE SECTOR NUMBER.
	STA	DCM.SEC		;SAVE SECTOR NUMBER.
	RET			;RETURN TO CALLER.

;******************************************************
; WRITE SECTOR - BUFFER TO WRITE IN HL. DCM.TRK AND   *
; DCM.SEC ARE SET UPON ENTRY			      *
;******************************************************

WRSEC:	PUSH	H		;SAVE SOURCE BUFFER
	LXI	B,DD.BFL	;NMBR BYTE TO MOVE.
	LHLD	D.ADDR		;DD SYS ADDRESS.
	LXI	D,DD.BUF	;SECTOR BUFFER OFFSET
	DAD	D		;HL NOW PTS CMND BLK.
	XCHG
	POP	H		;RESTORE SOURCE BUFFER
	CALL	BLK.MV		;PERFORM BLOCK MOVE.
	
;*******( PERFORM WRITE SECTOR )***********************

	MVI	A,DCM.WRS	;DCM WRITE SECTOR
	CALL	DSK.EX		;BIOS WRITE SECTOR.
	RET			;ERROR RETURN.

;******************************************************
; READ SECTOR - READ SECTOR TO BUFFER IN HL. DCM.TRK  *
; AND ; DCM.SEC ARE SET UPON ENTR		      *
;******************************************************

;*******( PERFORM READ SECTOR )***********************

RESEC:	MVI	A,DCM.RDS	;DCM READ SECTOR
	CALL	DSK.EX		;EXECUTE
	RET			;RETURN

;*******( COPY SECTOR BUFFER )************************

	PUSH	H		;SAVE DEST BUFFER
	LXI	B,DD.BFL	;NMBR BYTE TO MOVE.
	LHLD	D.ADDR		;DD SYS ADDRESS.
	LXI	D,DD.BUF	;SECTOR BUFFER OFFSET
	DAD	D
	POP	D		;HL=SRC DE=DEST
	CALL	BLK.MV		;PERFORM BLOCK MOVE.
	
	RET

;******************************************************
; DOUBLE D EXECUTION SUBROUTINE			      *
;******************************************************

;*******( COMMAND BLOCK TO DOUBLE D AND EXEC )*********

DSK.EX:	PUSH	H		;SAVE H
	STA	DCM.CMD		;STORE DCM COMMAND.
	LXI	B,7		;NMBR BYTE TO MOVE.
	LXI	D,DD.CBT	;COMMAND BYTE OFFSET.
	LHLD	D.ADDR		;DD SYS ADDRESS.
	DAD	D		;HL NOW PTS CMND BLK.
	XCHG			;NOW ADDR IN DE.
	LXI	H,DCM.CMD	;BIOS CMND BLOCK.
	CALL	BLK.MV		;PERFORM BLOCK MOVE.
				;DE PTR TO STATUS BYTE
	MVI	A,DC.EXC	;LOAD DD INTERRUPT.
	OUT	D.PORT		;ISSUE DD INTERRUPT.

;*******( WAIT FOR DOUBLE D HALT )*********************

	LDA	D.MASK		;LOAD HALT BIT MASK.
	MOV	B,A		;MASK IN B REGISTER.
..WAIT:	IN	D.PORT		;READ DD STATUS.
	ANA	B		;TEST HALT* FLAG.
	JNZ	..WAIT		;TEST UNTIL HALTED.

;*******( GET DOUBLE D STATUS )************************

	MVI	A,DC.MRQ	;SWITCH DD INTO SYS.
	OUT	D.PORT		;ISSUE HARDWARE CMND.
	XCHG			;EXCHANGE SRC/DSTN.
	MOV	A,M		;STATUS INTO A REG.
	STAX	D		;STORE STATUS BYTE.
	ANA	A		;TEST FOR ERRORS.
	POP	H		;RESTORE H
	RET			;RETURN TO CALLER.


;******************************************************
; DOUBLE D - DCM COMMAND BLOCK BUFFER		      *
;******************************************************

DCM.CMD:.BYTE	0		;DCM COMMAND.
DCM.DRV:.BYTE	0		;DRIVE NUMBER.
DCM.TRK:.BYTE	0		;TRACK NUMBER.
DCM.SEC:.BYTE	0		;SECTOR NUMBER.
DCM.SP0:.BYTE	0		;SPARE BYTE 0.
DCM.CHR:.BYTE	0		;LIST CHARACTER.
DCM.MOD:.BYTE	00000000B	;MODE CONTROLS.
DCM.STS:.BYTE	0		;COMMAND STATUS.

	.PAGE
	.SBTTL  "DCM INJECTOR"
;******************************************************
; THE FOLLOWING SECTION INJECTS THE DCM MODULE INTO   *
; MEMORY BANK 0 ON THE CONROLLER. AFTER THE MODULE    *
; IS INJECTED, IT IS PATCHED TO SKIP LOADING THE      *
; THE CP/M BIOS LOADER FROM TRACK 0 / SECTOR 2.       *
; AFTER THE DCM IS LOADED, THE CONTROLLER SENT A      *
; BEGIN COMMAND TO START DCM.                         *
;******************************************************

INJECT:	IN	D.PORT		;INPUT STATUS PORT.
	ANI	DS.ASW		;MASK FOR ADDR SWS.
	RLC			;POSITION BITS.
	ORI     D.BASE > 8	;OR IN BASE ADDR.
	MOV	H,A		;HIGH BYTE VALUE.
	MVI	L,0		;LOW BYTE VALUE.
	SHLD    D.ADDR		;STORE THE ADDRESS

	MVI	A,DS.HLT	;LOAD HALT BIT MASK.
	STA	D.MASK		;STORE FOR BIOS USE.

	MVI	A,DC.MB0	;REQUEST DD MEM BANK 0.
	OUT	D.PORT		;ISSUE COMMAND.

	LXI	B,IM.END-RST.0	;INJECTION MODULE SIZE.
	XCHG			;D.ADDR HL TO DE.	
	LXI	H,RST.0		;INJECTION MODULE ADDR.
	CALL 	BLK.MV		;BLOCK MOVE.

	LXI	H,M.LOAD	;DISPLAY LOADED MESSAGE
	CALL	IO.MSG

	LHLD    D.ADDR		;GET THE ADDRESS
	CALL	PR.WRD
	CALL	PR.NL

	RET

;******************************************************
; WAIT FOR TASK COMPLETION			      *
;******************************************************

W.HLT:	LXI	H,0FFFFH	;STORE TIMEOUT IN HL

	LDA	D.MASK		;LOAD HALT BIT MASK.
	MOV	B,A		;MASK IN B REGISTER.

..WAIT:	DCX	H
	MOV	A,H
	ORA	A
	JZ	..TO		;TIMEOUT

	IN	D.PORT		;READ DD STATUS.
	ANA	B		;TEST HALT* FLAG.
	JNZ	..WAIT		;TEST UNTIL HALTED.

	LXI	H,M.RESP
	JMP	IO.MSG		;PRINT - RETURN

..TO:	LXI	H,M.NORE
	CALL	IO.MSG

	RET

;******************************************************
; BLOCK MOVE SUBROUTINE (Z80 BLOCK MOVE REGISTERS)    *
;******************************************************

BLK.MV:	MOV	A,M		;GET BYTE.
	INX	H		;INC POINTER
	XCHG			;GET DESTINATION.
	MOV	M,A		;PUT BYTE.
	INX	H		;INC POINTER
	XCHG			;GET SOURCE.
	DCX	B		;ONE LESS TO DO.
	MOV	A,B		;GET HI COUNT.
	ORA	C		;GET LO COUNT.
	JNZ	BLK.MV		;FINISH LOADING.
	RET

	.PAGE
	.SBTTL  "CONSOLE I/O ROUTINES"
;******************************************************
; 88-2SIO CHANNEL A SERIAL INTERFACE EQUATES          *
;******************************************************

SIO.CTL	==	10H		;88-2SIO CONTROL PORT
SIO.DAT ==	11H		;88-2SIO DATA PORT
SIO.TXR ==	02H		;XMIT READY MASK
SIO.RDR ==	01H		;RCV READY MASK

;******************************************************
; IO.INI - RESET AND INITIALIZE 2SIO PORT A IF NOT    *
; RUNNING UNDER CP/M				      *
;******************************************************
IO.INI:	LDA	CPM.FLG		;RUNNING UNDER CP/M?
	ORA	A
	RNZ			;YES, 2SIO ALREADY INITIALIZED

	MVI	A,3		;RESET ACIA
	OUT	SIO.CTL
	MVI	A,015H		;RTS ON, 8N1
	OUT	SIO.CTL
	RET

;******************************************************
; IO.ST - TEST SERIAL PORT A FOR A CHARACTER. RETURN
; WITH A=0 AND Z SET IF NO CHARACTER AVAILABLE.
;******************************************************
IO.ST:	IN	SIO.CTL		;WAIT FOR A CHARACTER
	ANI	SIO.RDR		;SET Z, CLEAR A IF NO CHR		
	RET

;******************************************************
; IO.IN - RETURN A CHARACTER FROM THE SERIAL PORT IN A.
; MSB IS CLEARED.
; Z IS CLEARED UNLESS RECEIVED CHR IS A NULL.
;******************************************************
IO.IN:	CALL	IO.ST		;WAIT FOR A CHARACTER
	JZ	IO.IN

	IN	SIO.DAT		;A = RECEIVED CHARACTER
	ANI	07FH		;STRIP PARITY, CLEAR z UNLESS NULL
	RET

;------------------------------------------------------------------------------
; IO.OUT - SEND THE CHARACTER IN A OUT THE SERIAL PORT. CLOBBERS C.
;------------------------------------------------------------------------------
IO.OUT:	PUSH	B
	MOV	C,A
..LOOP:	IN	SIO.CTL		;WAIT UNTIL OK TO XMIT
	ANI	SIO.TXR
	JZ	..LOOP

	MOV	A,C		
	OUT	SIO.DAT		;SEND THE CHARACTER
	POP	B

	RET

;------------------------------------------------------------------------------
; IO.GET - 
; RETURN NULL TERMINATED STRING IS HL
;------------------------------------------------------------------------------
IO.GET:	LXI	H,..BUF
	MVI	B,..BUFL-1

..LOOP:	CALL	IO.IN		;GET CHARACTER

	CPI	CR
	JNZ	..NOCR

	CALL	IO.OUT
	MVI	A,LF
	CALL	IO.OUT
	XRA	A
	MOV	M,A		;NULL TERMINATOR
	JMP	..DONE
	
..NOCR:	CPI	BS
	JNZ	..NOBS

	MOV	A,B
	CPI	..BUFL-1
	JZ	..LOOP

	MVI	A,BS
	CALL	IO.OUT
	MVI	A,' '
	CALL	IO.OUT
	MVI	A,BS
	CALL	IO.OUT

	DCX	H
	INR	B

	JMP	..LOOP

..NOBS:	MOV	C,A
	MOV	A,B
	ORA	A
	JZ	..LOOP

	MOV	A,C
	CPI	'A'+020H	;CONVERT LOWER TO UPPER CASE
	JC	..UPR
	SUI	020H

..UPR:	MOV	M,A
	CALL	IO.OUT

	INX	H
	DCR	B

	JMP	..LOOP

..DONE:	XRA	A
	MOV	M,A
	LXI	H,..BUF
	RET

..BUFL	==	10		;BUFFER LENGTH
..BUF:	.BLKB	..BUFL		;INPUT BUFFER



;------------------------------------------------------------------------------
; IO.MSG - DISPLAY NULL TERMINATED STRING IN HL.
; CLOBBERS C,H,L. CLEARS A, SETS Z.
;------------------------------------------------------------------------------
IO.MSG:	PUSH	PSW		;SAVE A

..LOOP:	MOV	A,M		;NEXT CHARACTER TO SEND
	ORA	A		;EXIT ON NULL
	JZ	..DONE

	CALL	IO.OUT

	INX	H

	JMP	..LOOP

..DONE:	POP	PSW		;RESTORE A
	RET

;------------------------------------------------------------------------------
; PR.NL - DISPLAY A CR/LF NEW LINE.
; CLOBBERS C.
;------------------------------------------------------------------------------
PR.NL:	MVI	A,CR
	CALL	IO.OUT
	MVI	A,LF
	JMP	IO.OUT


;------------------------------------------------------------------------------
; PR.NIB - DISPLAY VALUE IN A AS A SINGLE DIGIT ASCII-HEXIDECIMAL VALUE.
;------------------------------------------------------------------------------
PR.NIB:	PUSH	PSW
	PUSH	B

CPI	10		;GREATER THAN 9?
	JC	..SEND		;0-9
	ADI	007H

..SEND:	ADI	'0'
	CALL	IO.OUT

	POP	B
	POP	PSW

	RET

;------------------------------------------------------------------------------
; PR.HEX - DISPLAY VALUE IN A AS A TWO DIGIT ASCII-HEXIDECIMAL VALUE.
; CLOBBERS A,B.
;------------------------------------------------------------------------------
PR.BYT:	MOV	B,A		;STORE VALUE IN B
	ANI	0F0H		;HIGH NIBBLE
	RRC
	RRC
	RRC
	RRC
	CALL	PR.NIB		;PRINT THE 16'S DIGIT

	MOV	A,B
	ANI	0FH
	JMP	PR.NIB		;PRINT THE 1'S DIGIT

;------------------------------------------------------------------------------
; PR.WRD - DISPLAY VALUE IN HL AS A FOUR DIGIT ASCII-HEXIDECIMAL VALUE.
; CLOBBERS A,B.
;------------------------------------------------------------------------------
PR.WRD:	PUSH	H		;SAVE HL
	MOV	A,H		;STORE MSB IN A
	CALL	PR.BYT		;PRINT MSB

	MOV	A,L		;STORE LSB IN A
	CALL	PR.BYT		;PRINT LSB
	POP	H
	RET


	.PAGE
	.SBTTL  "MESSAGES"
;------------------------------------------------------------------------------
; PR.SEC - DISPLAY SECTOR BUFFER
;------------------------------------------------------------------------------
PR.SEC:	MVI	A,DC.MB0	;RESELECT DD BANK 0.
	OUT	D.PORT		;ISSUE TO DD HARDWARE.
	LHLD	D.ADDR		;HL TO SECTOR BUFFER
	LXI	D,DD.BUF
	DAD	D
	SHLD	D.ADR
	CALL	PR.BUF
	RET

;------------------------------------------------------------------------------
; PR.BUF - DISPLAY 128 BYTES OF BUFFER POINTED BY D.ADR
;------------------------------------------------------------------------------
PR.BUF: LHLD	D.ADR
	XRA	A

..NEXT:	STA	..CNT
	PUSH    H
	ANI	0FH
	JNZ	..NADR

	CALL	PR.WRD
	MVI	A,':'
	CALL	IO.OUT

..NADR:	MOV     A,M
	MOV	C,A		;SAVE BYTE IN C
	STA	..VAL
	CALL    PR.BYT
	MVI     A,' '
	CALL    IO.OUT

	MOV	A,C
	CPI	' '		;PRINTABLE ASCII CHARACTER?
	JC	..NOA
	CPI	07FH
	JC	..ASC

..NOA:	MVI	C,'.'		;NOT PRINTABLE

..ASC:	LDA     ..CNT
	ANI	0FH

	LXI	H,..ABUF
	MOV	E,A
	MVI	D,0
	DAD	D

	MOV	M,C

	CPI	0FH
	JNZ     ..NONL

	LXI	H,..ABUF
	CALL	IO.MSG

..NONL:	POP     H
	INX     H
	SHLD	D.ADR
	LDA     ..CNT
	INR     A
	CPI     128		;LAST BYTE, RETURN
	RZ

	JMP     ..NEXT

..CNT:	.BYTE	0
..VAL:	.BYTE	0
..ABUF:	.ASCIZ	'................'[CR][LF]

;------------------------------------------------------------------------------
; G.TKN - GET TOKEN FROM CONSOLE INPUT BUFFER IN HL. RETURNS IN A.
;------------------------------------------------------------------------------
G.TKN:

..LOOP: MOV	A,M
	ORA	A
	RZ			;RETURN IF NULL

	INX	H		;QUEUE NEXT CHARACTER

	CPI	' '		;SKIP SPACES
	JZ	..LOOP

	RET			;RETURN TOKEN IN A
	
;------------------------------------------------------------------------------
; G.ADR - GET ADDRESS FROM CONSOLE INPUT BUFFER IN HL. RETURNS IN HL.
;------------------------------------------------------------------------------
G.ADR:	XCHG			;HL TO DE
	LXI	H,0		;ZERO HL

..LOOP: LDAX	D		;GET DIGIT
	ORA	A
	RZ			;RETURN IF NULL

	INX	D		;QUEUE NEXT CHARACTER

	CPI	' '		;SKIP SPACES
	JZ	..LOOP

	CPI	'A'		;CONVERT LETTERS TO UPPERCASE
	JC	..FHN
	ANI	('a'-'A') ^ 0FFH

..FHN:	DAD	H		;MAKE ROOM FOR NEW DIGIT
	DAD	H
	DAD	H
	DAD	H

	CALL	H.CON		;DO THE CONVERSION
	RNC			;ERROR

	ADD	L
	MOV	L,A		;MOVE NEW DIGIT IN

	JMP	..LOOP

;------------------------------------------------------------------------------
; H.CON - CONVERT ASCII HEX DIGIT IN A TO BINARY. RETURNS IN A.
;------------------------------------------------------------------------------
H.CON:	SUI	'0'		;REMOVE ASCII BIAS
	CPI	10
	RC			;IF 0-9 THEN WE'RE DONE

	SUI	9+('A'-'9')	;SHOULD BE 0-5 NOW
	CPI	6		;GAP CHR OR TOO HIGH?
	RNC			;ERROR IF SO

	SUI	0F6H		;ADD 0AH, SET CARRY
	RET
	
;******************************************************
; MESSAGES                                            *
;******************************************************

BELL	==	07H
BS	==	08H
CR	==	0DH
LF	==	0AH

SPT:	.WORD	0
D.ADR:	.WORD	0

M.CRLF:	.ASCIZ	[CR][LF]
M.VER:	.ASCIZ	'JADE "DOUBLE D" UTILITY, VER 1.1'[CR][LF]
M.AUTH:	.ASCIZ	'DELTEC ENTERPRISES LLC 2020'[CR][LF]

M.HELP:	.ASCII	[CR][LF]
	.ASCII	'B     - DISPLAY DCM BUFFERS'[CR][LF]
	.ASCII	'C     - DISPLAY COMMAND BUFFER'[CR][LF]
	.ASCII	'D     - DUMP MEMORY'[CR][LF]
	.ASCII	'E n   - EXECUTE DCM COMMAND n'[CR][LF]
	.ASCII	'F     - FORMAT SSSD FLOPPY'[CR][LF]
	.ASCII	'I     - INITIALIZE DOUBLE D'[CR][LF]
	.ASCII	'M n   - REQUEST MEMORY BANK n'[CR][LF]
	.ASCII	'S     - DISPLAY DD STATUS REGISTER'[CR][LF]
	.ASCII	'T     - DISPLAY BOOT TRACKS'[CR][LF]
	.ASCII	[CR][LF]
	.ASCIZ	'X     - EXIT TO CP/M'[CR][LF]

M.LOAD:	.ASCIZ	[CR][LF]'LOADED DCM AT ADDRESS '
M.BGN:	.ASCIZ	[CR][LF]'RESTARTING DOUBLE D AND DCM...'
M.CMD:	.ASCIZ	[CR][LF]'CMD>'
M.CPM:	.ASCIZ	[CR][LF]'INSERT CP/M DISK INTO DRIVE A, THEN PRESS RETURN...'

M.CB:	.ASCIZ	'DCM COMMAND BLOCK:'[CR][LF][LF]
M.CB0:	.ASCIZ	'DCM COMMAND   : '
M.CB1:	.ASCIZ	'DRIVE NUMBER  : '
M.CB2:	.ASCIZ	'TRACK NUMBER  : '
M.CB3:	.ASCIZ	'SECTOR NUMBER : '
M.CB4:	.ASCIZ	'COMMAND STATUS: '

M.DSL:	.ASCIZ	'DCM STORAGE:'[CR][LF][LF]
M.DSL0:	.ASCIZ	'RETRY ERROR COUNTER    : '
M.DSL1:	.ASCIZ	'BL.CTL DRIVE BITS      : '
M.DSL2:	.ASCIZ	'BL.CTL DRIVE/SIDE BITS : '
M.DSL3:	.ASCIZ	'BL.CTL LAST ISSUED     : '
M.DSL4:	.ASCIZ	'FD179X-02 STATUS VALUE : '
M.DSL5:	.ASCIZ	'PHYSICAL TRACK NUMBER  : '

M.EXC:	.ASCIZ	'EXECUTING COMMAND '
M.EXCH:	.ASCII	'0 - LOGON'[CR][LF]
	.ASCII	'1 - READ SECTOR'[CR][LF]
	.ASCII	'2 - WRITE SECTOR'[CR][LF]
	.ASCII	'3 - FORMAT SECTOR'[CR][LF]
	.ASCII	'4 - READ ADDRESS'[CR][LF]
	.ASCII	'5 - LIST OUTPUT'[CR][LF]
	.ASCII	'6 - LIST STATUS'[CR][LF]
	.ASCIZ	'7 - IDLE'[CR][LF]

M.FE:	.ASCIZ	'FORMAT TRACK ERROR'[CR][LF]
M.FMT:	.ASCIZ	[CR][LF]'FORMATTING SSSD FLOPPY IN DRIVE A...'

M.MB:	.ASCIZ	'REQUESTED MEMORY BANK '

M.PORT:	.ASCIZ	'DISK PROCESSOR STATUS PORT: '

M.RESP:	.ASCIZ	'COMMAND EXECUTED'
M.NORE:	.ASCIZ	'COMMAND TIMED OUT'

M.SEC:	.ASCIZ	[CR][LF]'DCM SECTOR BUFFER:'[CR][LF]

M.VAL:	.ASCIZ	'VALUE: '

	.BLKB	64
STACK	==	.


;******************************************************
;******************************************************
;****************  START OF DCM MODULE  ***************
;******************************************************
;******************************************************

	.TITLE  "DISK CONTROLLER MODULE (DCM2)"
	.SBTTL  "TITLE PAGE"
;******************************************************
;                                                     *
;       PROGRAM ID:     DISK CONTROLLER MODULE        *
;                                                     *
;       VERSION:        2.2  8"     RELEASE 2A        *
;                                                     *
;******************************************************
;                                                     *
;       PRESENTED BY:   JADE COMPUTER PRODUCTS INC.   *
;                       4901 W. ROSECRANS BLVD.       *
;                       HAWTHORNE, CALIFORNIA         *
;                       90250,   U.S.A.               *
;                                                     *
;******************************************************
;                                                     *
;       WRITTEN BY:     STAN KRUMME                   *
;                                                     *
;******************************************************
; THE DISK CONTROLLER MODULE (DCM2) EXECUTES INTERNAL *
; TO THE  JADE DOUBLE D DISK CONTROLLER BOARD.  THIS  *
; PROGRAM PROVIDES A FACILITY TO READ/WRITE DISKETTE  *
; SECTORS AND FORMAT DISKETTE TRACKS  (IN SINGLE AND  *
; DOUBLE DENSITY).  THIS DCM SETS THE PARAMETERS FOR  *
; EACH  DRIVE  DURING THE  "LOG-ON" OPERATION.   THE  *
; FORMAT.COM PROGRAM WRITES AN IDENTIFICATION SECTOR  *
; WHICH PROVIDES  THE NEEDED INFORMATION.   IF  THIS  *
; IDENTITY  SECTOR  IS NOT PRESENT  ON THE DISKETTE,  *
; IT IS  ASSUMED TO BE A  STANDARD  8"  3740 FORMAT.  *
; THIS PROGRAM CONTAINS A 4 WORD  TIMING BLOCK WHICH  *
; SHOULD BE PATCHED TO MATCH THE  USERS DISK DRIVES.  *
; THIS HAS NORMALLY BEEN SET FOR  SHUGART SA800/801.  *
;******************************************************

;******************************************************
; DISK CONTROLLER MODULE IS  COMMAND  COMPATABLE WITH *
; THE FOLLOWING  WESTERN  DIGITAL  CONTROLLER  CHIPS. *
; DOUBLE D  USER SWITCH 0  (U0 OR R0)  MUST BE SET TO *
; INDICATE THE  CONTROLLER CHIP  DATA  BUS  POLARITY. *
;******************************************************
;	CONTROLLER IC    	USER SW0	      *
;	-------------    	--------	      *
;	FD1791-02 (01)    	 CLOSED		      *
;	FD1793-02 (01)    	 OPENED		      *
;	FD1795-02         	 CLOSED		      *
;	FD1797-02         	 OPENED		      *
;******************************************************
; THE FD1795-02 AND FD1797-02 PROVIDE ENHANCED SINGLE *
; DENSITY PERFORMANCE IN THAT  THESE  CHIPS ARE FULLY *
; COMPATABLE WITH FD1771-01 3740 FORMATS.	      *
;******************************************************
	.PAGE
	.SBTTL  "HARDWARE DEFINITION"
;******************************************************
; THE FOLLOWING IS A LIST OF THE INTERNAL I/O ADDRESS *
; ASSIGNMENTS.   THESE PORTS AND CONTROLS CAN ONLY BE *
; USED BY THE ONBOARD Z80A.  THESE PORTS AND CONTROLS *
; ARE NOT IN THE S100 BUS ADDRESS SPACE.              *
;******************************************************

;*******( CONTROLLER PORT ASSIGNMENTS )****************

BL.STS  ==	000H	;BOARD STATUS PORT.
BL.CTL  ==	000H	;BOARD CONTROL PORT.
WD.CMD  ==	004H	;179X COMMAND REGISTER.
WD.STS  ==	004H	;179X STATUS REGISTER.
WD.TRK  ==	005H	;179X TRACK REGISTER.
WD.SEC  ==	006H	;179X SECTOR REGISTOR.
WD.DTA  ==	007H	;179X DATA REGISTER.

;*******( CONTROLLER FUNCTION ASSIGNMENTS )************

XP.STP  ==	008H	;ISSUE STEP PULSE.
XP.MTO  ==	010H	;MOTOR TURN OFF.
XP.IRR  ==	020H	;S100 INT-REQ RESET.
XP.MTX  ==	040H	;MOTOR TIME EXTEND.
XP.DSH  ==	080H	;DATA SYNC HOLD.

;******************************************************
; THE FOLLOWING LIST ASSIGNS EACH  BIT  POSITION  AND *
; FUNCTION OF THE BOARD CONTROL PORT (BL.CTL).        *
;******************************************************

;*******( BIT ASSIGNMENTS )****************************

BC.DSA  ==  00000001B	;DRIVE SELECT A (2*0).
BC.DSB  ==  00000010B	;DRIVE SELECT B (2*1).
BC.DSE  ==  00000100B	;DRIVE SELECT ENABLE.
BC.EIA  ==  00001000B	;EIA SIGNAL LEVEL OUT.
BC.DDE  ==  00010000B	;DOUBLE DENSITY ENABLE.
BC.DAS  ==  00100000B	;DIRECTION AND SIDE
BC.PCA  ==  01000000B	;PRECOMP SELECT A.
BC.PCB  ==  10000000B	;PRECOMP SELECT B.

;*******( FUNCTION ASSIGNMENTS )***********************

BC.DSN  ==  BC.DSA!BC.DSB	;DRIVE NMBR MASK.
BC.SDS  ==  0  			;SINGLE DENSITY.
BC.DDS  ==  BC.DDE		;DOUBLE DENSITY.
BC.PCH  ==  BC.PCA		;PRECOMP - HEAVY.
BC.PCM  ==  BC.PCB		;PRECOMP - MEDIUM.
BC.PCL  ==  BC.PCA!BC.PCB	;PRECOMP - LIGHT.
BC.SD1  ==  BC.DAS		;SELECT SIDE ONE.
BC.INW  ==  BC.DAS		;STEP INWARD DIRC.

;******************************************************
	.PAGE
;******************************************************
; THE FOLLOWING LIST DEFINES EACH BIT AND FUNCTION OF *
; THE BOARD STATUS PORT (BL.STS).                     *
;******************************************************

BS.US0  ==  00000001B	;USER SWITCH 0.
BS.US1  ==  00000010B	;USER SWITCH 1.
BS.TST  ==  00000100B	;TEST MODE SWITCH.
BS.INT  ==  00001000B	;HOST INT REQUEST.
BS.EIA  ==  00010000B	;EIA SIGNAL LEVEL IN.
BS.MOF  ==  00100000B	;MOTOR OFF INIDCATOR.
BS.TSD  ==  01000000B	;TWO SIDED DRIVE FLAG.
BS.DCN  ==  10000000B	;DISK CHANGE INDICATOR.

;******************************************************
; THE FOLLOWING IS A LIST OF  COMMAND CODES ISSUED TO *
; THE 179X-02 DISK CONTROLLER.                        *
;******************************************************

DC.HDL  ==  00011000B	;SEEK/LOAD RW HEAD.
DC.HDU  ==  00010000B	;SEEK/UNLD RW HEAD.
DC.RDS  ==  10001000B	;READ SECTOR.
DC.WRS  ==  10101000B	;WRITE SECTOR.
DC.WRT  ==  11110000B	;WRITE TRACK FORMAT.
DC.RDA  ==  11000000B	;READ TRACK ADDRESS.
DC.STS  ==  11010000B	;SET TYPE 1 STATUS
DC.IFI  ==  11011000B	;FORCED INTERRUPT.

;******************************************************
; THE FOLLOWING LIST  CONTAINS ALL THE  MASKS USED TO *
; TEST THE 179X-02 STATUS CODES  (PORT WD.STS).       *
;******************************************************

DM.RER  ==  10011101B	;READ ERROR TEST.
DM.WER  ==  11111101B	;WRITE ERROR TEST.
DM.FER  ==  11100100B	;FORMAT ERROR TEST.
DM.TK0  ==  00000100B	;TRACK 0 TEST.
DM.HDL  ==  00100000B	;HEAD LOAD TEST.
DM.DNR  ==  10000000B	;DRIVE NOT READY.
DM.LDE  ==  00000100B	;LOST DATA ERROR.

;******************************************************
	.PAGE
	.SBTTL  "MEMORY ASSIGNMENTS"
;******************************************************
; THE FOLLOWING LIST DEFINES INTERNAL MEMORY.         *
;******************************************************

;**********( BASE ADDRESS FOR DCM )********************

BASE    ==      1000H 		;BASE ADDRESS.

;**********( MEMORY BANKS )****************************

BANK.0  ==  BASE+0000H		;BANK 0 DEFINED.
BANK.L  ==  0400H		;BANK LENGTH.
BANK.1  ==  BANK.0+BANK.L	;BANK 1 DEFINED.

;***********( RESTART VECTORS )************************

RST.0   ==   BANK.0+0000H	;RESTART 0.
RST.1   ==   BANK.0+0008H	;RESTART 1.
RST.2   ==   BANK.0+0010H	;RESTART 2.
RST.3   ==   BANK.0+0018H	;RESTART 3.
RST.4   ==   BANK.0+0020H	;RESTART 4.
RST.5   ==   BANK.0+0028H	;RESTART 5.
RST.6   ==   BANK.0+0030H	;RESTART 6.
RST.7   ==   BANK.0+0038H	;RESTART 7.

;**********( INTERRUPT VECTORS )***********************

HR.INT  ==  RST.7		;MASKABLE.
NM.INT  ==  BANK.0+0066H	;NON MASKABLE.

;**********( I/O COMMUNICATION  )**********************

IO.BLK  ==  BANK.0+0370H	;I/O BLOCK BEGIN.
TP.STK  ==  IO.BLK+0000H	;TOP OF STACK.
CMD.BK  ==  IO.BLK+0000H	;COMMAND BLOCK.
BUF.BG  ==  IO.BLK+0010H	;SECTOR BUFFER.
FMT.BG  ==  BANK.1+0300H	;FORMAT BUFFER.
FMT.PS  ==  FMT.BG+0008H	;FORMAT PROGRAM.

;******************************************************
	.PAGE
	.SBTTL  "MACRO DEFINITIONS"
;******************************************************
; WAIT IS A "RESTART" TO THE TIMER SUBROUTINE ENTRY.  *
; THIS SUBROUTINE PROVIDES MOST OF THE TIMING USED BY *
; THE DOUBLE D CONTROLLER.                            *
;******************************************************

	.DEFINE WAIT = [
	 RST    1]

;******************************************************
; ASSEMBLER DIRECTIVES                                *
;******************************************************

	.PABS		;ABSOLUTE ADDRESSING.
	.PHEX 		;INTEL HEX OBJECT FILE.
	.XLINK		;NO LINKAGE REQUIRED.

;******************************************************
; TENTH MILLESECOND TIMING CONSTANTS / 0.2 MS FOR 5"  *
;******************************************************

TMR.FC  ==      0019H	;TIMING CONSTANT, FIRST PASS.
TMR.NC  ==      001CH	;TIMING CONSTANT, REPEAT PASS.

;******************************************************
;      BAUD RATE GENERATOR - TIMING CONSTANTS         *
;******************************************************
;   BAUDRATE      US/BIT       8" SYS     5 " SYS     *
;   --------      ------       ------     -------     *
;    19200         52.1           9         N.A.      *
;     9600        104.2          25           9       *
;     4800        208.3          57          25       *
;     2400        416.6         121          57       *
;     1200        833.3         248         121       *
;      600       1666.6         N.A.        248       *
;******************************************************

BAUD.C  ==      25.	;BAUD RATE CONSTANT 9600 8".

;******************************************************
; ERROR RECOVERY VALUES                               *
;******************************************************

RTY.SK  ==      5	;REPOSITION R/W HEAD ON RETRY.
RTY.LS  ==      9	;LAST REPEATED RETRY.

TRK.OB  ==      26	;AT FIRST THIRD TRACK OF DISK.
TRK.IB  ==      52	;AT SECOND THIRD TRACK.

;******************************************************
	.PAGE
	.SBTTL  "BASE PAGE"
;******************************************************
; THE FOLLOWING AREA IS THE INITIAL START JUMP TABLE. *
; THE FIRST JUMP IS EXECUTED WHEN THE ONBOARD Z80A IS *
; RESET.   THE  SECOND JUMP IS THE  DCM ENTRY  FROM A *
; BOOTSTRAP LOADER.  THIS ENTRY ASSUMES  DCM HAS BEEN *
; LOADED INTO DOUBLE D BANK 1  BY THE LOADER ROUTINE. *
; THE LAST  TWO BYTES  HOLD THE  JUMP ADDRESS USED BY *
; RESTART INTERRUPT ROUTINE AT BANK 0 + 0380H.        *
;******************************************************

	.LOC    RST.0 		;MODULE BEGINNING.

	 JMP    INIT.U		;JDDUTIL ENTRY
	 JMP    INIT.B+BANK.L	;BOOTSTRAPPED ENTRY.
HR.VEC: .WORD   X.CUTE		;HOST INTERRUPT VECTOR.

;******************************************************
; THE FOLLOWING SUBROUTINE IS THE ENTRY POINT FOR THE *
; DISK CONTROLLER TIMING MODULE. THIS MODULE PROVIDES *
; DELAYS WHICH ARE MULTIPLES OF 100 MICROSECONDS. THE *
; CONTENTS OF REGISTER PAIR  DE  DETERMINES THE TOTAL *
; PERIOD.   (DELAY = (DE )* 100 MICROSECONDS).   THIS *
; SUBROUTINE IS ENTERED BY THE MACRO "WAIT".          *
;******************************************************

	.LOC    RST.1 		;TIMING ENTRY POINT.

	MVI     B,TMR.FC	;FIRST TICK CONSTANT.
	DJNZ    .		;AUTO DEC UNTIL ZERO.
	JMP     TICK.E		;JUMP TO TICK ENTRY.

;******************************************************
; THE FOLLOWING SECTION IS THE DISK DRIVE TIMING AREA.*
; THE TIMES ARE SET FOR THE SHUGART SA800.  THIS AREA *
; SHOULD BE MODIFIED FOR THE END USERS DRIVE TYPE.    *
;******************************************************

;*******( TIMING VALUES IN 0.1 MS )********************

	.LOC    RST.2

TM.HLD: .WORD   350	;HEAD ENGAGE TIME.
TM.STP: .WORD   80	;STEPPER INTERVAL.
TM.ALS: .WORD   80	;AFTER LAST STEP.
TM.MTO: .WORD   1 	;MOTOR START UP.

;******************************************************
	.PAGE
;******************************************************
; THE FOLLOWING SUBROUTINE PROVIDES THE R/W HEAD CNTL *
; FUNCTION.  AS THE  FD179X-02  DOES  NOT  OFFER THIS *
; EXPLICIT COMMAND, THE SEEK COMMAND (TYPE-1) IS USED *
; WITH THE HEAD LOAD BIT SET / RESET. THE DESTINATION *
; TRACK IS SET EQUAL TO THE TRACK REGISTER  TO BYPASS *
; THE  FD179X-02 STEPPING FUNCTION.   PLEASE REFER TO *
; THE FD179X-02 FLOW-CHART FOR TYPE-1 COMMANDS.       *
;******************************************************

EX.HCF: POP     Y		;RETURN ADDR IN REG Y.
	IN      WD.TRK		;READ PRESENT TRACK.
	OUT     WD.DTA		;SET DESTINATION TRK.
	MOV     A,B   		;LOAD TYPE-1 COMMAND.
	XRA     C		;INVERT (1791-01).
	OUT     WD.CMD		;ISSUE COMMAND.
	JMPR    .		;WAIT FOR INTERRUPT.

;******************************************************
; THE  FOLLOWING  SUBROUTINE  UPDATES  THE  FD179X-02 *
; STATUS PORT TO REFLECT CURRENT TYPE-1 STATUS CODES. *
; NOTE: THIS IS A  TYPE-4  COMMAND WITH  NO INTERRUPT *
; CONDITIONS SET.                                     *
;******************************************************

EX.STS: MVI     A,DC.STS	;LOAD SET-STATUS CMND.
	XRA     C		;INVERT (1791-01).
	OUT     WD.CMD		;ISSUE COMMAND.
	XTHL   			;PAUSE FOR FD179X-02.
	XTHL   			;PAUSE MORE.
	XTHL   			;PAUSE STILL MORE.
	XTHL   			;PAUSE LAST TIME.
	IN      WD.STS		;INPUT STATUS PORT.
	XRA     C		;INVERT (1791-01).
	RET			;RETURN TO USER.

;******************************************************
; THE  FOLLOWING SECTION IS THE  MASKABLE  INTERRUPT  *
; ROUTINE.  THIS ROUTINE IS EXECUTED WHEN RESTARTING  *
; THE Z80 FROM A HALT.   THE FUNCTIONS ARE RESET THE  *
; DOUBLE D INT REQ FLIP-FLOP,  PUT  THE  INTERRUPTED  *
; ADDR IN REG DE, AND JUMP ADDRESS AT "HR.VEC".       *
;******************************************************

	.LOC    HR.INT		;HOST INTERRUPT ADDR.
	IN      XP.IRR		;RESET INTERRUPT REQ FF
	POP     D		;PURGE INTERRUPTED ADDR
	LHLD    HR.VEC		;LOAD RETURN ADDRESS
	PCHL   			;JUMP RETURN ADDRESS

;******************************************************
	.PAGE
	.SBTTL  "COMMAND FETCH AND BRANCH"
;******************************************************
; THE FOLLOWING SECTION HALTS EXECUTION OF THE        *
; ONBOARD Z80A PROCESSOR.  DURING THIS TIME THE HOST  *
; SYSTEM CAN SWITCH THE CONTROLLER MEMORY INTO THE    *
; S100 BUS FOR STATUS CHECK, SETTING COMMAND BLOCK,   *
; AND SECTOR DATA TRANSFERS.                          *
;******************************************************

FETCH:  EI		;ENABLE INTERRUPT START
	HLT   		;HALT ON-BOARD PROCESSOR

;******************************************************
; THE FOLLOWING SECTION GAINS CONTROL AFTER THE DISK  *
; CONTROLLER IS INTERRUPTED FROM THE HALT CONDITION.  *
; THIS SECTION BRACNCHES TO THE INDIVIDUAL COMMAND    *
; ROUTINES.  THE COMMAND TABLE CONTAINS THE ADDRESSES *
; FOR THIS DISTRIBUTION.                              *
;******************************************************

X.CUTE: LDA     CB.CMD		;LOAD HOST COMMAND.
	ANI     CM.MSK		;MASK ANY OPTIONS.
	ADD     A		;GET 2*A VALUE.
	MVI     D,0   		;ZERO D REGISTER.
	MOV     E,A   		;DE NOW TABLE OFFSET.
	LXI     H,CM.DTA	;LOAD TABLE ADDRESS.
	DAD     D		;NOW POINTS TO ENTRY.
	MOV     E,M   		;LOW ORDER ADDR LOAD.
	INX     H		;POINT TO NEXT BYTE.
	MOV     D,M   		;HI ORDER ADDRESS.
	XCHG   			;BRANCH ADDR IN HL.
	PCHL   			;BRANCH TO COMMAND.

;******************************************************
; THE FOLLOWING AREA IS  THE  COMMAND  DRIVER  TABLE. *
; EACH ENTRY POINTS TO THE COMMAND DRIVER ROUTINE.    *
;******************************************************

CM.DTA  ==      .		;COMMAND TABLE.

..CM0A: .WORD   $.LGON		;LOG-ON DRIVE.
..CM1A: .WORD   $.READ		;READ SECTOR.
..CM2A: .WORD   $.WRIT		;WRITE SECTOR.
..CM3A: .WORD   $.FORM		;FORMAT TRACK.
..CM4A: .WORD   $.ADDR		;READ ADDRESS.
..CM5A: .WORD   $.LIST		;LIST OUTPUT.
..CM6A: .WORD   $.LSTT		;LIST STATUS.
..CM7A: .WORD   $.IDLE		;BACKGROUND.

CM.MSK  ==      007H  		;COMMAND MASK.

;******************************************************
	.PAGE
	.SBTTL  "NON MASKABLE INTERRUPT"
;******************************************************
; THE FOLLOWING SECTION IS THE NON-MASKABLE INTERRUPT *
; ROUTINE.  UPON 179X-02 COMMAND TERMINATION THE Z80  *
; RECIEVES A NON-MASKABLE INTERRUPT.  THE STATUS PORT *
; IS INTERROGATED AND SAVED (SV.STS).  REGISTER IY    *
; CONTAINS THE RETURN ADDRESS.                        *
;******************************************************

	.LOC    NM.INT		;NON-MASKABLE INT.

WD.INT: IN      WD.STS		;GET 179X STATUS.
	XRA     C		;INVERT (1791).
	STA     SV.STS		;SAVE STATUS.
	XTIY   			;EXCHANGE (SP)<>IY.
	RETN   			;RETURN AT OLD IY.

;******************************************************
; THIS SECTION IS THE REMAINDER OF THE TIMING         *
; SECTION ENTERED BY A RESTART 1.  SEE THAT SECTION   *
; FOR THE DESCRIPTION.                                *
;******************************************************

TICK.R: MVI     B,TMR.NC	;NORMAL TICK CONSTANT.
	DJNZ    .		;AUTO DEC UNTIL ZERO.
TICK.E: DCX     D		;DECREMENT AMOUNT.
	MOV     A,D   		;GET HIGH ORDER.
	ORA     E		;AND LOW ORDER.
	NOP			;TIMING ADJUST.
	NOP			;TIMING ADJUST.
	JRNZ    TICK.R		;REPEAT UNTIL ZERO.
	RET			;RETURN TO USER.

;******************************************************
	.PAGE
	.SBTTL  "COMMAND CONTROLLERS"
;******************************************************
;  $.READ IS THE READ-SECTOR COMMAND CONTROLLER.      *
;******************************************************

$.READ: CALL    SELECT		;SELECT DRIVE ROUTINE.
	CALL    SEEK  		;SEEK TRACK, SET CTLS.
	JRNZ    ..EXIT		;DRIVE OR SEEK ERROR.
	CALL    RD.SEC		;READ DISK SECTOR.
..EXIT: JMP     FETCH 		;GET NEXT COMMAND.
;******************************************************
;  $.WRIT IS THE WRITE-SECTOR COMMAND CONTROLLER.     *
;******************************************************

$.WRIT: CALL    SELECT		;SELECT DRIVE ROUTINE.
	CALL    SEEK  		;SEEK TRACK, SET CTLS.
	JRNZ    ..EXIT		;DRIVE OR SEEK ERROR.
	CALL    WR.SEC		;WRITE DISK SECTOR.
..EXIT: JMP     FETCH 		;GET NEXT COMMAND.

;******************************************************
;  $.FORM IS THE FORMAT-TRACK COMMAND CONTROLLER.     *
;******************************************************

$.FORM: CALL    SELECT		;SELECT DRIVE NUMBER.
	LDA     CB.SEC		;LOAD FORMAT FLAGS.
	MOV     DV.FLG(X),A	;RESET DRIVE FLAGS.
	CALL    SEEK  		;SEEK TRACK, SET CTLS.
	JRNZ    ..EXIT		;DRIVE OR SEEK ERROR.
	CALL    WR.TRK		;WRITE DISK TRACK.
..EXIT: JMP     FETCH 		;GET NEXT COMMAND.

;******************************************************
;  $.LGON IS THE DRIVE LOG-ON COMMAND CONTROLLER      *
;******************************************************

$.LGON: CALL    SELECT		;SELECT DRIVE NUMBER.
	XRA     A		;ZERO REGISTER A.
	STA     CB.TRK		;SET TRACK AT 0.
	INR     A		;NOW A REG IS 1.
	STA     CB.SEC		;SET SECTOR TO ID.
	CALL    SEEK  		;SEEK TRACK, SET CTLS.
	JRNZ    ..EXIT		;DRIVE OR SEEK ERROR.
	CALL    RD.SEC		;READ ID SECTOR.
	JRNZ    ..EXIT		;READ ERROR DETECTED.
	CALL    LOG.ON		;LOG ON DISK DRIVE.
..EXIT: JMP     FETCH 		;GET NEXT COMMAND.

;******************************************************
	.PAGE
;******************************************************
;  $.ADDR IS THE READ-ADDRESS COMMAND CONTROLLER.     *
;******************************************************

$.ADDR: MVI     A,0FFH		;LOAD ALL ONES.
	STA     CB.STS		;STORE ERRORS.
	JMP     FETCH 		;NOT IMPLEMENTED.

;******************************************************
;  $.LIST IS A LIST DEVICE COMMAND CONTROLLER.        *
;******************************************************

$.LIST: CALL    LST.OT		;SEND CHAR TO LIST.
	JMP     FETCH 		;GET NEXT COMMAND.

;******************************************************
;  $.LSTT CHECKS LIST DEVICE STATUS                   *
;******************************************************

$.LSTT: IN      BL.STS		;GET BOARD STATUS.
	ANI     BS.EIA		;TEST READY BIT.
	JZ      ..EXIT		;IF ZERO GOTO EXIT.
	MVI     A,0FFH		;LOAD ALL ONES.
..EXIT: STA     CB.STS		;STORE STATUS.
	JMP     FETCH 		;GET NEXT COMMAND.

;******************************************************
;  $.IDLE IS THE IDLE COMMAND CONTROLLER.             *
;******************************************************

$.IDLE: IN      BL.STS		;INPUT BOARD STATUS.
	ANI     BS.INT		;CHECK HOST INTERRUPT.
	JRZ     $.IDLE		;REPEAT IDLE CHECK.
	IN      XP.IRR		;RESET INTERRUPT REQ.
	JMP     FETCH 		;GET NEXT COMMAND.

;******************************************************
	.PAGE
	.SBTTL  "DRIVE SELECTION ROUTINE"
;******************************************************
; THE FOLLOWING  SUBROUTINE  SELECTS  REQUESTED DRIVE *
; NUMBER 0-3 (A-D). BEFORE DRIVE SELECTION, THE DRIVE *
; MOTOR CONTROL STATE IS TESTED AND ENABLED IF NEEDED.*
; INDEX REGISTER X  IS SET  POINTING TO THE REQUESTED *
; DRIVE TABLE ENTRY.  THE DRIVE IS THEN SELECTED.     *
;******************************************************

;*******( MOTOR CHECK ROUTINE )************************

SELECT:	IN      BL.STS		;BOARD LEVEL STATUS.
	ANI     BS.MOF		;CHECK MOTOR STATE.
	IN      XP.MTX		;START OR EXTEND TIMER.
	JRZ     ..CKDV		;IF WAS ON, NO STARTUP.
	LDED    TM.MTO		;MOTOR STARTUP DELAY.
	WAIT   			;PROGRAMMABLE DELAY.

;*******( NEW SELECTION CHECK )************************

..CKDV: LDA     CB.DRV		;LOAD DRIVE NUMBER.
	ANI     BC.DSN		;GET DRIVE NUMBER.
	CMP     DV.NBR(X)	;CURRENTLY SELECTED?
	RZ			;RETURN IF DRV SAME.

;*******( SET TABLE POINTER )**************************

	LXI     X,DV.TBL	;DRIVE TABLE ADDR.
	LXI     D,DV.DES	;DRIVE ENTRY SIZE.
..NEXT: DCR     A		;DECREMENT DRV NO.
	JM      ..DSLT		;IF S=1 EXIT.
	DADX    D		;POINT NEXT DRIVE.
	JMPR    ..NEXT		;TRY THIS DRIVE.

;*******( DESELECT OLD DRIVE )*************************

..DSLT: MVI     B,DC.HDU	;LOAD UNLOAD R/W HEAD.
	CALL    EX.HCF		;FD179X-02 TYPE 1 CMND.
	LDA     SV.CTL		;BL.CTL LAST ISSUED.
	ANI     #BC.DSE		;DRIVE SELECT DSBLD.
	OUT     BL.CTL		;ISSUE DESELECT.

;*******( SELECT NEW DRIVE )***************************

	ANI     #BC.DSN		;STRIP OFF DRIVE NMBR.
	ORA     DV.NBR(X)	;OR IN NEW DRIVE NMBR.
	OUT     BL.CTL		;OUTPUT DRIVE NMBR.
	ORI     BC.DSE		;SET DRV ENABLE BIT.
	OUT     BL.CTL		;ENABLE NEW DRIVE.
	ANI     BC.DSN!BC.DSE	;NOW JUST DRIVE ENBLED.
	STA     SV.DRV		;SAVE DRIVE SELECT.
	RET			;DRIVE IS SELECTED.

;******************************************************
	.PAGE
	.SBTTL  'SEEK TRACK ROUTINE'
;******************************************************
; THE FOLLOWING SUBROUTINE PERFORMS THE TRACK SEEK    *
; OPERATION.  AFTER THE SEEK OPERATION, THE DENSITY   *
; AND PRE-COMPENSATION CONTROLS ARE SET.              *
;******************************************************

;*******( HEAD LOADING )*******************************

SEEK:   CALL    EX.STS		;GET DRIVE STATUS.
	ANI     DM.HDL!DM.DNR	;CHECK HEAD AND READY.
	JM      ..NRDY		;DRIVE NOT READY EXIT.
	JNZ     ..DTAS		;BYPASS IF HEAD LOADED.
	MVI     B,DC.HDL	;HEAD-LOAD COMMAND.
	CALL    EX.HCF		;EXEC FD179X-02 TYPE 1.
	LDED    TM.HLD		;SET HEAD-LOAD DELAY.
	WAIT   			;PROGRAMMABLE DELAY.

;*******( DETERMINE TRACK NMBR AND SIDE )**************

..DTAS: IN      BL.STS		;INPUT BOARD STATUS.
	ANI     BS.TSD		;TEST DISK SIDES FLAG.
	LDA     CB.TRK		;GET LOGICAL TRACK NO.
	MOV     L,A   		;SAVE LOGICAL TRACK.
	JNZ     ..NDBL		;SKIP IF NOT DBL SIDED.
	RAR			;DIV BY 2 DOUBLE SIDE.
..NDBL: STA     PH.TRK		;STORE PHYSICAL TRACK.
	MOV     H,A   		;SAVE PHYSICAL NUMBER.
	LDA     SV.DRV		;LOAD DRV NMBR ENABLED.
	JRNC    ..SID0		;SKIP NEXT IF SIDE 0.
	ORI     BC.SD1		;OR IN SELECT SIDE 1.
..SID0: STA     SV.DAS		;STORE DRV AND SIDE EN.
	MOV     D,A   		;SAVE DRV AND SIDE EN.
	MOV     A,H   		;LOAD PHYSICAL NUMBER.
	SUB     DV.TRK(X)	;TRACK OFFSET TESTED.
	JRNZ    ..SEEK		;IF OFFTRACK, DO SEEK.
	IN      BL.STS		;INPUT BOARD STATUS.
	ANI     BS.TSD		;TEST DISK SIDES FLAG
	JZ     ..DSID 		;GOTO DOUBLE SIDE CTL.

;*******( SINGLE SIDED DISKETTE )**********************

..SSID: MOV     A,DV.CTL(X)	;GET PREVIOUS CONTROLS.
	JMP     ..EXIT		;SET CONTROLS / EXIT.
;*******( DRIVE NOT READY EXIT )***********************

..NRDY: MVI     A,CS.DNR	;DRIVE NOT READY FLAG.
	STA     CB.STS		;STORE ERROR STATUS.
	ANA     A		;SET NOT ZERO FLAG.
	RET			;ERROR EXIT.

;******************************************************
	.PAGE
;*******( DISKETTE IS DOUBLE SIDED )*******************

..DSID: MOV     A,H   		;GET PHYSICAL TRK NMBR.
	ANA     A		;TEST IF TRACK ZERO.
	JRZ     ..DCTL		;IF ZERO, RESET CNTLS.
	MOV     A,DV.CTL(X)	;LOAD OLD DRV CTLS.
	ANI     #BC.SD1		;STRIP OFF SIDE CMND.
	ORA     D		;OR IN NEW SIDE CMND.
	JMP     ..EXIT		;SET CONTROLS / EXIT.

;*******( SET DIRECTION AND COUNT STEPS )**************

..SEEK: PUSH    PSW   		;SAVE REG A AND FLGS.
	LDED    TM.SAW		;STEP AFTER WRITE.
	WAIT   			;PROGRAMMABLE DELAY.
	POP     PSW   		;RESTORE A AND FLGS.
	JRC     ..SOUT		;IF CARRY STEP OUT.
..SIN:  MOV     L,A   		;MOVE OFFSET TO L.
	LDA     SV.DRV		;DRIVE SELECT BITS.
	ORI     BC.INW		;SET STEP DIRC IN.
	OUT     BL.CTL		;OUTPUT CONTROL.
	JMPR    ..STEP		;GOTO STEP ROUTINE.
..SOUT: NEG			;COMPLEMENT OFFSET.
	JM      ..HOME		;BETTER HOME DRV.
	MOV     L,A   		;MOVE OFFSET TO L.
	LDA     SV.DRV		;DRIVE SELECT BITS.
	OUT     BL.CTL		;SET DIRECTION OUT.
..STEP: IN      XP.STP		;ISSUE STEP PULSE.
	LDED    TM.STP		;STEP DELAY TIME.
	WAIT   			;PROGRAMMABLE DELAY.
	DCR     L		;DECREMENT STEPS.
	JRNZ    ..STEP		;REPEAT OPERATION.
	LDA     SV.DAS		;LOAD DRV AND SIDE.
	OUT     BL.CTL		;OUTPUT CONTROL.
	LDED    TM.ALS		;MORE AFTER LAST STP.
	WAIT   			;PROGRAMMABLE DELAY.

;******************************************************
	.PAGE
;*******( CONTROL DETERMINATION )**********************

..DCTL: LDA     CB.TRK		;LOAD LOGICAL TRACK.
	CPI     1		;COMPARE AGAINST 1.
	JRC     ..SDEN		;TRACK 0 IS SDENS.
	MVI     A,DF.DTD	;DATA TRK DENS FLG.
	JNZ     ..DTST		;GOTO TEST DENSITY.
	MVI     A,DF.T1D	;TRACK 1 DENS FLAG.
..DTST: ANA     DV.FLG(X)	;TEST DENSITY FLAGS.
	JZ      ..SDEN		;IF ZERO, THEN SDENS.
..DDEN: LDA     PH.TRK		;LOAD PHYSICAL TRACK.
	CPI     TRK.OB		;TEST OUTSIDE BOUNDRY.
	MVI     B,BC.DDS!BC.PCL	;DDENS AND LOW PRECOMP.
	JRC     ..CTLS		;SET FOR OUTSIDE TRKS.
	CPI     TRK.IB		;TEST INSIDE BOUNDRY.
	MVI     B,BC.DDS!BC.PCM	;DDENS AND MED PRECOMP.
	JRC     ..CTLS		;JUMP TO CONTROLS SET.
	MVI     B,BC.DDS!BC.PCH	;DDENS AND MAX PRECOMP.
	JMPR    ..CTLS		;JUMP TO CONTROLS SET.        
..SDEN: MVI     B,BC.SDS!BC.PCL	;SDEN AND PC-LOW.

;*******( SET CONTROL VALUES AND EXIT )****************

..CTLS: LDA     SV.DAS		;GET DRIVE AND SIDE.
	ORA     B		;SET PRECOMP AND DENS.
	MOV     DV.CTL(X),A	;SAVE CONTROLS FOR DRV.
..EXIT: OUT     BL.CTL		;OUTPUT CONTROLS.
	STA     SV.CTL		;SAVE THESE CONTROLS.
	LDA     PH.TRK		;PHYSICAL TRACK NMBR.
	MOV     DV.TRK(X),A	;SET DRIVE TABLE.
	LDA	CB.TRK		;LOGICAL TRACK NMBR.
	XRA     C		;INVERT (1791-01).
	OUT     WD.TRK		;SET TRACK REGISTER.
	XRA     A		;SET ZERO FLAG.
	RET			;RETURN TO CALLER.

;*******( CALIBRATE TRACK NUMBER )*********************

..HOME: CALL    HOME.D		;HOME SELECTED DRIVE.
	RNZ			;EXIT SEEK, HOME BAD.
	JMP     ..DTAS		;NOW SEEK TRACK.

;******************************************************
	.PAGE
	.SBTTL  "READ SECTOR DRIVER"
;******************************************************
; RD.SEC IS THE SUBROUTINE THAT INTERACTS WITH THE    *
; 179X-02 DURING READ SECTOR OPERATIONS. THIS SECTION *
; INITIATES THE DISK TRANSFER, SERVICES THE CONTROLLER*
; CHIP DURING DATA TRANSFER, AND TERMINATES OPERATION *
; WHEN FINISHED.  ERROR DETECTION IS IMPLEMENTED AND  *
; RETRIES ARE EXRCUTED IF DATA ERRORS ARE DETECTED.   *
;******************************************************

;*******( INITIALIZE READ OPERATION )******************

RD.SEC: XRA     A		;ZERO A REGISTER.
	STA     ERR.CT		;ZERO ERROR COUNT.
	LDA     CB.SEC		;LOAD SECTOR NMBR.
	XRA     C		;INVERT (1791-01).
	OUT     WD.SEC		;SET SECTOR REGISTER.
..RTRY: LXI     Y,..NMI		;LOAD NMI VECTOR.
	LHLD    BUF.ST		;BUFFER START.
	MVI     A,DC.RDS	;READ SECTOR COMMAND.
	XRA     C		;INVERT (1791-01).
	OUT     WD.CMD		;ISSUE READ COMMAND

;*******( DATA TRANSFER LOOP )*************************

..REPT: IN      XP.DSH		;HOLD FOR DATA
	IN      WD.DTA		;INPUT DATA.
	XRA     C		;INVERT (1791-01).
	MOV     M,A   		;PUT INTO BUFFER
	INX     H		;BUMP BUFF POINTER
	JMPR    ..REPT		;GO FOR ANOTHER

;*******( CHECK STATUS )*******************************

..NMI:  ANI     DM.RER		;TEST FOR ERRORS.
	STA     CB.STS		;SAVE READ STATUS.
	RZ				;RETURN COMPLETE.
	CALL    CHK.RT		;CHECK ABOUT RETRYS.
	JRZ     ..RTRY		;PERFORM RETRY.
	RET			;ERROR RETURN.

;******************************************************
	.PAGE
	.SBTTL  "WRITE SECTOR DRIVER"
;******************************************************
; WR.SEC  SUBROUTINE  INTERACTS  WITH  THE  FD179X-02 *
; DURING WRITE SECTOR OPERATIONS. THIS SECTION        *
; INITIATES THE DISK TRANSFER, SERVICES THE CONTROLLER*
; CHIP, AND TERMINATES THE OPERATION. ERROR DETECTION *
; IS IMPLEMENTED.                                     *
;******************************************************

;*******( INITIALIZE WRITE OPERATION )*****************

WR.SEC: XRA     A		;ZERO REGISTER.
	STA     ERR.CT		;SET ERROR COUNTER.
	LDA     CB.SEC		;LOAD SECTOR NMBR.
	XRA     C		;INVERT (1791-01).
	OUT     WD.SEC		;SET SECTOR REGISTER.
..RTRY: LXI     Y,..NMI		;SET NMI RETURN.
	LHLD    BUF.ST		;BUFFER START.
	MVI     A,DC.WRS	;LOAD WRITE SECTOR CMD.
	XRA     C		;INVERT (1791-01).
	OUT     WD.CMD		;ISSUE COMMAND.

;*******( DATA TRANSFER LOOP )*************************

..REPT: IN      XP.DSH		;HOLD FOR DATA REQ.
	MOV     A,M   		;GET DATA BYTE.
	XRA     C		;INVERT (1791-01).
	OUT     WD.DTA		;OUTPUT DATA BYTE.
	INX     H		;INCREMENT BUFF POINTER
	JMPR    ..REPT		;REPEAT SEQUECE

;*******( CHECK STATUS )*******************************

..NMI:  ANI     DM.WER		;TEST FOR WRITE ERRORS.
	STA     CB.STS		;STORE WRITE STATUS.
	RZ			;RETURN COMPLETE.
	CALL    CHK.RT		;CHECK ABOUT RETRYS.
	JRZ     ..RTRY		;PERFORM RETRY.
	RET			;ERROR RETURN.

;******************************************************
	.PAGE
	.SBTTL  "WRITE TRACK DRIVER"
;******************************************************
; WR.TRK IS THE SUBROUTINE WHICH INITIATES A FORMAT   *
; TRACK COMMAND (WRITE-TRACK 179X-02 TYPE 3).  THE    *
; FORMATTING BYTE STREAM IS PROVIDED BY A PROGRAM     *
; WHICH MUST BE PRESENT IN THE FORMAT BUFFER.         *
;******************************************************

;*******( INITIALIZE WRITE TRACK )*********************

WR.TRK: LXI     Y,..NMI		;LOAD NMI VECTOR.
	MVI     A,DC.WRT	;WRITE TRACK CMND.
	XRA     C			;INVERT (1791-01).
	OUT     WD.CMD		;ISSUE COMMAND.
	JMP     FMT.PS		;FORMAT PROG START.

;*******( CHECK COMPLETION STATUS )********************

..NMI:  ANI     DM.FER		;TEST FOR ERRORS.
	MOV     B,A   		;HOLD THIS STATUS.
	IN      BL.STS		;INPUT BOARD STATUS.
	ANI     BS.TSD		;TEST TWO SIDED BIT.
	MOV     A,B   		;RESTORE STATUS TO A.
	JRNZ    ..EXIT		;NOT ZERO IS ONE SIDED.
	ORI     CS.TSD		;OR IN TWO SIDED FLAG.
..EXIT: STA     CB.STS		;STORE FORMAT STATUS.
	SHLD    CW.LNG		;DISPLAY TRAIL BYTES.
	RET			;RETURN TO USER.

;******************************************************
	.PAGE
	.SBTTL  "RETRY CONTROLLER"
;******************************************************
; CHK.RT IS THE SUBROUTINE USED BY RD.SEC AND         *
; WR.SEC TO COUNT RETRY OPERATIONS AND PERFORM A      *
; RE-SEEK OPERATION WHEN NEEDED.                      *
;******************************************************

;*******( CHECK IF RECOVERABLE )***********************

CHK.RT: ANI     DM.DNR		;TEST NOT READY BIT.
	JRNZ    ..EXIT		;CAN NOT RECOVER.
	LDA     CB.MOD		;GET COMMAND MODE.
	ANI     CM.NRT		;NO RETRYS CHECK.
	JRNZ    ..EXIT		;SHOULD NOT RECOVER.
	IN      XP.MTX		;MOTOR TIME EXTEND.

;*******( RECORD RETRY )*******************************

	LDA     ERR.CT		;GET ERROR COUNT.
	INR     A			;INCREMENT.
	STA     ERR.CT		;STORE NEW COUNT.
	CPI     RTY.SK		;SHOULD TRY SEEK?
	JRNZ    ..CKLS		;IF NOT, CHECK LAST.

;*******( REPOSITION R/W HEAD )************************
	
	CALL    HOME.D		;HOME SELECTED DRIVE.
	JRNZ    ..EXIT		;ERROR EXIT.
	CALL    SEEK  		;SEEK DESIRED TRACK.

;*******( HOLD READ GATE FOR 3/4 REVOLUTION )**********

..CKLS: CPI     RTY.LS		;WAS THIS THE LAST.
	JRZ     ..STNZ		;ERROR LAST RETRY.
	LDED    TM.PLD		;PHASE LOCK DELAY.
	WAIT   			;PROGRAMMABLE DELAY.
	XRA     A		;CLEAR FOR RETRY.
	RET			;TRY AGAIN EXIT.

;*******( ERROR EXIT )*********************************

..STNZ: INR     A		;SET NOT ZERO.
..EXIT: RET			;ERROR EXIT.

;******************************************************
	.PAGE
	.SBTTL  "RESTORE TRACK 0"
;******************************************************
; HOME.D IS THE SUBROUTINE THAT STEPS THE DISK DRIVE  *
; R/W HEAD OUTWARD UNTIL THE TRACK 0 FLAG BECOMES     *
; ACTIVE OR 255 STEPS HAVE BEEN ISSUED.               *
;******************************************************

;*******( RESTORE R/W HEAD )***************************

HOME.D: LDA     SV.DRV		;LOAD DRV NMBR ENABLED.
	OUT     BL.CTL		;ISSUE CONTROLS.
	STA     SV.CTL		;AND SAVE THESE.
	MVI     L,255 		;SET STEP COUNTER.
..STEP: CALL    EX.STS		;CHECK DISK STATUS.
	ANI     DM.TK0		;INSPECT TRACK 0 FLG.
	JRNZ    ..EXIT		;IF SET, GO ..EXIT.
	DCR     L		;DECREMENT STEP COUNT.
	JRZ     ..EROR		;ERROR IF 255 STEPS.
	IN      XP.STP		;ISSUE STEP PULSE.
	LDED    TM.STP		;LOAD STEP DELAY.
	WAIT   			;PROGRAMMABLE DELAY.
	JMPR    ..STEP		;TRY STEPPING AGAIN.

;*******( DRIVE IS RESTORED )**************************

..EXIT: LDED    TM.ALS		;TIME AFTER LAST STEP.
	WAIT   			;PROGRAMMABLE DELAY.
	MOV     A,C   		;GET WD TRK 0 VALUE.
	OUT     WD.TRK		;ZERO TRACK REGISTER.
	XRA     A		;ZERO A REG, SET FLAG.
	MOV     DV.TRK(X),A	;SET TRACK VALUE.
	RET			;RETURN TO CALLER.

;*******( TRACK 0 NOT FOUND )**************************

..EROR: MVI     A,CS.HME	;LOAD HOME ERROR FLAG.
	STA     CB.STS		;STORE ERROR STATUS.
	ANA     A		;SET RETURN FLAGS.
	RET			;RETURN TO CALLER.

;******************************************************
	.PAGE
	.SBTTL  "LOG-ON DISKETTE"
;******************************************************
; LOG.ON IS THE SUBROUTINE THAT READS THE IDENTITY    *
; SECTOR FROM THE DISKETTE AND MAKES THE NEEDED       *
; ENTRYS INTO THE DRIVE TABLE.  THE SECTOR DATA IS    *
; ALSO LEFT IN THE SECTOR BUFFER FOR BIOS TO FINISH   *
; THE LOG-ON OPERATION.                               *
;******************************************************

;*******( CHECK JADE IDENTITY )************************

LOG.ON: LXI     D,JADEID	;ID ADDRESS LOADED.
	LXI     H,ID.LBL	;SECTOR ID ADDRESS.
	MVI     B,ID.SZE	;ID LABEL SIZE.
..CKJI: LDAX    D		;GET CHARACTER.
	CMP     M		;CHECK AGAINST DISK.
	JRNZ    ..3740		;IF DIFFERENT: 3740.
	INX     D		;CHECK NEXT.
	INX     H		;CHECK NEXT.
	DJNZ    ..CKJI		;REPEAT OPERATION.

;*******( LOG-ON JADE FORMAT )*************************

	LDA     ID.FLG		;SIDE AND DENSITIES.
	MOV     DV.FLG(X),A	;STORE IN DRIVE TBL.
	RET			;RETURN TO CALLER.

;*******( ASSUME 3740 FORMAT )*************************
..3740: MVI     A,ID.FLD	;SIDE AND DENSITIES.
	MOV     DV.FLG(X),A	;STORE IN DRIVE TBL.
	RET			;RETURN TO CALLER.

;******************************************************
	.PAGE
	.SBTTL  "CHARACTER TRANSMISSION"
;******************************************************
; THE FOLLOWING ROUTINE SENDS ONE 8 BIT CHARACTER OUT *
; THE EIA LEVEL TRANSMISSION BIT.  SET FOR BAUD RATE. *
;******************************************************

;*******( SET UP FOR TRANSMISSION )********************

LST.OT: IN      BL.STS		;GET BOARD STATUS.
	ANI     BS.EIA		;TEST LIST READY BIT.
	JZ      LST.OT		;WAIT READY (JZ/JNZ).
	LDA     CB.CHR		;GET LIST CHARACTER.
	CMA			;COMPLEMENT ACUMULATOR.
	MOV     E,A   		;CHARACTER TO E REG.
	LDA     SV.CTL		;LAST CONTROLS USED.

;*******( SEND THE START BIT )*************************

	STC			;SET CARRY BIT.
	CALL    BIT.OT		;OUTPUT START BIT.
	NOP			;EQUALIZE TIMING.
	NOP			;EQUALIZE TIMING.
	MVI     D,8   		;NUMBER OF DATA BITS.

;*******( SEND EACH DATA BIT )*****( 39 CYCLE LOOP )***

..DATA: RRCR    E		;ROTATE E REG RIGHT.
	CALL    BIT.OT		;SEND ONE DATA BIT.
	DCR     D		;ONE LESS BIT TO DO.
	JNZ     ..DATA		;REPEAT IF MORE BITS.

;*******( SEND STOP BIT )******************************

	NOP			;EQUALIZE TIMING.
	ANA     A		;CLEAR CARRY FLAG.
	CALL    BIT.OT		;SEND STOP BIT.
	RET			;RETURN TO CALLER.

;*******( SET EIA BIT AND OUTPUT )****( 39 CYCLES )****

BIT.OT: JC      ..ONE 		;IF CARRY, SET TO ONE.
	RES     3,A   		;ZERO EIA IN ACUM REG.
	JMP     ..OUT 		;GO TO OUTPUT PORT.
..ONE:  SET     3,A   		;SET EIA IN ACUM.
	JMP     ..OUT 		;EQUALIZE TIMING.
..OUT:  OUT     BL.CTL		;SEND ACUM TO PORT.

;*******( SET DELAY FOR BAUDRATE )*********************

	MVI     B,BAUD.C	;LOAD TIMING CSNT.
	DJNZ    .		;DELAY FOR BIT.
	RET			;RETURN TO LST CALL.
;******************************************************
	.PAGE
	.SBTTL  "CONSTANTS AND VARIABLES"
;******************************************************
; PROGRAM STORAGE LOACTIONS                           *
;******************************************************

BUF.ST: .WORD   BUF.BG 	;BUFFER STARTING ADDRESS.
ERR.CT: .BYTE   0	;RETRY ERROR COUNTER.

SV.DRV: .BYTE   0	;BL.CTL DRIVE BITS.
SV.DAS: .BYTE   0	;BL.CTL DRIVE AND SIDE BITS.
SV.CTL: .BYTE   0	;BL.CTL LAST ISSUED.
SV.STS: .BYTE   0	;FD179X-02 STATUS VALUE.

PH.TRK: .BYTE   0	;PHYSICAL TRACK NUMBER.

DD.DSL	==	ERR.CT-BASE	;DCM STORAGE LOCATION.

;******************************************************
; TIMING VALUES - 0.1 MS INCREMENTS                   *
;******************************************************

TM.PLD: .WORD   1200 	;PHASE LOCK RECOVERY.
TM.SAW: .WORD   10	;STEP AFTER WRITING.
TM.SDD  ==      24	;SIDE SELECT DELAY.

;******************************************************
; DISKETTE IDENTITY LABEL                             *
;******************************************************

JADEID: .ASCII  "Jade DD "	;DISKETTE ID LABEL.
ID.SZE  ==      (. - JADEID) 	;ID LABEL SIZE.

ID.LBL  ==  BUF.BG+0000H	;ID SECTOR LABEL.
ID.BLK  ==  ID.LBL+0020H	;ID BLOCK AREA.
ID.FLG  ==  ID.BLK+0011H	;DISKETTE FLAGS.
ID.FLD  ==  00000000B 		;3740 FLAGS.

;******************************************************
	.PAGE
	.SBTTL  "DRIVE TABLE"
;******************************************************
; DRIVE TABLE AREA DEFINED                            :
;******************************************************

;*******( DRIVE TABLE ENTRIES )************************

DV.NBR  ==      0	;CURRENT DRIVE NUMBER.
DV.TRK  ==      1	;CURRENT TRACK NUMBER.
DV.FLG  ==      2	;SIDE AND DENSITY FLAGS
DV.CTL  ==      3	;LAST CONTROLS USED.

;*******( DRIVE TABLE AREA )***************************

DV.TBL  ==      .	;DRIVE TABLE BEGGINING ADDRESS.
DT.DE0: .BYTE   0,255,DF.DFL,0C4H	;DRIVE 0.
DT.DE1: .BYTE   1,255,DF.DFL,0C5H	;DRIVE 1.
	.BYTE   2,255,DF.DFL,0C6H	;DRIVE 2.
	.BYTE   3,255,DF.DFL,0C7H	;DRIVE 3.
DT.DED: .BYTE   4,255,0,0   		;DUMMY.

DV.DES  ==      DT.DE1-DT.DE0	;EACH DRIVE ENTRY SIZE.

;*******( FLAG BIT DEFINITIONS )***********************

DF.T1D  ==  00000010B	;TRACK 1 DENSITY (1 = DOUBLE).
DF.DTD  ==  00000100B	;DATA TRACKS DENSITY (1 = DD).
DF.TSD  ==  00001000B	;TWO SIDED ( 1 = TWO SIDES).
DF.DFL  ==  DF.T1D	;DEFAULT FLAGS.

;******************************************************
	.PAGE
	.SBTTL  "I/O COMMUNICATION BLOCK"
;******************************************************
; THE FOLLOWING AREA IS DEFINED AS THE COMMAND BLOCK. *
; THIS AREA IS RESERVED FOR SPECIFICATION BY THE HOST *
; SYSTEM FOR ALL DISK OPERATIONS.   CONTROLLER STATUS *
; AT COMPLETION OF OPERATION IS PRESENT IN THIS AREA. *
;******************************************************

	.LOC    CMD.BK	;COMMAND BLOCK.

CB.CMD: .BYTE   0	;CONTROL COMMAND.
CB.DRV: .BYTE   0	;DRIVE NUMBER.  
CB.TRK: .BYTE   0	;LOGICAL TRACK NUMBER.
CB.SEC: .BYTE   0	;SECTOR NUMBER.
CB.FFG: .BYTE   0	;FORMAT FLAGS.
CB.CHR: .BYTE   0	;EIA CHARACTER.
CB.MOD: .BYTE   0	;MODE SELECTS.
CB.STS: .BYTE   0	;CONTROLLER STATUS.

CW.LAD: .WORD   0	;LOAD ADDRESS.
CW.LNG: .WORD   0	;LOAD LENGTH

;*******( MODE BIT DEFINITIONS )***********************

CM.NRT  ==  10000000B	;NO RETRYS ( = 1 ).

;*******( STATUS BIT DEFINITIONS )*********************

CS.DNR  ==  10000000B	;DRIVE NOT READY.
CS.WRP  ==  01000000B	;WRITE PROTECTED.
CS.BT5  ==  00100000B	;NOT ASSIGNED.
CS.RNF  ==  00010000B	;RECORD NOT FOUND.
CS.CRC  ==  00001000B	;CRC ERROR.
CS.LDE  ==  00000100B	;LOST DATA ERROR.
CS.HME  ==  00000010B	;DRIVE HOME ERROR.
CS.TSD  ==  00000001B	;TWO SIDES FLAG (FORMAT).
;******************************************************
	.PAGE
	.SBTTL  "DCM INITIALIZE"
;******************************************************
; THIS SECTION RESIDES IN THE DCM SECTOR BUFFER. THIS *
; SECTION MOVES DCM FROM BANK 1 DOWN TO BANK 0.   THE *
; C REGISTER IS SET FOR 1791-01 OR 1793-01.  THE LAST *
; OPERATION IS TO  READ THE  BIOS  LOADER  SECTOR  TO *
; OVERLAY THIS INITIALIZATION SEQUENCE.   BIOS LOADER *
; THEN READ BIOS INTO BANK 1 AND HALTS.               *
;******************************************************

;*******( EXECUTES IN BANK 1 )*************************

	.LOC    BUF.BG		;RESIDES IN BUFFER.
INIT.B: LXI     B,BANK.L	;SET BANK LENGTH.
	LXI     D,BANK.0	;SET DESTINATION.
	LXI     H,BANK.1	;SET SOURCE ADDR.
	LDIR   			;MOVE BLOCK.
	JMP     ..DOWN		;JUMP TO NEW IMAGE.

;*******( NOW IN BANK 0, SET INT MODE )****************

..DOWN: LXI     SP,TP.STK	;SET STACK PNTR.
	IM1			;INTERRUPT MODE 1.

;*******( SET 1791-01/1793-01 )************************

	MVI     C,0   		;LOAD C REG ZERO.
	IN      BL.STS		;BOARD STATUS.
	ANI     BS.US0		;TEST USER SW #1.
	JRNZ    LD.BLT		;SW OPEN - 1793.
	MVI     C,0FFH		;SW CLOSED - 1791.

;*******( OVERLAY WITH BIOS LOADER TRANSIENT )*********

LD.BLT: LXI     X,DT.DED	;INIT DRIVE TBL.
	MVI     A,2   		;BIOS LOADER SECTOR.
	STA     CB.SEC		;SET SECTOR VALUE.
	IN      XP.MTX		;MOTOR TIME EXTEND.
	LXI     H,BUF.BG	;SET RETURN ADDR.
	PUSH    H		;PUSH INTO STACK.
	JMP     RD.SEC		;GET BIOS LOADER.

;******************************************************

;*******( JDDUTIL INIT. SET INT MODE )****************

INIT.U: LXI     SP,TP.STK	;SET STACK PNTR.
	IM1			;INTERRUPT MODE 1.

;*******( SET 1791-01/1793-01 )************************

	MVI     C,0   		;LOAD C REG ZERO.
	IN      BL.STS		;BOARD STATUS.
	ANI     BS.US0		;TEST USER SW #1.
	JRNZ    ..DONE		;SW OPEN - 1793.
	MVI     C,0FFH		;SW CLOSED - 1791.

..DONE:	LXI	X,DT.DED	;INIT DRIVE TABLE.
	MVI     A,1   		;BIOS LOADER SECTOR.
	STA     CB.SEC		;SET SECTOR VALUE.
	MVI	A,0C4H		;SELECT DRIVE 0
	OUT	BL.CTL
	JMP	FETCH

IM.END	==	.		;END OF INJECTION MOD.


;******************************************************
;******************************************************
;**************  START OF FORMAT MODULE  **************
;******************************************************
;******************************************************

	.TITLE	"FORMAT - JADE DOUBLE D"
	.SBTTL	'TITLE PAGE'
;******************************************************
;						      *
;	PROGRAM ID:	FORMAT                        *
;						      *
;******************************************************
;						      *
;	PRESENTED BY:	JADE COMPUTER PRODUCTS	      *
;			4901 W. ROSECRANS BLVD.	      *
;			HAWTHORNE, CALIFORNIA	      *
;			90250,   U.S.A.		      *
;						      *
;******************************************************
;						      *
;	VERSION:	CP/M 2.2   RELEASE 2A         *
;						      *
;******************************************************
;						      *
;	WRITTEN BY:	STAN KRUMME		      *
;						      *
;******************************************************
; FORMAT  IS A SYSTEM UTILITY  WHICH PROVIDES A MEANS *
; TO WRITE A  SINGLE OR  DOUBLE DENSITY FORMAT ON ANY *
; OF DRIVES  A THROUGH D.  THIS UTILITY ALSO PROVIDES *
; A  COPY-SYSTEM-TRACKS  FEATURE.  THIS  IS  A USEFUL *
; FUNCTION FOR  FORMAT AS THE  SYSTEM  TRACKS  CAN BE *
; WRITTEN WITH THE  OPERATING SYSTEM  WHEN FORMATTED. *
; FORMAT IS 8080/8085/Z80 COMPATABLE.                 *
;******************************************************

;******************************************************
; FORMAT INJECTION MODULES ARE COMMAND COMPATABLE WITH*
; THE FOLLOWING  WESTERN  DIGITAL  CONTROLLER  CHIPS. *
; DOUBLE D  USER SWITCH 0  (U0 OR R0)  MUST BE SET TO *
; INDICATE THE  CONTROLLER CHIP  DATA  BUS  POLARITY. *
;******************************************************
;	CONTROLLER IC    	USER SW0	      *
;	-------------    	--------	      *
;	FD1791-02 (01)    	 CLOSED		      *
;	FD1793-02 (01)    	 OPENED		      *
;	FD1795-02         	 CLOSED		      *
;	FD1797-02         	 OPENED		      *
;******************************************************

;******************************************************
; RELEASE 2A:   SINGLE AND DOUBLE SIDED DRIVES CAN BE *
; FORMATED.  INSPECTION OF TWO SIDED* SIGNAL FROM THE *
; DISK DRIVE DETERMINES NUMBER OF SIDES.  WITH DOUBLE *
; SIDED DISKETTES, BOTH SIDES FORM ONE LOGICAL DISK.  *
; EACH DOUBLE DENSITY TRACK NOW CONTAINS 50 SECTORS.  *
;******************************************************

	.PAGE
	.SBTTL	"PROGRAM EQUATES"
;******************************************************
; DRIVER MODULE DEFINITIONS			      *
;******************************************************

EOM	==	'$'		;STRING TERMINATOR.
TRK.0	==	0		;TRACK 0.
TRK.1	==	1		;TRACK 1.
TRK.2	==	2		;TRACK 2.
SEC.SZ	==	128		;128 BYTES PER SECTOR.
ID.SEC	==	1		;ID SECTOR NUMBER.
REBOOT	==	0		;REBOOT ADDRESS.
BS.PTR	==	0001H		;WARM ADDR POINTER.
NO.LOG	==	01H		;REQUEST NO LOG-ON.
FT.ERC	==	11111110B	;FORMAT ERROR MASK.
FT.TSM	==	00000001B	;TWO SIDED MASK.

;******************************************************
; INJECTION MODULE DEFINITIONS			      *
;******************************************************

FMT.EA	==	1700H		;FORMAT EXEC ADDRESS.
WD.TRK	==	005H		;DOUBLE D TRACK PORT.
WD.DTA	==	007H		;DOUBLE D DATA PORT.
;XP.DSH	==	080H		;DATA SYNC HOLD PORT.
ZEROS	==	00000000B	;ALL ZERO BYTE.
ONES	==	11111111B	;ALL ONES BYTE.

;******************************************************
; WORKING VARIABLES				      *
;******************************************************

TF.INX:	.WORD	0F80H	;TRANSFER INDEX.
TF.PTR:	.WORD	0	;LIST ADDRESS POINTER.
TF.DIR:	.BYTE	0	;TRANSFER DIRECTION.
MSG.SV:	.WORD	0	;MESSAGE SAVE ADDRESS.
FT.STS:	.BYTE	0	;FORMAT STATUS SAVE.
TS.FLG:	.BYTE	0	;TWO SIDED DRIVE FLAG.
TRK.NO:	.BYTE	0	;TRACK NUMBER HOLD.
TRK.MX:	.BYTE	0	;LAST TRACK LIMIT.
SEC.NO:	.BYTE	0	;SECTOR NUMBER HOLD.
F.FLAG:	.BYTE	0	;FORMAT FLAG (DCM).
SYS.RF:	.BYTE	0	;SYSTEM TRACK READ FLAG.
SV.NBR:	.BYTE	0	;SEL.DV TEMP STORAGE.
FD.NBR:	.BYTE	0	;FORMAT DRIVE NUMBER.


	.PAGE
	.SBTTL	'SYSTEM TRACKS TRANSFER LIST'
;******************************************************
; THE FOLLOWING  IS A  LIST OF  SYSTEM TRACK  SECTORS *
; USED  BY THE  TRNSFR  SUBROUTINE.   THERE ARE THREE *
; ENTRIES PER SECTOR.   1ST IS TRACK NUMBER.   2ND IS *
; SECTOR NUMBER.   3RD IS MEMORY LOAD OFFSET.         *
;******************************************************
; SECTORS 2 THRU 26 ARE TRANSFERED ON TRACK 0. SECTOR *
; 1 IS NOT TRANSFERED,  THIS IS THE  IDENTITY SECTOR. *
; TRACK 0 SECTOR ARE LOCATED IN SEQUENCIAL ORDER,  SO *
; THIS LIST IS STAGGERED.   SECTORS  1  THRU  48  ARE *
; TRANSFERED ON TRACK 1.			      *
;******************************************************

TK0	==	0		;DEFINE TRACK 0.
TK1	==	1		;DEFINE TRACK 1.
EOL	==	0FFH		;DEFINE END OF LIST.

;******************************************************

ST.LST:	.BYTE	TK0,04,04,TK0,08,08,TK0,12,12,TK0,16,16
	.BYTE	TK0,20,20,TK0,24,24,TK0,02,02,TK0,06,06
	.BYTE	TK0,10,10,TK0,14,14,TK0,18,18,TK0,22,22
	.BYTE	TK0,26,26,TK0,05,05,TK0,09,09,TK0,13,13
	.BYTE	TK0,17,17,TK0,21,21,TK0,25,25,TK0,03,03
	.BYTE	TK0,07,07,TK0,11,11,TK0,15,15,TK0,19,19
	.BYTE	TK0,23,23

	.BYTE	TK1,01,27,TK1,02,28,TK1,03,29,TK1,04,30
	.BYTE	TK1,05,31,TK1,06,32,TK1,07,33,TK1,08,34
	.BYTE	TK1,09,35,TK1,10,36,TK1,11,37,TK1,12,38
	.BYTE	TK1,13,39,TK1,14,40,TK1,15,41,TK1,16,42
	.BYTE	TK1,17,43,TK1,18,44,TK1,19,45,TK1,20,46
	.BYTE	TK1,21,47,TK1,22,48,TK1,23,49,TK1,24,50
	.BYTE	TK1,25,51,TK1,26,52,TK1,27,53,TK1,28,54
	.BYTE	TK1,29,55,TK1,30,56,TK1,31,57,TK1,32,58
	.BYTE	TK1,33,59,TK1,34,60,TK1,35,61,TK1,36,62
	.BYTE	TK1,37,63,TK1,38,64,TK1,39,65,TK1,40,66
	.BYTE	TK1,41,67,TK1,42,68,TK1,43,69,TK1,44,70
	.BYTE	TK1,45,71,TK1,46,72,TK1,47,73,TK1,48,74

	.BYTE	EOL		;END OF LIST.

;******************************************************
	.PAGE
	.SBTTL	"IDENTITY SECTORS"
;******************************************************
; JADE SINGLE DENSITY - IDENTITY SECTORS	      *
;******************************************************

IDS.SS:	.ASCII	"Jade DD S Sided S Density Format "

	.LOC	IDS.SS+20H	;LOCATE CP/M 2.2 DPB.
	.WORD	26		;SECTORS PER TRACK.
	.BYTE	3		;BLOCK SHIFT FACTOR.
	.BYTE	7		;BLOCK MASK.
	.BYTE	0		;EXM.
	.WORD	26*75/8-1	;DISK SIZE - 1.
	.WORD	63		;DIRECTORY MAXIMUM.
	.BYTE	11000000B	;ALLOC 0.
	.BYTE	0		;ALLOC 1.
	.WORD	16		;CHECK SIZE.
	.WORD	2		;TRACK OFFSET.

	.LOC	IDS.SS+30H	;LOCATE DCM BLOCK.
	.BYTE	0		;NOT USED.
SD.FLG:	.BYTE	00000010B	;DISKETTE FLAGS.

	.LOC	IDS.SS+SEC.SZ	;EXTEND FULL SECTOR.

;******************************************************

IDS.DS:	.ASCII	"Jade DD D Sided S Density Format "

	.LOC	IDS.DS+20H	;LOCATE CP/M 2.2 DPB.
	.WORD	26		;SECTORS PER TRACK.
	.BYTE	4		;BLOCK SHIFT FACTOR.
	.BYTE	15		;BLOCK MASK.
	.BYTE	1		;EXM.
	.WORD	26*152/16-1	;DISK SIZE - 1.
	.WORD	63		;DIRECTORY MAXIMUM.
	.BYTE	10000000B	;ALLOC 0.
	.BYTE	0		;ALLOC 1.
	.WORD	16		;CHECK SIZE.
	.WORD	2		;TRACK OFFSET.

	.LOC	IDS.DS+30H	;LOCATE DCM BLOCK.
	.BYTE	0		;NOT USED.
	.BYTE	00001010B	;DISKETTE FLAGS.

	.LOC	IDS.DS+SEC.SZ	;EXTEND FULL SECTOR.

;******************************************************
	.PAGE
;******************************************************
; JADE DOUBLE DENSITY - IDENTITY SECTORS	      *
;******************************************************

IDS.SD:	.ASCII	"Jade DD S Sided D Density Format "

	.LOC	IDS.SD+20H	;LOCATE CP/M 2.2 DPB.
	.WORD	50		;SECTORS PER TRACK.
	.BYTE	4		;BLOCK SHIFT FACTOR.
	.BYTE	00001111B	;BLOCK MASK.
	.BYTE	1		;EXM.
	.WORD	50*75/16-1	;DISK SIZE - 1.
	.WORD	63		;DIRECTORY MAXIMUM.
	.BYTE	10000000B	;ALLOC 0.
	.BYTE	0		;ALLOC 1.
	.WORD	16		;CHECK SIZE.
	.WORD	2		;TRACK OFFSET.

	.LOC	IDS.SD+30H	;LOCATE DCM BLOCK.
	.BYTE	0		;NOT USED.
DD.FLG:	.BYTE	00000110B	;DISKETTE FLAGS.

	.LOC	IDS.SD+SEC.SZ	;EXTEND TO FULL SIZE

;******************************************************

IDS.DD:	.ASCII	"Jade DD D Sided D Density Format "

	.LOC	IDS.DD+20H	;LOCATE CP/M 2.2 DPB.
	.WORD	50		;SECTORS PER TRACK.
	.BYTE	5		;BLOCK SHIFT FACTOR.
	.BYTE	31		;BLOCK MASK.
	.BYTE	3		;EXM.
	.WORD	50*152/32-1	;DISK SIZE - 1.
	.WORD	127		;DIRECTORY MAXIMUM.
	.BYTE	10000000B	;ALLOC 0.
	.BYTE	0		;ALLOC 1.
	.WORD	32		;CHECK SIZE.
	.WORD	2		;TRACK OFFSET.

	.LOC	IDS.DD+30H	;LOCATE DCM BLOCK.
	.BYTE	0		;NOT USED.
	.BYTE	00001110B	;DISKETTE FLAGS.

	.LOC	IDS.DD+SEC.SZ	;EXTEND TO FULL SIZE

;******************************************************
; FORMAT DISK IN SINGLE DENSITY			      *
;******************************************************

F.DSK:	LXI	H,M.FMT
	CALL	IO.MSG

	LDA	SD.FLG		;LOAD SDENS FLAGS.
	STA	DCM.SEC		;STORE FORMAT FLAGS.
	MVI	A,77-1		;SINGLE SIDED MAX.
	STA	TRK.MX		;SET MAX TRACK.

	MVI	A,TRK.0		;TRACK 0.
	STA	DCM.TRK		;SET TRACK NUMBER.
	CALL	..FSD		;FORMAT TRACK SDENS.
	JNZ	TRK.ER		;JUMP ERROR DETECTED.

	MVI	A,TRK.1		;TRACK 1
	STA	DCM.TRK		;SET TRACK NUMBER.
	CALL	..FDD		;FORMAT TRACK DDENS.
	JNZ	TRK.ER		;JUMP ERROR DETECTED.

	MVI	A,TRK.2		;TRACK 2.
	STA	DCM.TRK		;SET TRACK NUMBER.

..REPT:	CALL	..FSD		;FORMAT TRACK SDENS.
	JNZ	TRK.ER		;JUMP ERROR DETECTED.
	CALL	TRK.NX		;SET FOR NEXT TRACK.
	JZ	..REPT		;FORMAT NEXT TRACK.

..ID:	CALL	WSD.ID		;WRITE SDENS ID SECTOR.
	RET			;DONE

..FSD:	LXI	H,FT3740	;LOAD INJECTION ADDR.
	JMP	..FTRK		;GO SET DMA ADDR.

..FDD:	LXI	H,FTJ50D	;LOAD INJECTION ADDR.

..FTRK:	SHLD	..FINJ		;SAVE INJECTION ADDR.
	MVI	A,DC.MRQ	;SWITCH DD INTO SYSTEM.
	OUT	D.PORT
	MVI	A,DC.MB1	;SELECT DD BANK 1.
	OUT	D.PORT		;ISSUE HARDWARE CMND.
	LXI	B,DD.FBL	;FORMAT PROG SIZE.
	LHLD	D.ADDR		;DD SYSTEM ADDRESS.
	LXI	D,DD.FBF	;DD FORMAT BUF OFFSET.
	DAD	D		;HL NOW DD FBUF ADDR.
	XCHG			;DE NOW DD FBUF ADDR.
	LHLD	..FINJ		;FORMAT PROGRAM ADDR.
	CALL	BLK.MV		;BLOCK MOVE ROUTINE.
	MVI	A,DC.MB0	;RESELECT DD BANK 0.
	OUT	D.PORT		;ISSUE TO DD HARDWARE.
	MVI	A,DCM.FMT	;LOAD FORMAT TRK CMND.
	CALL	DSK.EX		;CALL DISK EXECUTIVE.
	LDA	DCM.STS		;LOAD FORMAT STATUS.
	RET			;FORMAT EXIT.

..FINJ:	.WORD	0		;FORMAT INJECTION ADDR

;******************************************************
; NEXT TRACK SELECT ROUTINE			      *
;******************************************************

TRK.NX:	LDA	TRK.MX		;LOAD MAX TRACK NMBR.
	MOV	B,A		;SAVE IN REG B.
	LDA	DCM.TRK		;GET THIS TRACK NO.
	CMP	B		;CHECK FOR LAST TRACK.
	JZ	..DONE		;JUMP IF LAST TRACK.
	INR	A		;GET NEXT TRACK.
	STA	DCM.TRK		;STORE NEXT TRACK.
	XRA	A		;SET ZERO FLAG.
	RET			;RETURN TO CALLER.

..DONE:	MVI	A,ONES		;SET ALL ONES.
	ANA	A		;SET FLAG NOT ZERO.
	RET			;LAST TRACK EXIT.


;******************************************************
; FORMAT TRACK ERROR				      *
;******************************************************

TRK.ER:	LXI	H,M.FE		;FORMAT ERROR MSG ADDR.
	CALL	IO.MSG		;DISPLAY MESSAGE.
	RET			;RETURN

;******************************************************
	.PAGE
	.SBTTL	"WRITE DISKETTE IDENTITY"
;******************************************************
; WRITE ID SECTOR				      *
;******************************************************

;*******( STORE ID IN SECTOR BUFFER )******************

WSD.ID: LXI	B,DD.BFL	;NMBR BYTE TO MOVE.
	LHLD	D.ADDR		;DD SYS ADDRESS.
	LXI	D,DD.BUF	;SECTOR BUFFER OFFSET
	DAD	D		;HL NOW PTS CMND BLK.
	XCHG
	LXI	H,IDS.SS	;SINGLE DENSITY ID
	CALL	BLK.MV		;PERFORM BLOCK MOVE.
	
;*******( SET TRACK AND SECTOR NUMBERS )***************

	MVI	A,TRK.0		;TRACK 0 SET.
	STA	DCM.TRK		;BIOS SET TRACK.
	MVI	A,ID.SEC	;ID SECTOR VALUE.
	STA	DCM.SEC		;BIOS SET SECTOR.
	
;*******( PERFORM WRITE SECTOR )***********************

	MVI	A,DCM.WRS	;DCM WRITE SECTOR
	CALL	DSK.EX		;BIOS WRITE SECTOR.
	RET			;ERROR RETURN.

;******************************************************
	.PAGE
	.SBTTL	"INJECTION MODULE - MACRO DEFINITIONS"
;******************************************************
; FORMAT - TITLE BLOCK AND PAGE ALIGNMENT	      *
;******************************************************

	.DEFINE	FORMAT [NAME] = [
NAME	==	(.!0FFH)+1	;SET NEXT PAGE BOUNDRY.
	.LOC	NAME		;SET LOC TO NEXT PAGE.
OFFSET	=	FMT.EA-NAME	;DETERMINE ADDR OFFSET.
	.Z80			;NOW USE Z8O  CODE.
	.ASCII	'FORMAT!']	;INCLUDE HEADER!

;******************************************************
; DENSITY - DECLARE TYPE			      *
;******************************************************

	.DEFINE	DENSITY	[TYPE] = [
	.IFIDN	[TYPE][SINGLE], [
	.ASCII	'S'
	.EXIT]
	.IFIDN	[TYPE][DOUBLE], [
	.ASCII	'D'
	.EXIT]
	.ERROR	'INVALID DENSITY']

;******************************************************
; SECTORS - SPECIFY SEQUENCE AND NUMBER OF SECTORS    *
;******************************************************

	.DEFINE	SECTORS	[LIST,NMBR] = [
	LXI	H,LIST+OFFSET	;SECTOR SEQUENCE ADDR.
	MVI	E,NMBR]		;NUMBER OF SECTORS.

;******************************************************
; BLOCK - GENERATE A BLOCK OF CONSTANTS		      *
;******************************************************

	.DEFINE	BLOCK [COUNT,BYTE,%REPT] = [
	NMBR = COUNT		;SET EQUAL FOR NOW.
	MVI	B,NMBR		;LOAD NMBR OF BYTES.
%REPT:	IN	XP.DSH		;WAIT FOR DATA REQ.
	MVI	A,BYTE		;LOAD BYTE VALUE.
	XRA	C		;INVERT (1791-01).
	OUT	WD.DTA		;WRITE DATA PORT.
	DJNZ	%REPT]		;REPEAT FOR COUNT.

;******************************************************
	.PAGE
;******************************************************
; REPEAT - REPEAT FORMAT SECTION FOR EACH SECTOR      *
;******************************************************

	.DEFINE	REPEAT [LOCATION] = [
	DCR	E		;DEC NMBR SECTORS LEFT.
	JNZ	LOCATION+OFFSET]

;******************************************************
; ENDING - RECORD NMBR OF TRAILING BYTES WRITTEN      *
;******************************************************

	.DEFINE	ENDING	[BYTE,%REPT] = [
	LXI	H,0		;COUNT OF ZERO.
%REPT:	IN	XP.DSH		;WAIT FOR REQ.
	MVI	A,BYTE		;LOAD CONSTANT.
	XRA	C		;INVERT (1791-01).
	OUT	WD.DTA		;WRITE TO PORT.
	INX	H		;INCREMENT COUNT.
	JMP	%REPT+OFFSET	;CONTINUE.]

;******************************************************
	.PAGE
;******************************************************
; WRITE - WRITE SPECIFIC FORMAT BYTES		      *
;******************************************************

	.DEFINE	WRITE [TYPE,VALU] = [

;*******( ID ADDRESS MARK )****************************

	.IFIDN	[TYPE][ID.MARK],   [
	 IN	XP.DSH		;WAIT FOR DATA REQ.
	 MVI	A,0FEH		;ID ADDR MARK.
	 XRA	C		;INVERT (1791-01).
	 OUT	WD.DTA		;WRITE DATA PORT.
	.EXIT]			;TERMINATE MACRO

;*******( INDEX MARK )*********************************

	.IFIDN	[TYPE][INDEX.MARK], [
	 IN	XP.DSH		;WAIT FOR DATA REQ.
	 MVI	A,0FCH		;INDEX MARK.
	 XRA	C		;INVERT (1791-01).
	 OUT	WD.DTA		;WRITE DATA PORT.
	.EXIT]			;TERMINATE MACRO

;*******( DATA ADDRESS MARK )**************************

	.IFIDN	[TYPE][DATA.MARK], [
	 IN	XP.DSH		;WAIT FOR DATA REQ.
	 MVI	A,0FBH		;DATA ADDR MARK.
	 XRA	C		;INVERT (1791-01).
	 OUT	WD.DTA		;WRITE DATA PORT.
	.EXIT]			;TERMINATE MACRO

;*******( CRC )****************************************

	.IFIDN	[TYPE][CRC], [
	 IN	XP.DSH		;WAIT FOR DATA REQ.
	 MVI	A,0F7H		;GENERATE CRC.
	 XRA	C		;INVERT (1791-01).
	 OUT	WD.DTA		;WRITE DATA PORT.
	.EXIT]			;TERMINATE MACRO

;*******( EXPLICIT BYTE VALUE )************************

	.IFIDN	[TYPE][BYTE], [
	 IN	XP.DSH		;WAIT FOR DATA REQ.
	 MVI	A,VALU 		;EXPLICIT VALUE.
	 XRA	C		;INVERT (1791-01).
	 OUT	WD.DTA		;WRITE DATA PORT.
	.EXIT]

;*******( TRACK NUMBER )*******************************

	.IFIDN	[TYPE][TRACK.NO], [
	 IN	XP.DSH		;WAIT FOR REQUEST.
	 IN	WD.TRK		;GET TRACK NMBR.
	 OUT	WD.DTA		;WRITE DATA PORT.
	.EXIT]

;*******( SECTOR NUMBER )******************************

	.IFIDN	[TYPE][SECTOR.NO], [
	IN	XP.DSH		;WAIT FOR REQUEST.
	MOV	A,M		;SET SECTOR NUMBR.
	XRA	C		;INVERT (1791-01).
	OUT	WD.DTA		;WRITE DATA PORT.
	INX	H		;INC SEC-NMBR PNTR.
	.EXIT			;TERMINATE MACRO]

;*******( SIDE NUMBER )********************************

	.IFIDN	[TYPE][SIDE.NO], [
	IN	XP.DSH		;WAIT FOR REQUEST.
	MVI	A,0		;SET SIDE NUMBER.
	XRA	C		;INVERT (1791-01).
	OUT	WD.DTA		;WRITE DATA PORT.
	.EXIT			;TERMINATE MACRO]

;*******( SECTOR SIZE CODE )***************************

	.IFIDN	[TYPE][SECTOR.SIZE], [
	SEC.CD = 0FFH		;DECLARE BLANK.
	.IFIDN [VALU][128],	[SEC.CD = 000H]
	.IFIDN [VALU][256],	[SEC.CD = 001H]
	.IFIDN [VALU][512],	[SEC.CD = 002H]
	.IFIDN [VALU][1024],	[SEC.CD = 003H]
	.IFE	(SEC.CD-0FFH),	[
	.ERROR	'INVALID SECTOR SIZE']
	 IN	XP.DSH		;WAIT FOR DATA REQ.
	 MVI	A,SEC.CD	;LOAD SIZE CODE.
	 XRA	C		;INVERT (1791-01).
	 OUT	WD.DTA		;WRITE DATA PORT.
	.EXIT			;TERMINATE MACRO]

;*******( ILLEGAL EXPANSION )**************************

	.ERROR	'ILLEGAL EXPANSION']

;******************************************************
	.PAGE
	.SBTTL	'INJECTION MODULE FT3740'
	FORMAT	FT3740
	DENSITY	SINGLE
	SECTORS	SS3740,26

BG3740:	BLOCK	40,ONES
	BLOCK	6,ZEROS
	WRITE	INDEX.MARK
	BLOCK	26,ONES
RP3740:	BLOCK	6,ZEROS
	WRITE	ID.MARK
	WRITE	TRACK.NO
	WRITE	SIDE.NO
	WRITE	SECTOR.NO
	WRITE	SECTOR.SIZE,128
	WRITE	CRC
	BLOCK	11,ONES
	BLOCK	6,ZEROS
	WRITE	DATA.MARK
	BLOCK	128,0E5H
	WRITE	CRC
	BLOCK	27,ONES
	REPEAT	RP3740

	ENDING	ONES

SS3740:	.BYTE	 1, 2, 3, 4, 5, 6, 7, 8, 9,10
	.BYTE	11,12,13,14,15,16,17,18,19,20
	.BYTE	21,22,23,24,25,26

	.PAGE
	.SBTTL	'INJECTION MODULE FTJ50D'
	FORMAT	FTJ50D
	DENSITY	DOUBLE
	SECTORS	SSJ50D,50

BGJ50D:	BLOCK	80,04EH
RPJ50D:	BLOCK	8,ZEROS
	BLOCK	3,0F5H
	WRITE	ID.MARK
	WRITE	TRACK.NO
	WRITE	SIDE.NO
	WRITE	SECTOR.NO
	WRITE	SECTOR.SIZE,128
	WRITE	CRC
	BLOCK	22,04EH
	BLOCK	12,ZEROS
	BLOCK	3,0F5H
	WRITE	DATA.MARK
	BLOCK	128,0E5H
	WRITE	CRC
	BLOCK	17,04EH
	REPEAT	RPJ50D

	ENDING	ONES

SSJ50D:	.BYTE	 1,11,21,31,41
	.BYTE	 2,12,22,32,42
	.BYTE	 3,13,23,33,43
	.BYTE	 4,14,24,34,44
	.BYTE	 5,15,25,35,45
	.BYTE	 6,16,26,36,46
	.BYTE	 7,17,27,37,47
	.BYTE	 8,18,28,38,48
	.BYTE	 9,19,29,39,49
	.BYTE	10,20,30,40,50


	.END
