SHELL=cmd
CC = xc32-gcc
OBJCPY = xc32-bin2hex
ARCH = -mprocessor=32MX130F064B
PORTN=$(shell type COMPORT.inc)

OBJ = Servo.o

Servo.elf: $(OBJ)
	$(CC) $(ARCH) -o Servo.elf Servo.o -mips16 -DXPRJ_default=default -legacy-libc -Wl,-Map=Servo.map
	$(OBJCPY) Servo.elf
	@echo Success!
   
Servo.o: Servo.c
	$(CC) -g -x c -mips16 -Os -c $(ARCH) -MMD -o Servo.o Servo.c -DXPRJ_default=default -legacy-libc

clean:
	@del *.o *.d *.map *.elf *.hex 2>NUL

FlashLoad:
	@taskkill /f /im putty.exe /t /fi "status eq running" > NUL
	pro32 -p Servo.hex
	@cmd /c start putty -serial $(PORTN) -sercfg 115200,8,n,1,N

putty:
	@taskkill /f /im putty.exe /t /fi "status eq running" > NUL
	@cmd /c start putty -serial $(PORTN) -sercfg 115200,8,n,1,N

dummy: Servo.hex Servo.map
	@echo ;-)
	
explorer:
	explorer .
