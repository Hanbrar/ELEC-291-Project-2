SHELL=cmd
CC = xc32-gcc
OBJCPY = xc32-bin2hex
ARCH = -mprocessor=32MX130F064B
OBJ = JDY40_test.o
PORTN=$(shell type COMPORT.inc)

JDY40_test.elf: $(OBJ)
	$(CC) $(ARCH) -o JDY40_test.elf JDY40_test.o -mips16 -DXPRJ_default=default -legacy-libc -Wl,-Map=JDY40_test.map
	$(OBJCPY) JDY40_test.elf
	@echo Success!
   
JDY40_test.o: JDY40_test.c
	$(CC) -mips16 -g -x c -c $(ARCH) -MMD -o JDY40_test.o JDY40_test.c -DXPRJ_default=default -legacy-libc

clean:
	@del *.o *.elf *.hex *.map *.d 2>NUL
	
LoadFlash:
	@taskkill /f /im putty.exe /t /fi "status eq running" > NUL
	pro32 -p JDY40_test.hex
	cmd /c start putty.exe -serial $(PORTN) -sercfg 115200,8,n,1,N

putty:
	@taskkill /f /im putty.exe /t /fi "status eq running" > NUL
	cmd /c start putty.exe -serial $(PORTN) -sercfg 115200,8,n,1,N

dummy: JDY40_test.hex JDY40_test.map
	$(CC) --version
