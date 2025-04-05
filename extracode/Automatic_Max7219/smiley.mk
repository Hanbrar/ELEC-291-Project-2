SHELL = cmd
CC = xc32-gcc
OBJCPY = xc32-bin2hex
ARCH = -mprocessor=32MX130F064B
OBJ = smiley.o
PORTN = $(shell type COMPORT.inc)

smiley.elf: $(OBJ)
	$(CC) $(ARCH) -o smiley.elf $(OBJ) -mips16 -DXPRJ_default=default -legacy-libc -Wl,-Map=smiley.map
	$(OBJCPY) smiley.elf
	@echo Success!

smiley.o: smiley.c
	$(CC) -mips16 -g -x c -c $(ARCH) -MMD -o smiley.o smiley.c -DXPRJ_default=default -legacy-libc

clean:
	@del *.o *.elf *.hex *.map *.d 2>NUL

LoadFlash:
	@taskkill /f /im putty.exe /t /fi "status eq running" > NUL
	pro32 -p smiley.hex
	cmd /c start putty.exe -serial $(PORTN) -sercfg 115200,8,n,1,N

putty:
	@taskkill /f /im putty.exe /t /fi "status eq running" > NUL
	cmd /c start putty.exe -serial $(PORTN) -sercfg 115200,8,n,1,N

dummy: smiley.hex smiley.map
	$(CC) --version
