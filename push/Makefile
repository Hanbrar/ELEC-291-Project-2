SHELL=cmd
CC=arm-none-eabi-gcc
AS=arm-none-eabi-as
LD=arm-none-eabi-ld
CCFLAGS=-mcpu=cortex-m0 -mthumb -g 

# Search for the path of the right libraries.  Works only on Windows.
GCCPATH=$(subst \bin\arm-none-eabi-gcc.exe,\,$(shell where $(CC)))
LIBPATH1=$(subst \libgcc.a,,$(shell dir /s /b "$(GCCPATH)*libgcc.a" | find "v6-m"))
LIBPATH2=$(subst \libc_nano.a,,$(shell dir /s /b "$(GCCPATH)*libc_nano.a" | find "v6-m"))
LIBSPEC=-L"$(LIBPATH1)" -L"$(LIBPATH2)"

OBJS=pushbutton.o serial.o UART2.o startup.o newlib_stubs.o

PORTN=$(shell type COMPORT.inc)

# For smaller hex file remove '-u _printf_float' below
pushbutton.elf : $(OBJS)
	$(LD) $(OBJS) $(LIBSPEC) -Os -nostdlib -lnosys -lgcc -T ../Common/LDscripts/stm32l051xx.ld --cref -Map pushbutton.map -o pushbutton.elf
	arm-none-eabi-objcopy -O ihex pushbutton.elf pushbutton.hex
	@echo Success!

pushbutton.o: pushbutton.c
	$(CC) -c $(CCFLAGS) pushbutton.c -o pushbutton.o

startup.o: ../Common/Source/startup.c
	$(CC) -c $(CCFLAGS) -DUSE_USART1 ../Common/Source/startup.c -o startup.o

serial.o: ../Common/Source/serial.c
	$(CC) -c $(CCFLAGS) ../Common/Source/serial.c -o serial.o

UART2.o: UART2.c
	$(CC) -c $(CCFLAGS) UART2.c -o UART2.o

newlib_stubs.o: ../Common/Source/newlib_stubs.c
	$(CC) -c $(CCFLAGS) ../Common/Source/newlib_stubs.c -o newlib_stubs.o

clean: 
	@del $(OBJS) 2>NUL
	@del pushbutton.elf pushbutton.hex pushbutton.map 2>NUL
	@del *.lst 2>NUL
	
Flash_Load:
	@taskkill /f /im putty.exe /t /fi "status eq running" > NUL
	@echo ..\stm32flash\stm32flash -w pushbutton.hex -v -g 0x0 ^^>sflash.bat
	@..\stm32flash\BO230\BO230 -b >>sflash.bat
	@sflash
	@echo cmd /c start putty.exe -sercfg 115200,8,n,1,N -serial ^^>sputty.bat
	@..\stm32flash\BO230\BO230 -r >>sputty.bat
	@sputty

putty:
	@echo cmd /c start putty.exe -sercfg 115200,8,n,1,N -serial ^^>sputty.bat
	@..\stm32flash\BO230\BO230 -r >>sputty.bat
	@sputty
	
explorer:
	@explorer .

# End of snippet from push/Makefile
