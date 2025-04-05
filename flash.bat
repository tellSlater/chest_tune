@echo off
REM ============================================================================
REM Batch script to repeatedly program ATtiny13A using avrdude and USBasp
REM ----------------------------------------------------------------------------
REM - Sets low fuse to 0x7A and high fuse to 0xFF
REM - Writes your_firmware.hex to flash
REM - Repeats the process each time you press Enter
REM - Close the window or press Ctrl+C to exit
REM ============================================================================

:loop
cls
echo [INFO] Setting fuses...
avrdude -c usbasp -p t13 -B 8 -U lfuse:w:0x7A:m -U hfuse:w:0xFF:m

echo [INFO] Flashing firmware...
avrdude -c usbasp -p t13 -U flash:w:Release/chest_tune.hex:i

echo.
echo Press Enter to flash another device, or close this window to quit.
pause >nul
goto loop
