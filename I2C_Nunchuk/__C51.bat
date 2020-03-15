@echo off
::This file was created automatically by CrossIDE to compile with C51.
C:
cd "\Users\Nick T\Downloads\STM32_Update_March_09\I2C_Nunchuk\"
"C:\CrossIDE\Call51\Bin\c51.exe" --use-stdout  "C:\Users\Nick T\Downloads\STM32_Update_March_09\I2C_Nunchuk\main.c"
if not exist hex2mif.exe goto done
if exist main.ihx hex2mif main.ihx
if exist main.hex hex2mif main.hex
:done
echo done
echo Crosside_Action Set_Hex_File C:\Users\Nick T\Downloads\STM32_Update_March_09\I2C_Nunchuk\main.hex
