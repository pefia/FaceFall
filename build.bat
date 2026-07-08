@echo off
setlocal
set GCC=%~dp0tools\w64devkit\bin\gcc.exe
set RAYLIB=%~dp0tools\raylib-6.0_win64_mingw-w64
if exist "%GCC%" (set "PATH=%~dp0tools\w64devkit\bin;%PATH%") else (set GCC=gcc)

"%GCC%" "%~dp0src\physics.c" "%~dp0src\collision.c" "%~dp0src\render.c" ^
    "%~dp0src\export.c" "%~dp0src\ui.c" "%~dp0src\main.c" ^
    -o "%~dp0facefall.exe" -O2 -Wall -Wextra ^
    -I "%~dp0src" -I "%RAYLIB%\include" -L "%RAYLIB%\lib" ^
    -lraylib -lopengl32 -lgdi32 -lwinmm

if %errorlevel%==0 (echo Build OK: facefall.exe) else (echo Build FAILED)
endlocal
