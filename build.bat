@echo off
rem FaceFall build script (Windows, uses the bundled w64devkit GCC).
rem If you have your own GCC on PATH, just replace the compiler path below.
setlocal
set GCC=%~dp0tools\w64devkit\bin\gcc.exe
set RAYLIB=%~dp0tools\raylib-6.0_win64_mingw-w64
rem gcc needs its own bin dir on PATH to find the assembler/linker (as, ld)
if exist "%GCC%" (set "PATH=%~dp0tools\w64devkit\bin;%PATH%") else (set GCC=gcc)

"%GCC%" "%~dp0facefall.c" -o "%~dp0facefall.exe" -O2 -Wall -Wextra ^
    -I "%RAYLIB%\include" -L "%RAYLIB%\lib" ^
    -lraylib -lopengl32 -lgdi32 -lwinmm

if %errorlevel%==0 (echo Build OK: facefall.exe) else (echo Build FAILED)
endlocal
