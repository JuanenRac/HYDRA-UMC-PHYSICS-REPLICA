@echo off
REM =============================================================================
REM HYDRA-UMC-PHYSICS-REPLICA - run.bat
REM Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
REM GPL-3.0 - see LICENSE
REM =============================================================================
REM Forwards all arguments (e.g. "run.bat fk --urdf arm.urdf --joints j1=0.5").
setlocal
cd /d "%~dp0"

set BIN=target\release\hydra-umc-physics-replica.exe

if not exist "%BIN%" (
    echo No compiled binary found - run build.bat first. 1>&2
    exit /b 1
)

"%BIN%" %*
set EXIT_CODE=%errorlevel%
pause
exit /b %EXIT_CODE%
