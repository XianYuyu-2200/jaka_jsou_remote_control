@echo off
setlocal
set "JAKA_ENABLE_MOTION=1"
set "GUI=%~dp0build\windows_jaka-vs\Release\windows_teleop_gui.exe"
if not exist "%GUI%" (
    echo GUI executable not found:
    echo %GUI%
    pause
    exit /b 1
)
start "" "%GUI%"
