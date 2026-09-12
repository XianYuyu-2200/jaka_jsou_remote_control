@echo off
set "JAKA_ENABLE_MOTION=1"
if not exist "%~dp0build\windows_jaka-vs\Release\windows_teleop_gui.exe" (
    echo GUI executable not found:
    echo %~dp0build\windows_jaka-vs\Release\windows_teleop_gui.exe
    pause
    exit /b 1
)
start "" "%~dp0build\windows_jaka-vs\Release\windows_teleop_gui.exe"
