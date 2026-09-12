@echo off
setlocal
set "MANAGER=%~dp0build\windows_jaka-manager-v4\Release\windows_robot_manager.exe"
if not exist "%MANAGER%" set "MANAGER=%~dp0build\windows_jaka-manager-v3\Release\windows_robot_manager.exe"
if not exist "%MANAGER%" set "MANAGER=%~dp0build\windows_jaka-manager-v2\Release\windows_robot_manager.exe"
if not exist "%MANAGER%" set "MANAGER=%~dp0build\windows_jaka-manager\Release\windows_robot_manager.exe"
if not exist "%MANAGER%" set "MANAGER=%~dp0build\windows_jaka-vs\Release\windows_robot_manager.exe"
if not exist "%MANAGER%" (
    echo Robot manager executable not found:
    echo %MANAGER%
    pause
    exit /b 1
)
start "" "%MANAGER%"
