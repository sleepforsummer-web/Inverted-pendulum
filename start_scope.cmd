@echo off
setlocal
cd /d "%~dp0"
title Double Loop PID Scope

if not exist "%~dp0db_pid_scope.py" goto missing_script
set "SCOPE_PYTHON=%LOCALAPPDATA%\Programs\Python\Python313\python.exe"
if exist "%SCOPE_PYTHON%" goto run

where py >nul 2>nul
if not errorlevel 1 set "SCOPE_PYTHON=py"
if not errorlevel 1 goto run

where python >nul 2>nul
if not errorlevel 1 set "SCOPE_PYTHON=python"
if not errorlevel 1 goto run

echo Python was not found. Please install Python first.
pause
exit /b 1

:run
if /i "%~1"=="--check" goto check
"%SCOPE_PYTHON%" "%~dp0db_pid_scope.py"
set "SCOPE_EXIT=%ERRORLEVEL%"
if not "%SCOPE_EXIT%"=="0" pause
exit /b %SCOPE_EXIT%

:check
"%SCOPE_PYTHON%" -c "import sys, tkinter, serial, numpy, matplotlib, pendulum_view, scope_theme, serial_commands; from pathlib import Path; p=Path(sys.argv[1]); compile(p.read_text(encoding='utf-8'), str(p), 'exec'); print('Scope launcher and dependencies OK'); print(sys.executable)" "%~dp0db_pid_scope.py"
exit /b %ERRORLEVEL%

:missing_script
echo db_pid_scope.py was not found next to this launcher.
pause
exit /b 1
