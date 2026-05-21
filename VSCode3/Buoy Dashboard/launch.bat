@echo off
chcp 65001 >nul
cd /d "%~dp0"
title Buoy Dashboard - Server

echo ============================================
echo   Antarctic Buoy Array - Mission Dashboard
echo ============================================
echo.
echo Starting Flask + Socket.IO bridge server...
echo Keep this window open. Close it to shut down.
echo.

REM Check Python is available
py --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Python not found in PATH.
    echo Install Python from https://python.org and re-run.
    pause
    exit /b 1
)

REM Install / upgrade dependencies silently
echo Checking dependencies...
py -m pip install flask flask-socketio flask-sqlalchemy flask-cors pyserial >nul 2>&1
echo Dependencies OK.
echo.

echo Server starting on http://127.0.0.1:5000
echo Browser will open automatically in 5 seconds.
echo.

REM Open the browser after a 5-second delay in a separate process.
REM The delay gives Flask time to fully bind the port before Chrome loads the page.
start "" /b cmd /c "timeout /t 5 >nul && start http://127.0.0.1:5000"

REM Run the server in this window so all output (serial, errors) stays visible.
py app.py

echo.
echo Server stopped. Press any key to close.
pause >nul