@echo off
chcp 65001 > nul
title v19_state Phase Plane Simulator Runner
echo ========================================================
echo   v19_state (Phase Plane & State Space Visualizer)
echo ========================================================
echo.

cd /d "%~dp0v19_state"

echo [1/3] Starting Python Simulation Server (Port 8200)...
start "v19_state_server" python server.py

echo [2/3] Waiting for server to initialize...
timeout /t 2 /nobreak > nul

echo [3/3] Launching Web Browser at http://localhost:8200 ...
start http://localhost:8200

echo.
echo ========================================================
echo   v19_state Simulator is running in your browser!
echo   (Server is running at http://localhost:8200)
echo ========================================================
