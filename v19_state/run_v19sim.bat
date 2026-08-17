@echo off
chcp 65001 > nul
title v19 MuJoCo Simulator Runner
echo ========================================================
echo   v19 MuJoCo Slackline Balance Simulator
echo ========================================================
echo.

cd /d "%~dp0"

echo [1/3] Starting Python Simulation Server...
start "v19_sim_server" python server.py

echo [2/3] Waiting for server to initialize...
timeout /t 2 /nobreak > nul

echo [3/3] Launching Web Browser at http://localhost:8200 ...
start http://localhost:8200

echo.
echo ========================================================
echo   Simulation is now running in your browser!
echo ========================================================
