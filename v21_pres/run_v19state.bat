@echo off
chcp 65001 > nul
title v19_state - v19 + State-Space Planes
cd /d "%~dp0"
echo ========================================================
echo   v19_state  :  v19 Simulator + State-Space Planes
echo   (beta-phi plane  /  prediction-point plane)
echo ========================================================
echo.

where python > nul 2> nul
if errorlevel 1 (
  echo [ERROR] python not found in PATH.
  echo         Install Python 3 and re-run.
  pause
  exit /b 1
)

echo [1/4] Checking packages ...
python -c "import mujoco, aiohttp, numpy, scipy" 2> nul
if errorlevel 1 (
  echo       missing packages - installing ^(one time^) ...
  python -m pip install mujoco aiohttp numpy scipy
  if errorlevel 1 (
    echo [ERROR] pip install failed.
    pause
    exit /b 1
  )
)
echo       OK

echo [2/4] Starting server on port 8200 ...
start "v19_state server" python server.py

echo [3/4] Waiting for MuJoCo build ...
timeout /t 6 /nobreak > nul

echo [4/4] Opening browser ...
start http://localhost:8200

echo.
echo ========================================================
echo   Running at  http://localhost:8200
echo   Close the "v19_state server" window to stop.
echo   (v19_sim uses port 8199 - both can run together)
echo ========================================================
echo.
pause
