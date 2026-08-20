@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo [1/3] 8210 포트의 이전 서버를 정리합니다...
for /f "tokens=5" %%a in ('netstat -ano ^| findstr ":8210" ^| findstr "LISTENING"') do taskkill /F /PID %%a >nul 2>&1
timeout /t 1 /nobreak >nul
for %%f in (sim_engine.py) do echo     물리엔진 파일 시각: %%~tf   ^(업데이트 직후라면 방금 시각이어야 정상^)
echo [2/3] 5초 뒤 브라우저가 자동으로 열립니다: http://localhost:8210
start "" /min cmd /c "timeout /t 5 /nobreak >nul && start http://localhost:8210"
echo [3/3] 서버 시작 — 이 창을 닫으면 서버도 꺼집니다.
echo.
python server.py
echo.
echo ^(서버가 바로 꺼졌다면 위 오류 메시지를 캡처해서 클로드에게 보여주세요^)
pause
