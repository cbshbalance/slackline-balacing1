@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo 8210 포트에 남아 있는 이전 서버를 정리합니다...
for /f "tokens=5" %%a in (netstat -ano | findstr :8210 | findstr LISTENING) do taskkill /F /PID %%a >nul 2>&1
echo 새 서버를 시작합니다. 이 창을 닫으면 서버도 꺼집니다.
echo 브라우저에서 http://localhost:8210 를 여세요
echo.
python server.py
pause
