@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo GitHub 에서 최신 변경을 받아옵니다...
echo.
git pull origin main
echo.
echo ============================================
echo  위에 "Already up to date" 또는 파일 목록이
echo  보이면 성공입니다. 오류가 보이면 이 창을
echo  캡처해서 클로드에게 보여주세요.
echo ============================================
pause
