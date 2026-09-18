@echo off
rem UAV Pathfinder -- cift tiklanabilir baslatici.
rem Menuyu scripts\launcher\uav_launcher.ps1 cizer; bu dosya yalnizca onu
rem dogru klasorde ve dogru ExecutionPolicy ile acar.
rem Parametre gecirmek icin komut satirindan da cagrilabilir, ornegin:
rem     UAV_Pathfinder.bat -Region mugla -Port 8800

setlocal
cd /d "%~dp0"

set "LAUNCHER=%~dp0scripts\launcher\uav_launcher.ps1"
if not exist "%LAUNCHER%" (
  echo.
  echo HATA: scripts\launcher\uav_launcher.ps1 bulunamadi.
  echo Bu dosya proje kokunde durmalidir.
  echo.
  pause
  exit /b 1
)

powershell -NoProfile -ExecutionPolicy Bypass -File "%LAUNCHER%" %*
set "RC=%ERRORLEVEL%"

if not "%RC%"=="0" (
  echo.
  echo Baslatici %RC% kodu ile kapandi.
  pause
)

endlocal & exit /b %RC%
