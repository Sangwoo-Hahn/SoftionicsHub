@echo off
setlocal EnableExtensions EnableDelayedExpansion
chcp 65001 >nul

set CFG=Release
set BUILD_DIR=%~dp0build_gui
set QT_KIT=C:\Qt\6.10.1\msvc2022_64

if not "%1"=="" set CFG=%1
if not "%2"=="" set BUILD_DIR=%2
if not "%3"=="" set QT_KIT=%3

for %%I in ("%~dp0.") do set ROOT=%%~fI
set DIST_ROOT=%ROOT%\dist
set DIST=%DIST_ROOT%\SoftionicsHub_Windows_x64_%CFG%
set EXE=%BUILD_DIR%\%CFG%\softionics_hub_gui.exe
set LOG=%DIST_ROOT%\package_windows_%CFG%.log

mkdir "%DIST_ROOT%" 2>nul
echo MODE=windows CFG=%CFG% > "%LOG%"

call :run cmake --build "%BUILD_DIR%" --config %CFG% --target softionics_hub_gui
if errorlevel 1 goto :fail

if not exist "%EXE%" (
  echo EXE not found: "%EXE%" >> "%LOG%"
  goto :fail
)

rmdir /s /q "%DIST%" 2>nul
mkdir "%DIST%" 2>nul

copy /y "%EXE%" "%DIST%\" >> "%LOG%" 2>&1

set WINDEPLOY=%QT_KIT%\bin\windeployqt.exe
if not exist "%WINDEPLOY%" (
  echo windeployqt not found: "%WINDEPLOY%" >> "%LOG%"
  goto :fail
)

call :run "%WINDEPLOY%" --release --compiler-runtime --no-translations --verbose 1 "%DIST%\softionics_hub_gui.exe"
if errorlevel 1 goto :fail

for %%F in ("%BUILD_DIR%\%CFG%\*.dll") do copy /y "%%F" "%DIST%\" >> "%LOG%" 2>&1

set ZIP=%DIST_ROOT%\SoftionicsHub_Windows_x64_%CFG%.zip
if exist "%ZIP%" del /f /q "%ZIP%" >nul 2>nul

powershell -NoProfile -Command "Compress-Archive -Path '%DIST%\*' -DestinationPath '%ZIP%' -Force" >> "%LOG%" 2>&1

echo DONE: %ZIP%
exit /b 0

:run
echo -------------------------------------------------- >> "%LOG%"
echo RUN: %* >> "%LOG%"
echo -------------------------------------------------- >> "%LOG%"
%* >> "%LOG%" 2>&1
exit /b %errorlevel%

:fail
echo FAILED. LOG: %LOG%
exit /b 1
