@echo off
setlocal EnableExtensions EnableDelayedExpansion
chcp 65001 >nul

set "ROOT=%~dp0"
pushd "%ROOT%"

set "CFG=Release"
set "CLEAN=0"

:parse
if "%~1"=="" goto parsedone
if /I "%~1"=="Release" set "CFG=Release"
if /I "%~1"=="Debug" set "CFG=Debug"
if /I "%~1"=="clean" set "CLEAN=1"
shift
goto parse

:parsedone

for /f %%i in ('powershell -NoProfile -Command "Get-Date -Format yyyyMMdd_HHmmss"') do set "TS=%%i"
set "LOGDIR=%ROOT%logs"
if not exist "%LOGDIR%" mkdir "%LOGDIR%"

set "LOG=%LOGDIR%\preset_%CFG%_%TS%.log"

rem --- preset names (CMakeUserPresets.json에 존재해야 함) ---
if /I "%CFG%"=="Debug" (
  set "CFG_PRESET=win-local-qt-debug"
  set "EXE=%ROOT%build_gui\Debug\softionics_hub_gui.exe"
) else (
  set "CFG_PRESET=win-local-qt"
  set "EXE=%ROOT%build_gui\Release\softionics_hub_gui.exe"
)

> "%LOG%" echo CFG=%CFG%
>> "%LOG%" echo PRESET=%CFG_PRESET%
>> "%LOG%" echo.

call :cmd git submodule update --init --recursive || goto :fail

if "%CLEAN%"=="1" (
  call :cmd cmake --preset "%CFG_PRESET%" --fresh || goto :fail
) else (
  call :cmd cmake --preset "%CFG_PRESET%" || goto :fail
)

call :cmd cmake --build --preset "%CFG_PRESET%" || goto :fail

if not exist "%EXE%" (
  >> "%LOG%" echo EXE not found: "%EXE%"
  goto :fail
)

call :ok "%EXE%" "%LOG%"
popd
exit /b 0

:fail
call :bad "%LOG%"
popd
exit /b 1

:cmd
echo --------------------------------------------------
echo Running: %*
>> "%LOG%" echo --------------------------------------------------
>> "%LOG%" echo Running: %*
cmd /c %* >> "%LOG%" 2>&1
exit /b %errorlevel%

:ok
set "OK_EXE=%~1"
set "OK_LOG=%~2"
powershell -NoProfile -Command ^
  "$Host.UI.RawUI.ForegroundColor='Green';" ^
  "Write-Host 'SUCCESS ✅ Build completed.';" ^
  "Write-Host ('Exe:  ' + '%OK_EXE%');" ^
  "Write-Host ('Log:  ' + '%OK_LOG%');" ^
  "[console]::beep(880,120); [console]::beep(1320,120);" ^
  "$Host.UI.RawUI.ForegroundColor='Gray';" >nul 2>&1
echo SUCCESS ✅ Build completed.
echo Exe: "%OK_EXE%"
echo Log: "%OK_LOG%"
exit /b 0

:bad
set "BAD_LOG=%~1"
powershell -NoProfile -Command ^
  "$Host.UI.RawUI.ForegroundColor='Red';" ^
  "Write-Host 'FAILED ❌ Build failed.';" ^
  "Write-Host ('Log:  ' + '%BAD_LOG%');" ^
  "[console]::beep(220,250);" ^
  "$Host.UI.RawUI.ForegroundColor='Gray';" >nul 2>&1
echo FAILED ❌ Build failed.
echo Log: "%BAD_LOG%"
exit /b 1
