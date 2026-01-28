@echo off
setlocal EnableExtensions EnableDelayedExpansion

set "ROOT=%~dp0"
pushd "%ROOT%"

set "MODE=gui"
set "CFG=Release"
set "CLEAN=0"

:parse
if "%~1"=="" goto parsedone
if /I "%~1"=="gui" set "MODE=gui"
if /I "%~1"=="cli" set "MODE=cli"
if /I "%~1"=="all" set "MODE=all"
if /I "%~1"=="Release" set "CFG=Release"
if /I "%~1"=="Debug" set "CFG=Debug"
if /I "%~1"=="clean" set "CLEAN=1"
shift
goto parse

:parsedone
if "%QT_KIT%"=="" set "QT_KIT=C:\Qt\6.10.1\msvc2022_64"
set "Qt6_DIR=%QT_KIT%\lib\cmake\Qt6"

for /f %%i in ('powershell -NoProfile -Command "Get-Date -Format yyyyMMdd_HHmmss"') do set "TS=%%i"

set "LOGDIR=%ROOT%logs"
if not exist "%LOGDIR%" mkdir "%LOGDIR%"

set "BUILD_DIR=%ROOT%build_%MODE%"
set "LOG=%LOGDIR%\cmake_%MODE%_%CFG%_%TS%.log"

echo MODE=%MODE%
echo CFG=%CFG%
echo QT_KIT=%QT_KIT%
echo LOG=%LOG%

> "%LOG%" echo MODE=%MODE%
>> "%LOG%" echo CFG=%CFG%
>> "%LOG%" echo QT_KIT=%QT_KIT%
>> "%LOG%" echo Qt6_DIR=%Qt6_DIR%
>> "%LOG%" echo BUILD_DIR=%BUILD_DIR%
>> "%LOG%" echo.

if "%CLEAN%"=="1" (
  if exist "%BUILD_DIR%" (
    echo Cleaning "%BUILD_DIR%"
    >> "%LOG%" echo Cleaning "%BUILD_DIR%"
    rmdir /s /q "%BUILD_DIR%" >> "%LOG%" 2>&1
  )
)

call :cmd git submodule update --init --recursive
call :cmd cmake -S . -B "%BUILD_DIR%" -G "Visual Studio 17 2022" -A x64 -DCMAKE_PREFIX_PATH="%QT_KIT%" -DQt6_DIR="%Qt6_DIR%"

if /I "%MODE%"=="gui" (
  call :cmd cmake --build "%BUILD_DIR%" --config %CFG% --target softionics_hub_gui
  set "EXE=%BUILD_DIR%\%CFG%\softionics_hub_gui.exe"
) else if /I "%MODE%"=="cli" (
  call :cmd cmake --build "%BUILD_DIR%" --config %CFG% --target softionics_hub_cli
  set "EXE=%BUILD_DIR%\%CFG%\softionics_hub_cli.exe"
) else (
  call :cmd cmake --build "%BUILD_DIR%" --config %CFG%
  set "EXE="
)

echo.
echo Done
echo Log: "%LOG%"
if not "%EXE%"=="" (
  if exist "%EXE%" echo Exe: "%EXE%"
)

popd
exit /b 0

:cmd
echo --------------------------------------------------
echo Running: %*
>> "%LOG%" echo --------------------------------------------------
>> "%LOG%" echo Running: %*
cmd /c %* >> "%LOG%" 2>&1
if errorlevel 1 (
  echo FAILED: %*
  >> "%LOG%" echo FAILED: %*
  popd
  exit /b 1
)
exit /b 0
