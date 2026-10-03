@echo off
rem ============================================================================
rem  build.bat - builds every program in apps\ into bin\<name>.exe
rem
rem    build.bat                  build everything
rem    build.bat position_tracker build just one
rem    build.bat sim [name]       simulator copies (no device; mouse = hand) -> bin\sim\r
rem
rem  Needs: Visual Studio 2022 (any edition) with "Desktop development with C++",
rem         and OpenHaptics Developer Edition. No .sln / .vcxproj files involved.
rem ============================================================================
setlocal EnableExtensions
cd /d "%~dp0"

set "SIM="
if /i "%~1"=="sim" set "SIM=1"
if defined SIM shift
if defined SIM goto :find_cl

if not defined OH_SDK_BASE set "OH_SDK_BASE=C:\OpenHaptics\Developer\3.5.0"
rem OH_SDK_BASE can be set but stale (lab PC: the SDK lives in "C:\OpenHaptics - Ben").
if not exist "%OH_SDK_BASE%\include\HD\hd.h" if exist "C:\OpenHaptics\Developer\3.5.0\include\HD\hd.h" set "OH_SDK_BASE=C:\OpenHaptics\Developer\3.5.0"
if not exist "%OH_SDK_BASE%\include\HD\hd.h" if exist "C:\OpenHaptics - Ben\Developer\3.5.0\include\HD\hd.h" set "OH_SDK_BASE=C:\OpenHaptics - Ben\Developer\3.5.0"
if not exist "%OH_SDK_BASE%\include\HD\hd.h" goto :no_sdk

set "HDLIB="
if exist "%OH_SDK_BASE%\lib\x64\Release\hd.lib" set "HDLIB=%OH_SDK_BASE%\lib\x64\Release"
if not defined HDLIB if exist "%OH_SDK_BASE%\lib\x64\hd.lib" set "HDLIB=%OH_SDK_BASE%\lib\x64"
if not defined HDLIB if exist "%OH_SDK_BASE%\lib\x64\Debug\hd.lib" set "HDLIB=%OH_SDK_BASE%\lib\x64\Debug"
if not defined HDLIB goto :no_lib

:find_cl
rem --- find the compiler (skip if already in a "Developer Command Prompt") ---
where cl >nul 2>nul
if not errorlevel 1 goto :have_cl
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" goto :no_vs
set "VSPATH="
for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSPATH=%%i"
if not defined VSPATH goto :no_vs
rem Skip VS's telemetry step: it launches powershell.exe, which is blocked on the lab PC (error popup).
set "VSCMD_SKIP_SENDTELEMETRY=1"
call "%VSPATH%\VC\Auxiliary\Build\vcvars64.bat" >nul
where cl >nul 2>nul
if errorlevel 1 goto :no_vs
:have_cl

if not exist bin mkdir bin
if not exist build\obj mkdir build\obj
if defined SIM if not exist bin\sim mkdir bin\sim

set "FAILED="
set "ONLY=%~1"
for /d %%a in (apps\*) do (
  if exist "%%a\main.cpp" (
    if "%ONLY%"=="" (call :build_one "%%~na") else if /i "%ONLY%"=="%%~na" (call :build_one "%%~na")
  )
)
echo.
if defined FAILED (
  echo BUILD FAILED for:%FAILED%
  exit /b 1
)
if defined SIM (echo Done. Simulator programs are in bin\sim\  e.g.  bin\sim\haptic_playground.exe) else (echo Done. Programs are in bin\  e.g.  bin\device_check.exe)
exit /b 0

:build_one
if defined SIM goto :build_sim
echo === %~1
cl /nologo /std:c++17 /EHsc /O2 /W3 /MD /DWIN32 /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS ^
   /I common /I "%OH_SDK_BASE%\include" ^
   "apps\%~1\main.cpp" /Fo"build\obj\%~1.obj" /Fe"bin\%~1.exe" ^
   /link /LIBPATH:"%HDLIB%" hd.lib >"build\%~1.log" 2>&1
if errorlevel 1 (
  type "build\%~1.log"
  set "FAILED=%FAILED% %~1"
) else (
  echo     -^> bin\%~1.exe
)
exit /b 0

:build_sim
echo === %~1 (simulator)
if not exist "build\obj\sim\%~1" mkdir "build\obj\sim\%~1"
cl /nologo /std:c++17 /EHsc /O2 /W3 /MD /DWIN32 /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS /DPHANTOM_MOCK ^
   /I common /I tools\hd_mock\include ^
   "apps\%~1\main.cpp" tools\hd_mock\hd_mock.cpp /Fo"build\obj\sim\%~1\\" /Fe"bin\sim\%~1.exe" >"build\sim_%~1.log" 2>&1
if errorlevel 1 (
  type "build\sim_%~1.log"
  set "FAILED=%FAILED% %~1"
) else (
  echo     -^> bin\sim\%~1.exe
)
exit /b 0

:no_sdk
echo ERROR: OpenHaptics not found at "%OH_SDK_BASE%".
echo Install OpenHaptics Developer Edition, or set OH_SDK_BASE to its folder.
exit /b 1
:no_lib
echo ERROR: hd.lib not found under "%OH_SDK_BASE%\lib\x64\...".
exit /b 1
:no_vs
echo ERROR: Visual Studio C++ compiler not found.
echo Install Visual Studio 2022 Community with "Desktop development with C++".
exit /b 1
