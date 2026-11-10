@echo off
setlocal
if not defined MCWP_KIT set "MCWP_KIT=%USERPROFILE%\NVIDIA\Omniverse\Kit-110.4.0\_build\windows-x86_64\release"
if not exist "%MCWP_KIT%\kit\kit.exe" (
    echo Kit 110.4 was not found. Set MCWP_KIT to the SDK release directory containing kit\kit.exe and extscache.
    exit /b 1
)
if not exist "%~dp0apps\mcwp.walking.kit" (
    echo Missing project application configuration: apps\mcwp.walking.kit
    exit /b 1
)
if not exist "%MCWP_KIT%\extscache" (
    echo Missing Kit extension cache. Prepare the Kit 110.4 base-editor environment first.
    exit /b 1
)
if not exist "%~dp0evidence" mkdir "%~dp0evidence"
if not exist "%~dp0recordings" mkdir "%~dp0recordings"
"%MCWP_KIT%\kit\kit.exe" "%~dp0apps\mcwp.walking.kit" --ext-folder "%MCWP_KIT%\exts" --ext-folder "%MCWP_KIT%\extscache" --exec "%~dp0scripts\mcwp_simulation.py" --/app/window/width=1360 --/app/window/height=760 %*
exit /b %errorlevel%
