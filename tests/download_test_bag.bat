@echo off
setlocal enabledelayedexpansion

REM Download script for test ROS bag file
REM This downloads a sample bag file for testing bag2rrd

REM Get the directory where this script is located
set "SCRIPT_DIR=%~dp0"
REM Remove trailing backslash if present
if "%SCRIPT_DIR:~-1%"=="\" set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"

set "BAG_URL=https://download.ifi.uzh.ch/rpg/drone_racing_data/race_1.bag"
set "BAG_FILE=%SCRIPT_DIR%\data\race_1.bag"

echo Downloading ROS bag file for testing...
echo URL: %BAG_URL%
echo Destination: %BAG_FILE%

if exist "%BAG_FILE%" (
    echo File already exists: %BAG_FILE%
    echo Skipping download. Remove the file if you want to re-download.
    goto :eof
)

REM Create data directory if it doesn't exist
if not exist "%SCRIPT_DIR%\data" (
    mkdir "%SCRIPT_DIR%\data"
)

REM Download with progress bar
echo Downloading...
curl -L -o "%BAG_FILE%" --progress-bar "%BAG_URL%"

if %errorlevel% neq 0 (
    echo ERROR: Download failed!
    exit /b 1
)

REM Get file size
for %%A in ("%BAG_FILE%") do set "FILE_SIZE=%%~zA"
set /a "FILE_SIZE_KB=%FILE_SIZE%/1024"
echo Download complete!
echo File size: %FILE_SIZE_KB% KB
echo.
echo You can now test bag2rrd with:
echo   cargo run -- inspect "%BAG_FILE%"
echo   cargo run -- convert "%BAG_FILE%" test_output.rrd

goto :eof
