# GUARD.IAM — rebuild the PlatformIO project folder structure.
# Run from:  C:\Users\Admin\Downloads\guardiam\quest_headpose_test
#
# Usage in PowerShell:
#   cd C:\Users\Admin\Downloads\guardiam\quest_headpose_test
#   powershell -ExecutionPolicy Bypass -File setup_esp32.ps1

$root = "C:\Users\Admin\Downloads\guardiam\quest_headpose_test"
$proj = Join-Path $root "esp32_ptz"
$src  = Join-Path $proj "src"

New-Item -ItemType Directory -Force -Path $src | Out-Null

# Move main.cpp into esp32_ptz\src\
$mainSrc = Join-Path $root "main.cpp"
if (Test-Path $mainSrc) {
    Move-Item -Force $mainSrc (Join-Path $src "main.cpp")
    Write-Host "Moved main.cpp -> esp32_ptz\src\main.cpp"
} elseif (Test-Path (Join-Path $src "main.cpp")) {
    Write-Host "main.cpp already in place."
} else {
    Write-Host "WARNING: main.cpp not found in $root"
}

# Write platformio.ini (it didn't survive the download).
$ini = @'
[env:esp32-s3-devkitc-1]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 115200
lib_deps =
    madhephaestus/ESP32Servo @ ^3.0.5

build_flags =
    -D ARDUINO_USB_MODE=1
    -D ARDUINO_USB_CDC_ON_BOOT=1
'@
Set-Content -Path (Join-Path $proj "platformio.ini") -Value $ini -Encoding UTF8
Write-Host "Wrote esp32_ptz\platformio.ini"

Write-Host ""
Write-Host "Done. In VS Code: File -> Open Folder -> $proj"
