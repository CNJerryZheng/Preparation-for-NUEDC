[CmdletBinding()]
param(
    [string]$SysConfigRoot = $env:SYSCONFIG_ROOT,
    [string]$Mspm0SdkRoot = $env:MSPM0_SDK_ROOT
)

$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $PSScriptRoot
$syscfgFile = Join-Path $projectRoot "Core\empty.syscfg"
$outputDir = Join-Path $projectRoot "build\syscfg"

if (-not (Test-Path -LiteralPath $syscfgFile)) {
    throw "SysConfig file not found: $syscfgFile"
}

$sysConfigCli = if ($SysConfigRoot -and $SysConfigRoot.EndsWith(".bat")) {
    $SysConfigRoot
} elseif ($SysConfigRoot) {
    Join-Path $SysConfigRoot "sysconfig_cli.bat"
}

if (-not $sysConfigCli -or -not (Test-Path -LiteralPath $sysConfigCli)) {
    throw "SysConfig CLI not found. Set SYSCONFIG_ROOT or pass -SysConfigRoot."
}

$syscfgText = Get-Content -LiteralPath $syscfgFile -Raw -Encoding UTF8
$sdkVersionMatch = [regex]::Match($syscfgText, "mspm0_sdk@([0-9.]+)")
if (-not $sdkVersionMatch.Success) {
    throw "MSPM0 SDK version is missing from empty.syscfg."
}

$sdkVersion = $sdkVersionMatch.Groups[1].Value
$sdkFolderName = "mspm0_sdk_" + ($sdkVersion -replace "\.", "_")

$productJson = if ($Mspm0SdkRoot -and $Mspm0SdkRoot.EndsWith(".json")) {
    $Mspm0SdkRoot
} elseif ($Mspm0SdkRoot) {
    Join-Path $Mspm0SdkRoot ".metadata\product.json"
}

if (-not $productJson -or -not (Test-Path -LiteralPath $productJson)) {
    throw "MSPM0 SDK $sdkVersion was not found. Set MSPM0_SDK_ROOT or pass -Mspm0SdkRoot."
}

New-Item -ItemType Directory -Force -Path $outputDir | Out-Null

& $sysConfigCli `
    -s $productJson `
    -d "MSPM0G3507" `
    -p "LQFP-48(PT)" `
    -o $outputDir `
    $syscfgFile

if ($LASTEXITCODE -ne 0) {
    throw "SysConfig generation failed with exit code $LASTEXITCODE."
}

Copy-Item -Force -LiteralPath (Join-Path $outputDir "ti_msp_dl_config.c") `
    -Destination (Join-Path $projectRoot "Core\Src\ti_msp_dl_config.c")
Copy-Item -Force -LiteralPath (Join-Path $outputDir "ti_msp_dl_config.h") `
    -Destination (Join-Path $projectRoot "Core\Inc\ti_msp_dl_config.h")

$eventDot = Join-Path $outputDir "excluded\Event.dot"
if (Test-Path -LiteralPath $eventDot) {
    Copy-Item -Force -LiteralPath $eventDot `
        -Destination (Join-Path $projectRoot "Core\excluded\Event.dot")
}

Write-Host "SysConfig files updated successfully:" -ForegroundColor Green
Write-Host "  Core/Src/ti_msp_dl_config.c"
Write-Host "  Core/Inc/ti_msp_dl_config.h"
