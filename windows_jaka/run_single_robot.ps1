[CmdletBinding()]
param(
    [string]$RegistryPath = "",
    [Parameter(Mandatory = $true)]
    [string]$RobotId,
    [ValidateSet("joint", "record", "playback")]
    [string]$ControlMode = "joint",
    [string]$RecordFile = "",
    [string]$PlaybackFile = "",
    [ValidateRange(0.1, 4.0)]
    [double]$PlaybackSpeed = 1.0,
    [string]$StatusDirectory = "",
    [string]$ControlPipe = "",
    [switch]$DryRun,
    [switch]$ArmMotion,
    [double]$DurationSec = 0,
    [string]$Configuration = "Release"
)

$ErrorActionPreference = "Stop"
if ($ArmMotion -and $env:JAKA_ENABLE_MOTION -ne "1") {
    throw "Real motion requires -ArmMotion and JAKA_ENABLE_MOTION=1."
}
if ($DryRun -and $ArmMotion) { throw "-DryRun and -ArmMotion are mutually exclusive." }
if ($ControlMode -eq "playback" -and -not $PlaybackFile) { throw "Playback requires -PlaybackFile." }

$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
if (-not $RegistryPath) { $RegistryPath = Join-Path $scriptRoot "..\config\robots.ini" }
$RegistryPath = [IO.Path]::GetFullPath($RegistryPath)
if (-not (Test-Path -LiteralPath $RegistryPath)) { throw "Robot registry not found: $RegistryPath" }
if (-not $StatusDirectory) { $StatusDirectory = Join-Path $scriptRoot "..\status" }
$StatusDirectory = [IO.Path]::GetFullPath($StatusDirectory)
New-Item -ItemType Directory -Path $StatusDirectory -Force | Out-Null
if (-not $ControlPipe) { $ControlPipe = "\\.\pipe\jaka_single_$($RobotId -replace '[^A-Za-z0-9_-]', '_')" }

function Read-IniFile([string]$Path) {
    $sections = [ordered]@{}
    $current = ""
    foreach ($rawLine in Get-Content -LiteralPath $Path) {
        $line = $rawLine.Trim()
        if (-not $line -or $line.StartsWith("#")) { continue }
        if ($line.StartsWith("[") -and $line.EndsWith("]")) {
            $current = $line.Substring(1, $line.Length - 2).Trim()
            $sections[$current] = [ordered]@{}
            continue
        }
        $equals = $line.IndexOf("=")
        if ($equals -lt 1 -or -not $current) { throw "Invalid registry line: $line" }
        $sections[$current][$line.Substring(0, $equals).Trim()] = $line.Substring($equals + 1).Trim()
    }
    return $sections
}

$sections = Read-IniFile $RegistryPath
$robot = $sections["robot:$RobotId"]
if ($null -eq $robot) { throw "Robot not found in registry: $RobotId" }
if ([string]$robot["enabled"] -in @("0", "false", "no", "off")) { throw "Robot is disabled: $RobotId" }

$candidateDirs = @(
    (Join-Path $scriptRoot "..\build\windows_jaka\$Configuration"),
    (Join-Path $scriptRoot "..\build\windows_jaka-manager-v9\Release"),
    (Join-Path $scriptRoot "..\build\windows_jaka-manager-v8\Release"),
    (Join-Path $scriptRoot "..\build\windows_jaka-manager-v7\Release"),
    (Join-Path $scriptRoot "..\build\windows_jaka-vs\Release"),
    (Join-Path $scriptRoot "..\build\windows_jaka-verify\Release"),
    (Join-Path $scriptRoot "..\build\windows_jaka-release")
)
$operatorExe = $null
foreach ($dir in $candidateDirs) {
    $candidate = Join-Path $dir "windows_operator.exe"
    if (Test-Path -LiteralPath $candidate) { $operatorExe = $candidate; break }
}
if (-not $operatorExe) { throw "windows_operator.exe was not found." }

if ($ControlMode -eq "record" -and -not $RecordFile) {
    $logDir = Join-Path $scriptRoot "..\logs"
    New-Item -ItemType Directory -Path $logDir -Force | Out-Null
    $stamp = Get-Date -Format "yyyyMMdd_HHmmss"
    $RecordFile = Join-Path $logDir "trajectory_$($RobotId)_$stamp.csv"
}
$statusFile = Join-Path $StatusDirectory "$RobotId.status"
$arguments = @(
    "--robot-id", $RobotId,
    "--operator-ip", [string]$robot["ip"],
    "--control-pipe", $ControlPipe,
    "--control-mode", $ControlMode,
    "--filter", [string]$robot["filter"],
    "--lpf-cutoff", [string]$robot["lpf_cutoff"],
    "--max-velocity", [string]$robot["max_velocity"],
    "--max-acceleration", [string]$robot["max_acceleration"],
    "--status-file", $statusFile,
    "--peer-port", "30999"
)
if ($ControlMode -eq "record") { $arguments += @("--record-file", "$RecordFile") }
if ($ControlMode -eq "playback") {
    $arguments += @("--playback-file", "$PlaybackFile", "--playback-speed", "$PlaybackSpeed")
}
if ($DurationSec -gt 0) { $arguments += @("--duration-sec", "$DurationSec") }
if ($ArmMotion) { $arguments += "--arm-motion" } else { $arguments += "--dry-run" }

Write-Host "Starting single robot $RobotId mode=$ControlMode ip=$($robot['ip']) pipe=$ControlPipe"
if ($RecordFile) { Write-Host "Record file: $RecordFile" }
if ($PlaybackFile) { Write-Host "Playback file: $PlaybackFile" }
& $operatorExe @arguments
exit $LASTEXITCODE
