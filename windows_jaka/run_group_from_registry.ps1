[CmdletBinding()]
param(
    [string]$RegistryPath = "",
    [Parameter(Mandatory = $true)]
    [string]$GroupId,
    [ValidateRange(1, 65000)]
    [int]$BasePort = 30101,
    [switch]$DryRun,
    [switch]$ArmMotion,
    [double]$DurationSec = 0,
    [string]$StatusDirectory = "",
    [string]$Configuration = "Release"
)

$ErrorActionPreference = "Stop"
if ($ArmMotion -and $env:JAKA_ENABLE_MOTION -ne "1") {
    throw "Real motion requires -ArmMotion and JAKA_ENABLE_MOTION=1."
}
if ($DryRun -and $ArmMotion) { throw "-DryRun and -ArmMotion are mutually exclusive." }

$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
if (-not $RegistryPath) {
    $RegistryPath = Join-Path $scriptRoot "..\config\robots.ini"
}
$RegistryPath = [IO.Path]::GetFullPath($RegistryPath)
if (-not $StatusDirectory) { $StatusDirectory = Join-Path $scriptRoot "..\status" }
$StatusDirectory = [IO.Path]::GetFullPath($StatusDirectory)
New-Item -ItemType Directory -Path $StatusDirectory -Force | Out-Null
if (-not (Test-Path -LiteralPath $RegistryPath)) {
    throw "Robot registry not found: $RegistryPath"
}

function Read-IniFile([string]$Path) {
    $sections = [ordered]@{}
    $current = ""
    $lineNumber = 0
    foreach ($rawLine in Get-Content -LiteralPath $Path) {
        ++$lineNumber
        $line = $rawLine.Trim()
        if (-not $line -or $line.StartsWith("#")) { continue }
        if ($line.StartsWith("[") -and $line.EndsWith("]")) {
            $current = $line.Substring(1, $line.Length - 2).Trim()
            if ($current -notin $sections.Keys) { $sections[$current] = [ordered]@{} }
            continue
        }
        $equals = $line.IndexOf("=")
        if ($equals -lt 1 -or -not $current) {
            throw "Invalid registry line $lineNumber"
        }
        $key = $line.Substring(0, $equals).Trim()
        $value = $line.Substring($equals + 1).Trim()
        $sections[$current][$key] = $value
    }
    return $sections
}

function Split-Csv([string]$Value) {
    return @($Value -split "," | ForEach-Object { $_.Trim() } | Where-Object { $_ })
}

$sections = Read-IniFile $RegistryPath
$groupSection = $sections["group:$GroupId"]
if ($null -eq $groupSection) { throw "Teleop group not found: $GroupId" }
$operatorId = [string]$groupSection["operator"]
$followerIds = Split-Csv ([string]$groupSection["followers"])
if (-not $operatorId -or $followerIds.Count -eq 0) {
    throw "Group $GroupId must contain one operator and at least one follower."
}

$operatorSection = $sections["robot:$operatorId"]
if ($null -eq $operatorSection) { throw "Operator robot not found: $operatorId" }
$robots = @{}
foreach ($id in @($operatorId) + $followerIds) {
    $section = $sections["robot:$id"]
    if ($null -eq $section) { throw "Robot not found in registry: $id" }
    if ([string]$section["enabled"] -in @("0", "false", "no", "off")) { throw "Robot is disabled: $id" }
    $robots[$id] = $section
}

$candidateDirs = @(
    (Join-Path $scriptRoot "..\build\windows_jaka\$Configuration"),
    (Join-Path $scriptRoot "..\build\windows_jaka-vs\Release"),
    (Join-Path $scriptRoot "..\build\windows_jaka-verify\Release"),
    (Join-Path $scriptRoot "..\build\windows_jaka-release")
)
$selected = $null
foreach ($dir in $candidateDirs) {
    $operatorExe = Join-Path $dir "windows_operator.exe"
    $followerExe = Join-Path $dir "windows_follower.exe"
    if ((Test-Path -LiteralPath $operatorExe) -and (Test-Path -LiteralPath $followerExe)) {
        $selected = [pscustomobject]@{ Operator=$operatorExe; Follower=$followerExe }
        break
    }
}
if ($null -eq $selected) { throw "Windows operator/follower executables were not found." }

$processes = @()
try {
    $peerPorts = @()
    for ($i = 0; $i -lt $followerIds.Count; ++$i) {
        $id = $followerIds[$i]
        $section = $robots[$id]
        $port = $BasePort + $i
        if ($port -gt 65535) { throw "BasePort leaves no room for all followers." }
        $peerPorts += $port
        $pipe = "\\.\pipe\jaka_multi_follower_$($id -replace '[^A-Za-z0-9_-]', '_')"
        $followerArgs = @(
            "--robot-id", $id,
            "--follower-ip", [string]$section["ip"],
            "--port", "$port",
            "--control-pipe", $pipe,
            "--control-mode", "teleop",
            "--filter", [string]$section["filter"],
            "--lpf-cutoff", [string]$section["lpf_cutoff"],
            "--max-velocity", [string]$section["max_velocity"],
            "--max-acceleration", [string]$section["max_acceleration"],
            "--status-file", (Join-Path $StatusDirectory "$id.status")
        )
        if ($DurationSec -gt 0) { $followerArgs += @("--duration-sec", "$DurationSec") }
        if ($ArmMotion) { $followerArgs += "--arm-motion" } else { $followerArgs += "--dry-run" }
        Write-Host "Starting follower $id ip=$($section['ip']) udp=$port pipe=$pipe"
        $processes += Start-Process -FilePath $selected.Follower -ArgumentList $followerArgs -PassThru -NoNewWindow
    }

    Start-Sleep -Seconds 3
    foreach ($process in $processes) {
        $process.Refresh()
        if ($process.HasExited) { throw "A follower failed to start (exit code $($process.ExitCode))." }
    }

    $operatorPipe = "\\.\pipe\jaka_multi_operator_$($GroupId -replace '[^A-Za-z0-9_-]', '_')"
    $operatorArgs = @(
        "--robot-id", $operatorId,
        "--operator-ip", [string]$operatorSection["ip"],
        "--control-pipe", $operatorPipe,
        "--control-mode", "teleop",
        "--filter", [string]$operatorSection["filter"],
        "--lpf-cutoff", [string]$operatorSection["lpf_cutoff"],
        "--max-velocity", [string]$operatorSection["max_velocity"],
        "--max-acceleration", [string]$operatorSection["max_acceleration"],
        "--status-file", (Join-Path $StatusDirectory "$operatorId.status")
    )
    foreach ($port in $peerPorts) { $operatorArgs += @("--peer-port", "$port") }
    if ($DurationSec -gt 0) { $operatorArgs += @("--duration-sec", "$DurationSec") }
    if ($ArmMotion) { $operatorArgs += "--arm-motion" } else { $operatorArgs += "--dry-run" }

    Write-Host "Starting operator $operatorId ip=$($operatorSection['ip']) followers=$($followerIds -join ',')"
    $operator = Start-Process -FilePath $selected.Operator -ArgumentList $operatorArgs -PassThru -NoNewWindow
    $processes += $operator

    Write-Host "Group $GroupId running. Press Ctrl+C to stop the group."
    while ($true) {
        Start-Sleep -Milliseconds 250
        foreach ($process in $processes) {
            $process.Refresh()
            if ($process.HasExited) { throw "A group process exited unexpectedly (exit code $($process.ExitCode))." }
        }
    }
}
finally {
    foreach ($process in $processes) {
        if ($process -and -not $process.HasExited) {
            try { Stop-Process -Id $process.Id -Force } catch { }
        }
    }
}
