[CmdletBinding()]
param(
    [string]$OperatorIp = "192.168.0.101",
    [string]$FollowerIp = "192.168.0.102",
    [ValidateRange(1, 65535)]
    [int]$Port = 30001,
    [switch]$DryRun,
    [switch]$ArmMotion,
    [switch]$Offline,
    [ValidateSet("none", "lpf", "nlf")]
    [string]$Filter = "none",
    [double]$LpfCutoff = 2.5,
    [double]$MaxVelocity = 1.0,
    [double]$MaxAcceleration = 8.0,
    [string]$ControlPipe = "\\.\pipe\jaka_dual_teleop",
    [ValidateSet("idle", "teleop", "joint", "record", "playback")]
    [string]$ControlMode = "teleop",
    [string]$OperatorControlPipe = "\\.\pipe\jaka_operator_teleop",
    [ValidateSet("idle", "teleop", "joint", "record", "playback")]
    [string]$OperatorControlMode = "idle",
    [string]$RecordFile = "",
    [string]$FollowerRecordFile = "",
    [string]$FollowerPlaybackFile = "",
    [string]$PlaybackFile = "",
    [ValidateRange(0.1, 4.0)]
    [double]$PlaybackSpeed = 1.0,
    [double]$DurationSec = 0,
    [string]$Configuration = "Release"
)

$ErrorActionPreference = "Stop"
if ($DurationSec -lt 0) { throw "-DurationSec must be non-negative." }
if ($LpfCutoff -le 0) { throw "-LpfCutoff must be positive." }
if ($MaxVelocity -le 0) { throw "-MaxVelocity must be positive." }
if ($MaxAcceleration -le 0) { throw "-MaxAcceleration must be positive." }
if ($PlaybackSpeed -le 0 -or $PlaybackSpeed -gt 4.0) { throw "-PlaybackSpeed must be in (0, 4]." }
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$candidateDirs = @(
    (Join-Path $root "..\build\windows_jaka\$Configuration"),
    (Join-Path $root "..\build\windows_jaka-vs\Release"),
    (Join-Path $root "..\build\windows_jaka-verify\Release"),
    (Join-Path $root "..\build\windows_jaka-release")
)
$candidates = foreach ($dir in $candidateDirs) {
    $op = Join-Path $dir "windows_operator.exe"
    $fol = Join-Path $dir "windows_follower.exe"
    if ((Test-Path -LiteralPath $op) -and (Test-Path -LiteralPath $fol)) {
        $opInfo = Get-Item -LiteralPath $op
        $folInfo = Get-Item -LiteralPath $fol
        [pscustomobject]@{ Operator = $op; Follower = $fol; Stamp = [math]::Max($opInfo.LastWriteTimeUtc.Ticks, $folInfo.LastWriteTimeUtc.Ticks) }
    }
}
if ($candidates) {
    $selected = $candidates | Sort-Object Stamp -Descending | Select-Object -First 1
    $operatorExe = $selected.Operator
    $followerExe = $selected.Follower
}

if ($ArmMotion -and $env:JAKA_ENABLE_MOTION -ne "1") {
    throw "Real motion requires both -ArmMotion and JAKA_ENABLE_MOTION=1."
}
if ($Offline -and $ArmMotion) {
    throw "-Offline cannot be combined with -ArmMotion."
}
if (-not (Test-Path -LiteralPath $operatorExe) -or -not (Test-Path -LiteralPath $followerExe)) {
    throw "Build outputs not found. Configure and build windows_jaka first."
}
Write-Host "Using follower executable: $followerExe"
Write-Host "Using operator executable: $operatorExe"

$followerArgs = @("--follower-ip", $FollowerIp, "--port", "$Port",
                  "--max-velocity", "$MaxVelocity",
                  "--max-acceleration", "$MaxAcceleration",
                  "--control-pipe", "$ControlPipe",
                  "--control-mode", "$ControlMode")
if ($FollowerRecordFile) { $followerArgs += @("--record-file", "$FollowerRecordFile") }
if ($FollowerPlaybackFile) { $followerArgs += @("--playback-file", "$FollowerPlaybackFile", "--playback-speed", "$PlaybackSpeed") }
if ($Filter -ne "none" -or $LpfCutoff -ne 2.5) { $followerArgs += @("--filter", $Filter, "--lpf-cutoff", "$LpfCutoff") }
if ($DurationSec -gt 0) { $followerArgs += @("--duration-sec", "$DurationSec") }
if ($ArmMotion) { $followerArgs += "--arm-motion" } else { $followerArgs += "--dry-run" }
if ($Offline) { $followerArgs += "--offline" }

$operatorArgs = @("--operator-ip", $OperatorIp, "--port", "$Port",
                  "--control-pipe", "$OperatorControlPipe",
                  "--control-mode", "$OperatorControlMode",
                  "--filter", "$Filter",
                  "--lpf-cutoff", "$LpfCutoff",
                  "--max-velocity", "$MaxVelocity",
                  "--max-acceleration", "$MaxAcceleration")
if ($DurationSec -gt 0) { $operatorArgs += @("--duration-sec", "$DurationSec") }
if ($RecordFile) { $operatorArgs += @("--record-file", $RecordFile) }
if ($PlaybackFile) { $operatorArgs += @("--playback-file", "$PlaybackFile", "--playback-speed", "$PlaybackSpeed") }
if ($ArmMotion) { $operatorArgs += "--arm-motion" } else { $operatorArgs += "--dry-run" }
$follower = $null
$operator = $null
try {
    Write-Host "Starting follower: $FollowerIp (port $Port)"
    $follower = Start-Process -FilePath $followerExe -ArgumentList $followerArgs -PassThru -NoNewWindow
    # Give the follower enough time to load jakaAPI.dll, login, and perform
    # the required powered/enabled check. Do not start the operator if the
    # follower has already failed; otherwise the GUI appears half-connected.
    for ($i = 0; $i -lt 12; $i++) {
        Start-Sleep -Milliseconds 250
        $follower.Refresh()
        if ($follower.HasExited) {
            throw "Follower failed to start (exit code $($follower.ExitCode)). Check that the follower robot is powered and enabled."
        }
    }

    Write-Host "Starting operator: $OperatorIp -> localhost:$Port"
    $operator = Start-Process -FilePath $operatorExe -ArgumentList $operatorArgs -PassThru -NoNewWindow

    Write-Host "Press Ctrl+C to stop both processes."
    # Wait for both children. One process may finish its own safety cleanup
    # slightly before the other, so treating the first exit as a reason to
    # tear down the session can terminate a healthy process too early.
    while (-not $operator.HasExited -or -not $follower.HasExited) {
        Start-Sleep -Milliseconds 250
    }
}
finally {
    # Console children do not have a useful main window to close. Let them
    # finish their own SDK cleanup, then force-kill only as a last resort.
    $processes = @(@($operator, $follower) | Where-Object { $null -ne $_ })
    # Give both processes time to abort servo motion, disable servo mode, and
    # log out. One second was too short for the operator SDK cleanup on some
    # runs, so the wrapper could force-terminate a process that was still
    # performing its safety cleanup.
    $deadline = [DateTime]::UtcNow.AddSeconds(3)
    do {
        $alive = $false
        foreach ($process in $processes) {
            $process.Refresh()
            if (-not $process.HasExited) { $alive = $true }
        }
        if ($alive) { Start-Sleep -Milliseconds 100 }
    } while ($alive -and [DateTime]::UtcNow -lt $deadline)

    foreach ($process in $processes) {
        if (-not $process.HasExited) {
            Write-Warning "Process $($process.Id) did not exit gracefully; forcing termination."
            Stop-Process -Id $process.Id -Force
        }
    }
}

if ($operator) {
    try { $operator.WaitForExit() } catch { }
    $operator.Refresh()
    Write-Host "operator exit code: $($operator.ExitCode)"
}
if ($follower) {
    try { $follower.WaitForExit() } catch { }
    $follower.Refresh()
    Write-Host "follower exit code: $($follower.ExitCode)"
}
