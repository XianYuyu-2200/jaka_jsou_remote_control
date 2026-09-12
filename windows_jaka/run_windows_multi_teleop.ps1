[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$OperatorIp,
    [Parameter(Mandatory = $true)]
    [string[]]$FollowerIps,
    [ValidateRange(1, 65000)]
    [int]$BasePort = 30001,
    [switch]$DryRun,
    [switch]$ArmMotion,
    [ValidateSet("none", "lpf", "nlf")]
    [string]$Filter = "lpf",
    [double]$LpfCutoff = 2.5,
    [double]$MaxVelocity = 1.0,
    [double]$MaxAcceleration = 8.0,
    [string]$Configuration = "Release"
)

$ErrorActionPreference = "Stop"
if ($FollowerIps.Count -eq 0) { throw "At least one follower IP is required." }
if ($ArmMotion -and $env:JAKA_ENABLE_MOTION -ne "1") {
    throw "Real motion requires -ArmMotion and JAKA_ENABLE_MOTION=1."
}
if ($DryRun -and $ArmMotion) { throw "-DryRun and -ArmMotion are mutually exclusive." }

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$candidateDirs = @(
    (Join-Path $root "..\build\windows_jaka\$Configuration"),
    (Join-Path $root "..\build\windows_jaka-vs\Release"),
    (Join-Path $root "..\build\windows_jaka-verify\Release"),
    (Join-Path $root "..\build\windows_jaka-release")
)
$selected = $null
foreach ($dir in $candidateDirs) {
    $operator = Join-Path $dir "windows_operator.exe"
    $follower = Join-Path $dir "windows_follower.exe"
    if ((Test-Path -LiteralPath $operator) -and (Test-Path -LiteralPath $follower)) {
        $stamp = [math]::Max((Get-Item -LiteralPath $operator).LastWriteTimeUtc.Ticks,
                             (Get-Item -LiteralPath $follower).LastWriteTimeUtc.Ticks)
        if ($null -eq $selected -or $stamp -gt $selected.Stamp) {
            $selected = [pscustomobject]@{ Operator = $operator; Follower = $follower; Stamp = $stamp }
        }
    }
}
if ($null -eq $selected) { throw "Build outputs not found." }

$processes = @()
try {
    $peerPorts = @()
    for ($i = 0; $i -lt $FollowerIps.Count; ++$i) {
        $port = $BasePort + $i
        if ($port -gt 65535) { throw "BasePort leaves no room for all follower ports." }
        $peerPorts += $port
        $pipe = "\\.\pipe\jaka_multi_follower_$i"
        $followerArgs = @(
            "--follower-ip", $FollowerIps[$i],
            "--port", "$port",
            "--control-pipe", $pipe,
            "--control-mode", "teleop",
            "--filter", $Filter,
            "--lpf-cutoff", "$LpfCutoff",
            "--max-velocity", "$MaxVelocity",
            "--max-acceleration", "$MaxAcceleration"
        )
        if ($ArmMotion) { $followerArgs += "--arm-motion" } else { $followerArgs += "--dry-run" }
        Write-Host "Starting follower[$i] $($FollowerIps[$i]) udp=$port pipe=$pipe"
        $processes += Start-Process -FilePath $selected.Follower -ArgumentList $followerArgs -PassThru -NoNewWindow
    }

    Start-Sleep -Seconds 3
    foreach ($process in $processes) {
        $process.Refresh()
        if ($process.HasExited) { throw "A follower failed to start (exit code $($process.ExitCode))." }
    }

    $operatorArgs = @(
        "--operator-ip", $OperatorIp,
        "--control-pipe", "\\.\pipe\jaka_multi_operator",
        "--control-mode", "teleop",
        "--filter", $Filter,
        "--lpf-cutoff", "$LpfCutoff",
        "--max-velocity", "$MaxVelocity",
        "--max-acceleration", "$MaxAcceleration"
    )
    foreach ($port in $peerPorts) { $operatorArgs += @("--peer-port", "$port") }
    if ($ArmMotion) { $operatorArgs += "--arm-motion" } else { $operatorArgs += "--dry-run" }

    Write-Host "Starting operator $OperatorIp -> followers=$($FollowerIps -join ',') ports=$($peerPorts -join ',')"
    $operator = Start-Process -FilePath $selected.Operator -ArgumentList $operatorArgs -PassThru -NoNewWindow
    $processes += $operator

    Write-Host "Multi-follower session running. Press Ctrl+C to stop all."
    while ($true) {
        Start-Sleep -Milliseconds 250
        foreach ($process in $processes) {
            $process.Refresh()
            if ($process.HasExited) { throw "A session process exited unexpectedly (exit code $($process.ExitCode))." }
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
