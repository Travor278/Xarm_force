[CmdletBinding()]
param(
    [string]$RemoteHost = "192.168.105.166",
    [string]$RemoteUser = "dell",
    [ValidateRange(1, 65535)][int]$LocalPort = 8765,
    [ValidateRange(1, 65535)][int]$RemotePort = 18765,
    [switch]$NoBrowser,
    [switch]$PrintOnly
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$deployment = "/home/dell/piperx-force-validation"
$remoteCommand = "cd $deployment && exec bash scripts/run_piperx_teleop_web_remote.sh $RemotePort"
$forward = "${LocalPort}:127.0.0.1:${RemotePort}"
$destination = "${RemoteUser}@${RemoteHost}"
$displayCommand = "ssh -tt -o ExitOnForwardFailure=yes -L $forward $destination `"$remoteCommand`""

if ($PrintOnly) {
    Write-Output $displayCommand
    exit 0
}

$listener = Get-NetTCPConnection -State Listen -LocalPort $LocalPort -ErrorAction SilentlyContinue
if ($listener) {
    throw "Local port $LocalPort is already listening. Stop that process or choose -LocalPort."
}

Write-Host "Starting standalone PiperX teleop + torque dashboard..." -ForegroundColor Cyan
Write-Host "Keep both leader and follower arms still until alignment passes." -ForegroundColor Yellow
Write-Host "SSH and sudo may request credentials in the visible window." -ForegroundColor DarkGray

$escapedRemote = $remoteCommand.Replace('"', '\"')
$sshArguments = @(
    "-tt",
    "-o", "ExitOnForwardFailure=yes",
    "-L", $forward,
    $destination,
    "`"$escapedRemote`""
)
$sshProcess = Start-Process -FilePath "ssh.exe" -ArgumentList $sshArguments -PassThru

$url = "http://127.0.0.1:$LocalPort"
$healthUrl = "$url/healthz"
$ready = $false
for ($attempt = 0; $attempt -lt 120; $attempt++) {
    if ($sshProcess.HasExited) {
        throw "SSH exited before standalone teleop became ready (exit $($sshProcess.ExitCode))."
    }
    try {
        $health = Invoke-RestMethod -Uri $healthUrl -TimeoutSec 1
        if ($health.ok) {
            $ready = $true
            break
        }
    } catch {
        Start-Sleep -Milliseconds 500
    }
}

if (-not $ready) {
    if (-not $sshProcess.HasExited) {
        Stop-Process -Id $sshProcess.Id -ErrorAction SilentlyContinue
    }
    throw "Standalone teleop did not become healthy within 60 seconds. Inspect the SSH window."
}

Write-Host "PiperX standalone teleop and dashboard ready: $url" -ForegroundColor Green
Write-Host "Close the SSH window to stop teleop/Web and restore EvoStudio." -ForegroundColor Yellow
if (-not $NoBrowser) {
    Start-Process $url
}
