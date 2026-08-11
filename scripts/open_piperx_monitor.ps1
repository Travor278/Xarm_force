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
$urdf = "/home/dell/Evo-RL.before-pr-sync/src/lerobot/assets/piper_x_description/urdf/piper_x_description_no_gripper.urdf"
$payload = "$deployment/config/piperx_gripper_payload.json"
$remoteCommand = @(
    "cd $deployment && exec /home/dell/anaconda3/bin/conda run --no-capture-output -n evo-rl",
    "python scripts/piperx_torque_web.py",
    "--urdf $urdf",
    "--payload $payload",
    "--arm left,004B00204148570D20343133,calibration/left.json",
    "--arm right,003F002D4148571320343133,calibration/right.json",
    "--host 127.0.0.1 --port $RemotePort --ui-rate 50"
) -join " "

$forward = "${LocalPort}:127.0.0.1:${RemotePort}"
$destination = "${RemoteUser}@${RemoteHost}"
$displayCommand = "ssh -o ExitOnForwardFailure=yes -L $forward $destination `"$remoteCommand`""

if ($PrintOnly) {
    Write-Output $displayCommand
    exit 0
}

$listener = Get-NetTCPConnection -State Listen -LocalPort $LocalPort -ErrorAction SilentlyContinue
if ($listener) {
    throw "Local port $LocalPort is already listening. Stop that process or choose -LocalPort."
}

Write-Host "Starting receive-only PiperX monitor through SSH..." -ForegroundColor Cyan
Write-Host "The SSH window is interactive and may prompt for the remote password." -ForegroundColor DarkGray

# This window stays visible because it carries an interactive SSH prompt and the
# monitor process. Closing it stops the tunnel and remote monitor cleanly.
$escapedRemote = $remoteCommand.Replace('"', '\"')
$sshArguments = @(
    "-o", "ExitOnForwardFailure=yes",
    "-L", $forward,
    $destination,
    "`"$escapedRemote`""
)
$sshProcess = Start-Process -FilePath "ssh.exe" -ArgumentList $sshArguments -PassThru

$url = "http://127.0.0.1:$LocalPort"
$healthUrl = "$url/healthz"
$ready = $false
for ($attempt = 0; $attempt -lt 60; $attempt++) {
    if ($sshProcess.HasExited) {
        throw "SSH exited before the monitor became ready (exit $($sshProcess.ExitCode))."
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
    throw "Monitor did not become healthy within 30 seconds. Inspect the SSH window."
}

Write-Host "PiperX monitor ready: $url" -ForegroundColor Green
Write-Host "SSH PID: $($sshProcess.Id). Close that window or stop the process to disconnect." -ForegroundColor DarkGray
if (-not $NoBrowser) {
    Start-Process $url
}
