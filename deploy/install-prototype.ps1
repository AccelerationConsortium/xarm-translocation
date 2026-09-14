# Installs only the NEW read-only prototype. Never stops another service.
[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)][string]$BindAddress,
    [switch]$Apply
)
$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'
$project = Split-Path -Parent $PSScriptRoot
$python = Join-Path $project '.venv\Scripts\python.exe'
$config = Join-Path $project '.state\robot-motion.local.json'
$nssm = 'C:\SDL_Tools\nssm.exe'
$serviceName = 'robot-motion-prototype'
$ruleName = 'robot-motion-prototype-8075'
$port = 8075

if ((Split-Path $project -Leaf) -ne 'robot-motion') { throw 'Unexpected project directory' }
foreach ($path in @($python,$config,$nssm)) {
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "Missing prerequisite: $path" }
}
$ip = [Net.IPAddress]::Parse($BindAddress)
$octets = $ip.GetAddressBytes()
if ($ip.AddressFamily -ne [Net.Sockets.AddressFamily]::InterNetwork -or
    $octets[0] -ne 100 -or $octets[1] -lt 64 -or $octets[1] -gt 127) {
    throw 'Bind only to the explicitly selected Tailnet IPv4 address'
}
if (-not (Get-NetIPAddress -IPAddress $BindAddress -ErrorAction SilentlyContinue)) {
    throw 'Selected Tailnet address is not assigned to this PC'
}
if (Get-Service -Name $serviceName -ErrorAction SilentlyContinue) {
    throw 'Prototype service already exists; review its configuration before updating'
}
if (Get-NetTCPConnection -State Listen -LocalPort $port -ErrorAction SilentlyContinue) {
    throw 'Port 8075 is occupied; no existing listener will be terminated'
}
if (Get-NetFirewallRule -Name $ruleName -ErrorAction SilentlyContinue) {
    throw 'Firewall rule already exists; review it rather than overwriting'
}
$parsed = Get-Content -LiteralPath $config -Raw | ConvertFrom-Json
if ($parsed.control_enabled -ne $false) { throw 'Prototype must explicitly disable control' }
& $python -c "from pathlib import Path; from robot_motion.config import load_settings; import sys; load_settings(Path(sys.argv[1]))" $config
if ($LASTEXITCODE -ne 0) { throw 'Invalid local application configuration' }
Write-Output "Preflight passed: new read-only service, Tailnet $BindAddress port 8075."
if (-not $Apply) { Write-Output 'Dry run; pass -Apply to install.'; return }

$admin = [Security.Principal.WindowsPrincipal]::new([Security.Principal.WindowsIdentity]::GetCurrent())
if (-not $admin.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    throw 'An administrator is required to register the new NSSM service'
}
$logs = Join-Path $project '.state\logs'
New-Item -ItemType Directory -Path $logs -Force | Out-Null

function Invoke-Nssm([string[]]$Arguments) {
    & $nssm @Arguments
    if ($LASTEXITCODE -ne 0) { throw "NSSM command failed: $($Arguments[0])" }
}
# LocalSystem is restricted to this read-only prototype. Future physical
# control requires a separate reviewed service-account/commissioning deployment.
Invoke-Nssm -Arguments @('install',$serviceName,$python)
Invoke-Nssm -Arguments @('set',$serviceName,'AppDirectory',$project)
$parameters = '-m robot_motion serve --config "' + $config + '" --host ' + $BindAddress + ' --port 8075'
Invoke-Nssm -Arguments @('set',$serviceName,'AppParameters',$parameters)
Invoke-Nssm -Arguments @('set',$serviceName,'DisplayName','Robot Motion (Prototyping)')
Invoke-Nssm -Arguments @('set',$serviceName,'Description','Read-only UR observation and offline graph UI; no physical robot control')
Invoke-Nssm -Arguments @('set',$serviceName,'Start','SERVICE_AUTO_START')
Invoke-Nssm -Arguments @('set',$serviceName,'AppExit','Default','Restart')
Invoke-Nssm -Arguments @('set',$serviceName,'AppRestartDelay','5000')
Invoke-Nssm -Arguments @('set',$serviceName,'AppStdout',(Join-Path $logs 'stdout.log'))
Invoke-Nssm -Arguments @('set',$serviceName,'AppStderr',(Join-Path $logs 'stderr.log'))
Invoke-Nssm -Arguments @('set',$serviceName,'AppRotateFiles','1')
Invoke-Nssm -Arguments @('set',$serviceName,'AppRotateOnline','1')
Invoke-Nssm -Arguments @('set',$serviceName,'AppRotateBytes','10485760')
Invoke-Nssm -Arguments @('set',$serviceName,'AppEnvironmentExtra','PYTHONUNBUFFERED=1')
New-NetFirewallRule -Name $ruleName -DisplayName 'Robot Motion prototype 8075 (Tailnet only)' -Direction Inbound -Action Allow -Protocol TCP -LocalAddress $BindAddress -LocalPort $port -RemoteAddress '100.64.0.0/10' -Profile Any -Program $python | Out-Null
Start-Service -Name $serviceName
$service = Get-Service -Name $serviceName
if ($service.Status -eq 'Paused') { Resume-Service -Name $serviceName }
$service.WaitForStatus('Running',[TimeSpan]::FromSeconds(15))
Write-Output 'New prototype service is running. Existing robot services were not restarted.'
