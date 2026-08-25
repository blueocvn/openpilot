[CmdletBinding()]
param(
  [ValidateSet('setup', 'start', 'status', 'stop', 'firewall-add', 'firewall-remove')]
  [string]$Command = 'start',
  [switch]$Visible,
  [int]$Port = 2000,
  [string]$WslDistro = 'Ubuntu',
  [string]$WslRemoteAddress = ''
)

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..\..\..')).Path
$StateRoot = Join-Path $RepoRoot '.carla'
$DownloadRoot = Join-Path $StateRoot 'downloads'
$InstallRoot = Join-Path $StateRoot 'CARLA_0.9.16'
$Archive = Join-Path $DownloadRoot 'CARLA_0.9.16.zip'
$DownloadUrl = 'https://tiny.carla.org/carla-0-9-16-windows'
$LogRoot = Join-Path $StateRoot 'logs'
$FirewallRuleName = 'openpilot CARLA WSL'
$FirewallLog = Join-Path $StateRoot 'firewall-events.jsonl'
$ModeFile = Join-Path $StateRoot 'carla-mode.txt'
$CarlaShippingExe = Join-Path $InstallRoot 'CarlaUE4\Binaries\Win64\CarlaUE4-Win64-Shipping.exe'

function Initialize-DDriveEnvironment {
  New-Item -ItemType Directory -Force -Path $StateRoot, $DownloadRoot, $LogRoot | Out-Null
  $env:TEMP = Join-Path $StateRoot 'tmp'
  $env:TMP = $env:TEMP
  $env:CARLA_CACHE_DIR = Join-Path $StateRoot 'cache'
  ${env:UE-LocalDataCachePath} = Join-Path $StateRoot 'ue-cache'
  $env:USERPROFILE = Join-Path $StateRoot 'windows-profile'
  ${env:HOMEDRIVE} = Split-Path -Qualifier $env:USERPROFILE
  ${env:HOMEPATH} = $env:USERPROFILE.Substring(${env:HOMEDRIVE}.Length)
  $env:APPDATA = Join-Path $env:USERPROFILE 'AppData\Roaming'
  $env:LOCALAPPDATA = Join-Path $env:USERPROFILE 'AppData\Local'
  New-Item -ItemType Directory -Force -Path $env:TEMP, $env:CARLA_CACHE_DIR, `
    ${env:UE-LocalDataCachePath}, $env:APPDATA, $env:LOCALAPPDATA | Out-Null
}

function Install-Carla {
  Initialize-DDriveEnvironment
  if (Test-Path (Join-Path $InstallRoot 'CarlaUE4.exe')) {
    Write-Host "CARLA 0.9.16 is already installed at $InstallRoot"
    return
  }
  if (-not (Test-Path $Archive)) {
    Write-Host "Downloading CARLA 0.9.16 to $Archive"
    & curl.exe -L --fail --retry 5 --output $Archive $DownloadUrl
    if ($LASTEXITCODE -ne 0) { throw 'CARLA download failed' }
  }
  New-Item -ItemType Directory -Force -Path $InstallRoot | Out-Null
  Write-Host "Extracting CARLA to $InstallRoot"
  Expand-Archive -LiteralPath $Archive -DestinationPath $InstallRoot -Force
}

function Get-CarlaProcess {
  Get-Process -Name 'CarlaUE4', 'CarlaUE4-Win64-Shipping' -ErrorAction SilentlyContinue
}

function Write-FirewallEvent([string]$Action, [bool]$Success, [string]$ErrorMessage = '') {
  $event = [ordered]@{
    timestamp = (Get-Date).ToUniversalTime().ToString('o')
    action = $Action
    success = $Success
    error = $ErrorMessage
    ruleName = $FirewallRuleName
    direction = 'Inbound'
    protocol = 'TCP'
    localPorts = '2000-2002'
    remoteAddress = $script:ResolvedWslRemoteAddress
    program = $CarlaShippingExe
    revertCommand = "& '$PSCommandPath' -Command firewall-remove"
  }
  Add-Content -LiteralPath $FirewallLog -Value ($event | ConvertTo-Json -Compress)
}

function Resolve-WslRemoteAddress {
  if ($WslRemoteAddress) { return $WslRemoteAddress }
  $addresses = (& wsl.exe -d $WslDistro -- hostname -I 2>$null) -split '\s+'
  $ipv4 = $addresses | Where-Object { $_ -match '^\d{1,3}(\.\d{1,3}){3}$' } | Select-Object -First 1
  if (-not $ipv4) { throw "Could not determine an IPv4 address for WSL distro '$WslDistro'" }
  return "$ipv4/32"
}

Initialize-DDriveEnvironment
switch ($Command) {
  'setup' { Install-Carla }
  'start' {
    Install-Carla
    $running = Get-CarlaProcess
    if ($running) {
      $currentMode = if (Test-Path $ModeFile) { (Get-Content -LiteralPath $ModeFile -Raw).Trim() } else { 'unknown' }
      if ($Visible -and $currentMode -ne 'visible') {
        Write-Host "Restarting existing $currentMode CARLA instance in visible mode"
        $running | Sort-Object Id -Descending | ForEach-Object { Stop-Process -Id $_.Id -Force }
        Start-Sleep -Seconds 2
      } else {
        Write-Host "CARLA is already running (PID $($running.Id -join ', '), mode $currentMode)"
        break
      }
    }
    $carlaArguments = @(
      '-quality-level=Low', "-carla-rpc-port=$Port", '-windowed', '-ResX=1280', '-ResY=720',
      '-nosound', '-log', "-abslog=$(Join-Path $LogRoot 'CarlaUE4.log')"
    )
    if (-not $Visible) { $carlaArguments += '-RenderOffScreen' }
    $startArgs = @{
      FilePath = (Join-Path $InstallRoot 'CarlaUE4.exe')
      ArgumentList = $carlaArguments
      WorkingDirectory = $InstallRoot
      PassThru = $true
    }
    if (-not $Visible) { $startArgs.WindowStyle = 'Hidden' }
    $process = Start-Process @startArgs
    Set-Content -LiteralPath $ModeFile -Value $(if ($Visible) { 'visible' } else { 'headless' })
    Write-Host "CARLA started (PID $($process.Id), port $Port)"
  }
  'status' {
    $running = Get-CarlaProcess
    if ($running) { $running | Select-Object Id, ProcessName, Path }
    else { Write-Host 'CARLA is not running'; exit 1 }
  }
  'stop' {
    $running = Get-CarlaProcess
    if ($running) {
      $running | Sort-Object Id -Descending | ForEach-Object { Stop-Process -Id $_.Id -Force }
      Remove-Item -LiteralPath $ModeFile -Force -ErrorAction SilentlyContinue
      Write-Host 'CARLA stopped'
    } else { Write-Host 'CARLA is not running' }
  }
  'firewall-add' {
    if (-not (Test-Path $CarlaShippingExe)) { throw "CARLA executable not found: $CarlaShippingExe" }
    $script:ResolvedWslRemoteAddress = Resolve-WslRemoteAddress
    $existing = Get-NetFirewallRule -DisplayName $FirewallRuleName -ErrorAction SilentlyContinue
    if (-not $existing) {
      try {
        New-NetFirewallRule -DisplayName $FirewallRuleName -Direction Inbound -Action Allow `
          -Protocol TCP -LocalPort 2000-2002 -RemoteAddress $script:ResolvedWslRemoteAddress `
          -Program $CarlaShippingExe -ErrorAction Stop | Out-Null
      } catch {
        Write-FirewallEvent 'add' $false $_.Exception.Message
        throw
      }
      Write-FirewallEvent 'add' $true
      Write-Host "Firewall rule created; audit log: $FirewallLog"
    } else {
      Write-Host "Firewall rule already exists: $FirewallRuleName"
    }
  }
  'firewall-remove' {
    $script:ResolvedWslRemoteAddress = if ($WslRemoteAddress) { $WslRemoteAddress } else { 'not-applicable' }
    $existing = Get-NetFirewallRule -DisplayName $FirewallRuleName -ErrorAction SilentlyContinue
    if ($existing) {
      try {
        Remove-NetFirewallRule -DisplayName $FirewallRuleName -ErrorAction Stop
      } catch {
        Write-FirewallEvent 'remove' $false $_.Exception.Message
        throw
      }
      Write-FirewallEvent 'remove' $true
      Write-Host "Firewall rule removed; audit log: $FirewallLog"
    } else {
      Write-Host "Firewall rule does not exist: $FirewallRuleName"
    }
  }
}
