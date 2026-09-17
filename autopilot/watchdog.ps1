# 자동운전 감시견 — 러너가 죽었으면 되살린다.
# 작업 스케줄러가 10분마다 부른다(install-task.ps1). 살아 있으면 아무것도 안 한다.
$ErrorActionPreference = 'SilentlyContinue'

$repo = Split-Path -Parent $PSScriptRoot
$hb   = Join-Path $PSScriptRoot 'state\heartbeat.json'
$log  = Join-Path $PSScriptRoot 'logs\watchdog.log'
New-Item -ItemType Directory -Force (Join-Path $PSScriptRoot 'logs') | Out-Null

function Say($m) { "$(Get-Date -f 'MM-dd HH:mm:ss') $m" | Out-File -Append -Encoding utf8 $log }

if (Test-Path (Join-Path $PSScriptRoot 'STOP')) { Say 'STOP 있음 — 되살리지 않음'; exit 0 }

# 심장박동이 40분 넘게 안 뛰면 죽었거나 멈춘 것으로 본다.
# (한 사이클 상한이 45분이라, 그 안에 러너가 반드시 한 번은 찍는다)
$dead = $true
if (Test-Path $hb) {
  $h = Get-Content $hb -Raw | ConvertFrom-Json
  $age = (Get-Date) - [datetime]$h.time
  if ($age.TotalMinutes -lt 40) { $dead = $false }
  # 프로세스가 실제로 살아 있으면 그것도 인정한다(긴 사이클 중일 수 있다)
  if ($dead -and $h.pid) {
    if (Get-Process -Id $h.pid) { $dead = $false; Say "pid $($h.pid) 살아 있음(박동 $([int]$age.TotalMinutes)분 전)" }
  }
}

if (-not $dead) { exit 0 }

Say '러너가 죽었다 — 되살린다'
$py = (Get-Command python).Source
Start-Process -FilePath $py `
  -ArgumentList @((Join-Path $PSScriptRoot 'runner.py'), 'run') `
  -WorkingDirectory $repo -WindowStyle Hidden
Say '되살림'
