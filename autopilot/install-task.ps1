# 자동운전을 윈도우에 등록한다 — 10분마다 감시견이 러너가 살아 있나 본다.
# 15일을 견디려면 재부팅·크래시·정전을 넘어야 한다. 그것을 OS에 맡긴다.
#
#   powershell -ExecutionPolicy Bypass -File autopilot\install-task.ps1
#   powershell -ExecutionPolicy Bypass -File autopilot\install-task.ps1 -Remove
#
# ⚠ `Register-ScheduledTask`는 이 PC에서 관리자 권한을 요구해 거절당한다(HRESULT 0x80070005).
#   `schtasks.exe`는 같은 일을 사용자 권한으로 해 준다 — 그래서 이쪽을 쓴다.
# ⚠ 이 파일은 **BOM 있는 UTF-8**이어야 한다. Windows PowerShell 5.1은 BOM 없는 .ps1을
#   cp949로 읽어 한글 주석이 깨지고 따옴표까지 삼켜 파서가 죽는다.
param([switch]$Remove)

$ErrorActionPreference = 'Stop'
$name = 'TomatoPicker-Autopilot'

if ($Remove) {
  schtasks /Delete /TN $name /F
  exit 0
}

$wd = Join-Path $PSScriptRoot 'watchdog.ps1'
$cmd = "powershell -NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -File `"$wd`""

schtasks /Create /TN $name /TR $cmd /SC MINUTE /MO 10 /F
if ($LASTEXITCODE -ne 0) { throw "작업 등록 실패 ($LASTEXITCODE)" }

Write-Host "등록함: $name (10분마다 감시견)"
Write-Host "확인: schtasks /Query /TN $name"
Write-Host "끄기: autopilot\STOP 파일을 만들거나 이 스크립트를 -Remove 로 실행"
