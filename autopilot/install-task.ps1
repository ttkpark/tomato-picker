# 자동운전을 윈도우에 등록한다 — 로그인 때 + 10분마다 감시견.
# 15일을 견디려면 재부팅·정전·크래시를 넘어야 한다. 그것을 OS에 맡긴다.
#
#   powershell -ExecutionPolicy Bypass -File autopilot\install-task.ps1
#   powershell -ExecutionPolicy Bypass -File autopilot\install-task.ps1 -Remove
param([switch]$Remove)

$ErrorActionPreference = 'Stop'
$name = 'TomatoPicker-Autopilot'
$ps   = (Get-Command powershell).Source
$wd   = Join-Path $PSScriptRoot 'watchdog.ps1'
$repo = Split-Path -Parent $PSScriptRoot

if ($Remove) {
  Unregister-ScheduledTask -TaskName $name -Confirm:$false
  Write-Host "제거함: $name"
  exit 0
}

$action = New-ScheduledTaskAction -Execute $ps `
  -Argument "-NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -File `"$wd`"" `
  -WorkingDirectory $repo

$t1 = New-ScheduledTaskTrigger -AtLogOn
$t2 = New-ScheduledTaskTrigger -Once -At (Get-Date).AddMinutes(2) `
        -RepetitionInterval (New-TimeSpan -Minutes 10) `
        -RepetitionDuration (New-TimeSpan -Days 30)

# 배터리로 돌아도, 유휴가 아니어도 계속한다. 감시견은 몇 초면 끝난다.
$set = New-ScheduledTaskSettingsSet -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries `
        -StartWhenAvailable -ExecutionTimeLimit (New-TimeSpan -Minutes 5) `
        -MultipleInstances IgnoreNew

Register-ScheduledTask -TaskName $name -Action $action -Trigger $t1, $t2 `
  -Settings $set -Description 'tomato-picker 자동운전 러너 감시견' -Force | Out-Null

Write-Host "등록함: $name (로그온 + 10분마다)"
Write-Host "끄기: autopilot\STOP 파일을 만들거나 -Remove 로 제거"
