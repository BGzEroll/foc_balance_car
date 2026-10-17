param([string]$MatlabExecutable)
$ErrorActionPreference = 'Stop'

# 桌面已有MATLAB时不再启动第二个实例。launch_balance_panel.m也会复用已有面板。
$desktopSessions = @(Get-CimInstance Win32_Process -Filter "Name = 'MATLAB.exe'" |
    Where-Object { $_.CommandLine -match '(?i)-desktop' })
if ($desktopSessions.Count -gt 0) {
    $ids = ($desktopSessions.ProcessId -join ', ')
    Write-Output "已有 MATLAB 桌面实例（PID: $ids），不再新开窗口。"
    Write-Output "请使用现有的工况面板；面板关闭时，在现有 MATLAB 中运行 launch_balance_panel.m。"
    return
}

if (-not $MatlabExecutable) {
    $matlabCommand = Get-Command matlab.exe -ErrorAction SilentlyContinue
    if ($matlabCommand) { $MatlabExecutable = $matlabCommand.Source }
}
if (-not $MatlabExecutable) {
    $registryRoot = 'HKLM:\SOFTWARE\MathWorks\MATLAB'
    if (Test-Path -LiteralPath $registryRoot) {
        foreach ($versionKey in (Get-ChildItem -LiteralPath $registryRoot | Sort-Object PSChildName -Descending)) {
            $installRoot = (Get-ItemProperty -LiteralPath $versionKey.PSPath -ErrorAction SilentlyContinue).MATLABROOT
            if ($installRoot) {
                $candidate = Join-Path $installRoot 'bin\matlab.exe'
                if (Test-Path -LiteralPath $candidate) { $MatlabExecutable = $candidate; break }
            }
        }
    }
}
if (-not $MatlabExecutable -or -not (Test-Path -LiteralPath $MatlabExecutable)) {
    throw '找不到MATLAB。请传入 -MatlabExecutable，或在已打开的MATLAB中运行 launch_balance_panel.m。'
}
$entry = (Join-Path $PSScriptRoot 'launch_balance_panel.m').Replace("'", "''")
$arguments = '-desktop -sd "{0}" -r "run(''{1}'');"' -f $PSScriptRoot, $entry
Start-Process -FilePath $MatlabExecutable -ArgumentList $arguments -WindowStyle Normal
