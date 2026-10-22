# 真机链路三步体检：ping -> 41100 端口 -> RSC 只读探测
#
# 用法（在仓库根执行）：
#     .\scripts\check_vplc_link.ps1
#     .\scripts\check_vplc_link.ps1 -Host 192.168.3.99 -Python C:\work\AIC_cv-dev\.venv\Scripts\python.exe
#
# 为什么单独一个脚本：现场最常见的三种「连不上」表现一模一样（超时），
# 但原因分三层 —— 网线/网段、端口/防火墙、变量名/密码。逐层过一遍比看日志快。
#
# 注意：本脚本只做**只读**探测，不写 PLC（不会动任何变量）。

param(
    [string]$TargetHost = "192.168.3.99",
    [int]$Port = 41100,
    [string]$Python = "",
    [string]$Map = ""
)

$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot

if ([string]::IsNullOrWhiteSpace($Map)) {
    $Map = Join-Path $repo "src\plugins\aic_plc_gateway\config\cam_official.yaml"
}
if ([string]::IsNullOrWhiteSpace($Python)) {
    $venvPy = Join-Path $repo ".venv\Scripts\python.exe"
    if (Test-Path $venvPy) {
        $Python = $venvPy
    } else {
        $Python = "python"
    }
}

$fail = 0
Write-Host "=== AIC vPLC 链路体检 ===" -ForegroundColor Cyan
Write-Host ("目标 {0}:{1}" -f $TargetHost, $Port)
Write-Host ("Python {0}" -f $Python)
Write-Host ("标签表 {0}" -f $Map)
Write-Host ""

# ---- 1. 网络可达 ----
Write-Host "[1/3] ping ..." -NoNewline
try {
    $ok = Test-Connection -ComputerName $TargetHost -Count 2 -Quiet -ErrorAction Stop
} catch {
    $ok = $false
}
if ($ok) {
    Write-Host " OK" -ForegroundColor Green
} else {
    Write-Host " 失败" -ForegroundColor Red
    Write-Host "  -> 检查网线是否接 ETH2、本机网卡是否 192.168.3.100/24、网关是否留空"
    $fail = 1
}

# ---- 2. RSC 端口 ----
Write-Host "[2/3] TCP $Port ..." -NoNewline
try {
    $tcp = Test-NetConnection -ComputerName $TargetHost -Port $Port -InformationLevel Quiet -WarningAction SilentlyContinue
} catch {
    $tcp = $false
}
if ($tcp) {
    Write-Host " OK" -ForegroundColor Green
} else {
    Write-Host " 不通" -ForegroundColor Red
    Write-Host "  -> 端口被防火墙挡 / vPLC 没起来；容器内要用桥接 IP 10.88.0.2 而不是 192.168.3.99"
    $fail = 1
}

# ---- 3. RSC 只读探测（8 个官方 Cam 变量是否存在）----
Write-Host "[3/3] RSC 只读探测 ..." -ForegroundColor Cyan
if (-not (Test-Path $Map)) {
    Write-Host ("  找不到标签表：{0}" -f $Map) -ForegroundColor Red
    $fail = 1
} else {
    $script = Join-Path $repo "src\vision\aic_vision_trad\scripts\run_to_plc.py"
    $env:PLCNEXT_HOST = $TargetHost
    $env:PLCNEXT_PORT = "$Port"
    & $Python $script --map $Map --probe
    if ($LASTEXITCODE -ne 0) {
        Write-Host "  -> 变量缺失/异常：按输出里的三条原因排查（POU 名 / 实例名 / 字段类型）" -ForegroundColor Yellow
        $fail = 1
    }
}

Write-Host ""
if ($fail -eq 0) {
    Write-Host "体检通过：网络、端口、变量名三层都通。可以跑 --once 单次拍照。" -ForegroundColor Green
} else {
    Write-Host "体检未通过：先修上面标红的那一层，再往下走。" -ForegroundColor Red
}
exit $fail
