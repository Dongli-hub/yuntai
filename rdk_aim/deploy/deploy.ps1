<#
============================================================================
 一键把 rdk_aim 部署到地瓜派（在 Windows 这边跑）

 用法（在 PowerShell 里）：
     cd D:\STM32\STM32projects\yuntai2\rdk_aim\deploy
     .\deploy.ps1 -Ip 192.168.1.123

 可选参数：
     -User sunrise               地瓜派登录用户名（RDK 默认就是 sunrise）
     -Dest /home/sunrise/e_aim   板子上的目标目录
     -SkipSetup                  只拷文件，不跑板子上的环境检查脚本

 它做四件事：
   1) 试连板子（连不上就直接停，不会瞎折腾）
   2) 把整个 rdk_aim 目录拷过去（先拷成临时名，再改名成 e_aim）
   3) 在板子上跑 setup_on_rdk.sh（查依赖 / 串口权限 / 摄像头）
   4) 打印下一步该敲什么

 前提：你已经能用 ssh 登录这台板子（VSCode Remote-SSH 配过就说明可以）。
       如果是密码登录，运行时 ssh 会自己弹提示让你输密码。
#>
param(
    [Parameter(Mandatory = $true)][string]$Ip,
    [string]$User = "sunrise",
    [string]$Dest = "",
    [switch]$SkipSetup
)

$ErrorActionPreference = "Stop"
$src = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
if (-not $Dest) { $Dest = "/home/$User/e_aim" }
$remote = "$User@$Ip"
$homeDir = "/home/$User"

Write-Host "==============================================================" -ForegroundColor Cyan
Write-Host (" 部署 rdk_aim -> " + $remote + ":" + $Dest)
Write-Host (" 源目录: " + $src)
Write-Host "==============================================================" -ForegroundColor Cyan

# ---- 0. 本地先清干净（别把 out/ 和 __pycache__ 拷过去）----
Get-ChildItem -Path $src -Recurse -Directory -Filter "__pycache__" -ErrorAction SilentlyContinue |
    Remove-Item -Recurse -Force -ErrorAction SilentlyContinue
$outDir = Join-Path $src "out"
if (Test-Path $outDir) {
    Get-ChildItem -LiteralPath $outDir -File -ErrorAction SilentlyContinue |
        Remove-Item -Force -ErrorAction SilentlyContinue
}
Get-ChildItem -Path (Join-Path $src "tools") -Recurse -Directory -Filter "build" -ErrorAction SilentlyContinue |
    Remove-Item -Recurse -Force -ErrorAction SilentlyContinue

# ---- 1. 试连 ----
Write-Host ""
Write-Host "[1/4] 试连板子..." -ForegroundColor Yellow
& ssh -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new $remote "echo CONNECT_OK; uname -a"
if ($LASTEXITCODE -ne 0) {
    Write-Host ("连不上 " + $remote) -ForegroundColor Red
    Write-Host "请确认：① 板子和这台电脑在同一网段  ② IP 对不对  ③ 板子的 ssh 服务开着"
    exit 1
}

# ---- 2. 拷贝 ----
Write-Host ""
Write-Host "[2/4] 拷贝文件..." -ForegroundColor Yellow
& ssh $remote ("rm -rf " + $homeDir + "/rdk_aim_tmp")
& scp -r -q $src ($remote + ":" + $homeDir + "/rdk_aim_tmp")
if ($LASTEXITCODE -ne 0) { Write-Host "拷贝失败" -ForegroundColor Red; exit 1 }
# 用 mv 改名，避免 scp 在目标已存在时多套一层目录
& ssh $remote ("rm -rf " + $Dest + " && mv " + $homeDir + "/rdk_aim_tmp " + $Dest + " && ls " + $Dest)
if ($LASTEXITCODE -ne 0) { Write-Host "改名失败" -ForegroundColor Red; exit 1 }

# ---- 3. 板子上的环境检查 ----
if (-not $SkipSetup) {
    Write-Host ""
    Write-Host "[3/4] 在板子上跑环境检查..." -ForegroundColor Yellow
    & ssh $remote ("bash " + $Dest + "/deploy/setup_on_rdk.sh " + $Dest)
}
else {
    Write-Host ""
    Write-Host "[3/4] 已跳过环境检查（-SkipSetup）" -ForegroundColor DarkGray
}

# ---- 4. 下一步 ----
Write-Host ""
Write-Host "[4/4] 完成。下一步：" -ForegroundColor Green
Write-Host "  ssh $remote"
Write-Host "  cd $Dest"
Write-Host "  python3 main.py ports                    # 找串口设备名"
Write-Host "  python3 main.py ping --port /dev/ttyS1   # 确认 H723 在通信"
Write-Host "  python3 tools/check_laser.py --port /dev/ttyS1 --source 0 --save"
Write-Host "  python3 tools/calib_boresight.py --port-gimbal /dev/ttyS1 --source 0"
Write-Host "  python3 main.py run --duration 20 --budget 4"
Write-Host ("  详细排查见 " + $Dest + "/docs/02_标定与现场调试.md")

