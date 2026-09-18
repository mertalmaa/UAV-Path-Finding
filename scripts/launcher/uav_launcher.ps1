# UAV Pathfinder -- menulu baslatici
#
# Normalde proje kokundeki UAV_Pathfinder.bat uzerinden cift tiklanarak calisir.
# Dogrudan da calistirilabilir:
#   powershell -NoProfile -ExecutionPolicy Bypass -File scripts\launcher\uav_launcher.ps1
#
# Secenekler:
#   -Region <id>   Mission UI acilisinda onyuklenecek bolge (varsayilan: bilecik)
#   -Port <n>      Mission UI portu (varsayilan: 8765; arazi karolari Port+1)
#   -NoBrowser     sunucu hazir olunca tarayiciyi acma

[CmdletBinding()]
param(
  [string]$Region = "bilecik",
  [int]$Port = 8765,
  [switch]$NoBrowser
)

$ErrorActionPreference = "Stop"
$script:Root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$script:LastExit = 0

# --- yardimcilar -----------------------------------------------------------

function Write-Line([string]$Text, [string]$Color = "Gray") {
  Write-Host $Text -ForegroundColor $Color
}

function Write-Title([string]$Text) {
  Write-Host ""
  Write-Host ("-" * 62) -ForegroundColor DarkGray
  Write-Host "  $Text" -ForegroundColor Cyan
  Write-Host ("-" * 62) -ForegroundColor DarkGray
}

function Wait-Menu {
  Write-Host ""
  Write-Host "Menuye donmek icin Enter..." -ForegroundColor DarkGray
  [void](Read-Host)
}

function Assert-ProjectRoot {
  if (-not (Test-Path (Join-Path $script:Root "planner\pose_search.py"))) {
    Write-Line "HATA: proje koku bulunamadi ($script:Root)." "Red"
    Write-Line "Bu betik <proje koku>\scripts\launcher\ altinda durmalidir." "Red"
    Wait-Menu
    exit 1
  }
}

# Kullanilacak Python: once .venv, sonra 'py -3', sonra 'python'.
function Get-Python {
  $venv = Join-Path $script:Root ".venv\Scripts\python.exe"
  if (Test-Path $venv) { return @{ Exe = $venv; Pre = @(); Label = ".venv" } }
  if (Get-Command py -ErrorAction SilentlyContinue) { return @{ Exe = "py"; Pre = @("-3"); Label = "py -3" } }
  if (Get-Command python -ErrorAction SilentlyContinue) { return @{ Exe = "python"; Pre = @(); Label = "python" } }
  return $null
}

function Show-NoPython {
  Write-Line "Python bulunamadi." "Red"
  Write-Line "python.org/downloads adresinden Python 3.11 kurun ve kurulumda" "Red"
  Write-Line "'Add python.exe to PATH' secenegini isaretleyin." "Red"
}

# Proje kokunde python calistirir. Cikis kodu $script:LastExit icine yazilir.
# Alt surecin ciktisi dogrudan konsola akar, yakalanmaz.
function Invoke-Py {
  param([string[]]$Arguments)
  $script:LastExit = 1
  $py = Get-Python
  if ($null -eq $py) { Show-NoPython; return }
  $all = @($py.Pre + $Arguments)
  Write-Line ">> $($py.Exe) $($all -join ' ')" "DarkGray"
  Push-Location $script:Root
  try {
    & $py.Exe @all
    $script:LastExit = $LASTEXITCODE
  } finally {
    Pop-Location
  }
  if ($script:LastExit -ne 0) { Write-Line "Cikis kodu: $script:LastExit" "Yellow" }
}

function Test-PortOpen {
  param([string]$TargetHost = "127.0.0.1", [int]$TargetPort)
  $client = New-Object System.Net.Sockets.TcpClient
  try {
    $client.Connect($TargetHost, $TargetPort)
    return $true
  } catch {
    return $false
  } finally {
    $client.Dispose()
  }
}

function Wait-PortOpen {
  param([string]$TargetHost = "127.0.0.1", [int]$TargetPort, [int]$TimeoutSec = 90)
  $sw = [System.Diagnostics.Stopwatch]::StartNew()
  while ($sw.Elapsed.TotalSeconds -lt $TimeoutSec) {
    if (Test-PortOpen -TargetHost $TargetHost -TargetPort $TargetPort) { return $true }
    Start-Sleep -Milliseconds 500
  }
  return $false
}

# Planlayici ayri bir alt surec (spawn) olusturur; agac halinde kapatilmali.
function Stop-Tree([int]$ProcessId) {
  if ($ProcessId -le 0) { return }
  & taskkill.exe /PID $ProcessId /T /F 2>&1 | Out-Null
}

# --- menu eylemleri --------------------------------------------------------

function Invoke-Setup {
  Write-Title "Kurulum: sanal ortam ve bagimliliklar"
  $venvPython = Join-Path $script:Root ".venv\Scripts\python.exe"
  if (Test-Path $venvPython) {
    Write-Line ".venv zaten var; bagimliliklar guncelleniyor." "DarkGray"
  } else {
    $py = Get-Python
    if ($null -eq $py) { Show-NoPython; Wait-Menu; return }
    Write-Line "Sanal ortam olusturuluyor (.venv)..." "Gray"
    Push-Location $script:Root
    try { & $py.Exe @($py.Pre + @("-m", "venv", ".venv")) } finally { Pop-Location }
    if (-not (Test-Path $venvPython)) {
      Write-Line "Sanal ortam olusturulamadi." "Red"
      Wait-Menu
      return
    }
  }
  $code = 1
  Push-Location $script:Root
  try {
    & $venvPython -m pip install --upgrade pip
    & $venvPython -m pip install -r requirements.txt
    $code = $LASTEXITCODE
  } finally { Pop-Location }
  Write-Host ""
  if ($code -eq 0) { Write-Line "Kurulum tamam. Bundan sonra .venv kullanilacak." "Green" }
  else { Write-Line "pip hata verdi (cikis kodu $code)." "Red" }
  Wait-Menu
}

function Invoke-MissionUI {
  Write-Title "Mission UI"
  $py = Get-Python
  if ($null -eq $py) { Show-NoPython; Wait-Menu; return }

  $uiPort = $Port
  while (Test-PortOpen -TargetPort $uiPort) {
    Write-Line "Port $uiPort dolu; $($uiPort + 2) deneniyor." "Yellow"
    $uiPort += 2
    if ($uiPort -gt $Port + 20) { Write-Line "Bos port bulunamadi." "Red"; Wait-Menu; return }
  }

  $url = "http://127.0.0.1:$uiPort"
  $argList = @($py.Pre + @("-m", "mission_ui.server", "--port", "$uiPort",
                           "--tile-port", "$($uiPort + 1)", "--warm", $Region))

  Write-Line "Sunucu baslatiliyor: $url   (bolge: $Region, karo portu: $($uiPort + 1))" "Gray"
  Write-Line "Ilk acilista bolge DEM'i onbellege alinir; birkac saniye surebilir." "DarkGray"

  $proc = Start-Process -FilePath $py.Exe -ArgumentList $argList -WorkingDirectory $script:Root -PassThru
  try {
    if (Wait-PortOpen -TargetPort $uiPort -TimeoutSec 90) {
      Write-Line "Hazir: $url" "Green"
      if (-not $NoBrowser) { Start-Process $url | Out-Null }
    } else {
      Write-Line "Sunucu 90 saniyede yanit vermedi; acilan pencerede hata olabilir." "Yellow"
    }
    Write-Host ""
    Write-Line "Sunucu ayri bir pencerede calisiyor." "DarkGray"
    Write-Line "Durdurmak icin bu pencerede Enter'a basin." "Yellow"
    [void](Read-Host)
  } finally {
    if ($proc -and -not $proc.HasExited) {
      Write-Line "Sunucu kapatiliyor (PID $($proc.Id))..." "DarkGray"
      Stop-Tree $proc.Id
    }
  }
}

function Invoke-SingleShot {
  Write-Title "Bilecik 31 km kanyon gorevi (tek atis)"
  Write-Line "Arama ~3 s; cikti: results\test_bilecik\ (JSON + PNG)" "DarkGray"
  Invoke-Py @("-B", "scripts/run_single_shot_31km.py")
  Wait-Menu
}

function Invoke-NinetyKm {
  Write-Title "Bilecik 5 x ~90 km gorev"
  Write-Line "Gorev basina ~10-15 s. Tek gorev icin kimlik girin: M90_01 ... M90_05" "DarkGray"
  $pick = (Read-Host "Gorev kimligi (bos = 5 gorevin hepsi)").Trim()
  $argv = @("-B", "scripts/run_bilecik_90km_5_missions.py")
  if ($pick) { $argv += @("--mission", $pick) }
  Invoke-Py $argv
  Wait-Menu
}

function Invoke-Tests {
  Write-Title "Testler"
  Write-Line "planner testleri (tests/)" "Gray"
  Invoke-Py @("-m", "pytest", "-q", "tests")
  Write-Host ""
  Write-Line "Mission UI sozlesme testleri (mission_ui/tests/)" "Gray"
  Invoke-Py @("-m", "pytest", "-q", "mission_ui/tests")
  Wait-Menu
}

function Invoke-SystemCheck {
  Write-Title "Sistem kontrolu"
  Invoke-Py @("scripts/launcher/system_check.py")
  Wait-Menu
}

function Open-Results {
  Write-Title "Sonuc klasoru"
  $results = Join-Path $script:Root "results\test_bilecik"
  if (-not (Test-Path $results)) { $results = $script:Root }
  Start-Process explorer.exe $results | Out-Null
  Write-Line "Gezginde acildi: $results" "Green"
  Start-Sleep -Seconds 1
}

# --- ana dongu -------------------------------------------------------------

function Show-Menu {
  Clear-Host
  $py = Get-Python
  $pyLabel = if ($py) { $py.Label } else { "bulunamadi -- once 5) Kurulum" }
  Write-Host ""
  Write-Host "  ============================================================" -ForegroundColor DarkCyan
  Write-Host "    UAV PATHFINDER" -ForegroundColor White
  Write-Host "    Sabit kanatli IHA -- araziye duyarli 3B yol planlama" -ForegroundColor Gray
  Write-Host "  ============================================================" -ForegroundColor DarkCyan
  Write-Host "    Proje : $script:Root" -ForegroundColor DarkGray
  Write-Host "    Python: $pyLabel" -ForegroundColor DarkGray
  Write-Host ""
  Write-Host "    1) Mission UI (harita arayuzu) baslat" -ForegroundColor White
  Write-Host "    2) Ornek gorev: Bilecik 31 km kanyon" -ForegroundColor White
  Write-Host "    3) Ornek gorev: 5 x ~90 km" -ForegroundColor White
  Write-Host "    4) Testleri calistir" -ForegroundColor White
  Write-Host "    5) Kurulum (.venv + requirements.txt)" -ForegroundColor White
  Write-Host "    6) Sistem kontrolu" -ForegroundColor White
  Write-Host "    7) Sonuc klasorunu ac" -ForegroundColor White
  Write-Host "    0) Cikis" -ForegroundColor White
  Write-Host ""
}

Assert-ProjectRoot
try { [Console]::OutputEncoding = [System.Text.Encoding]::UTF8 } catch { }

$running = $true
while ($running) {
  Show-Menu
  $choice = (Read-Host "  Secim").Trim().ToLowerInvariant()
  switch ($choice) {
    "1" { Invoke-MissionUI }
    "2" { Invoke-SingleShot }
    "3" { Invoke-NinetyKm }
    "4" { Invoke-Tests }
    "5" { Invoke-Setup }
    "6" { Invoke-SystemCheck }
    "7" { Open-Results }
    "0" { $running = $false }
    "q" { $running = $false }
    default { }
  }
}

Write-Host ""
Write-Line "Gorusuruz." "DarkGray"
