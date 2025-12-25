param(
  [Parameter(Mandatory = $true)]
  [string]$UserDataDir,
  [Parameter(Mandatory = $true)]
  [int]$Pid,
  [Parameter(Mandatory = $true)]
  [string]$ExePath
)

try {
  Wait-Process -Id $Pid -ErrorAction SilentlyContinue
} catch {
}

for ($i = 0; $i -lt 20; $i++) {
  $active = $false
  try {
    $procs = Get-Process -ErrorAction SilentlyContinue | Where-Object { $_.Path -eq $ExePath }
    if ($procs) {
      $active = $true
    }
  } catch {
  }
  if (-not $active) {
    break
  }
  Start-Sleep -Milliseconds 250
}

for ($i = 0; $i -lt 10; $i++) {
  try {
    if (Test-Path -LiteralPath $UserDataDir) {
      Remove-Item -LiteralPath $UserDataDir -Recurse -Force -ErrorAction Stop
    }
    break
  } catch {
    Start-Sleep -Milliseconds 500
  }
}

if (Test-Path -LiteralPath $UserDataDir) {
  try {
    cmd.exe /c "rmdir /s /q `"$UserDataDir`""
  } catch {
  }
}
