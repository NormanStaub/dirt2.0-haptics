# Starts the DualSense bridge together with DiRT Rally 2.0 and stops it when the game closes.
#
#   Desktop shortcut:      powershell -WindowStyle Hidden -ExecutionPolicy Bypass -File launch.ps1
#   Steam launch options:  powershell -WindowStyle Hidden -ExecutionPolicy Bypass -File "<path>\launch.ps1" %command%
#
# With %command% Steam passes the game's exe (and its arguments) to this script.

$here   = Split-Path -Parent $MyInvocation.MyCommand.Path
$bridge = Join-Path $here 'dirt_dualsense.py'

function Get-Bridge {
    Get-CimInstance Win32_Process -Filter "Name='python.exe'" |
        Where-Object { $_.CommandLine -match 'dirt_dualsense\.py' }
}

$startedBridge = $false
if (-not (Get-Bridge)) {
    Start-Process -FilePath 'python' -ArgumentList "`"$bridge`"" -WorkingDirectory $here -WindowStyle Minimized
    $startedBridge = $true
    Start-Sleep -Seconds 2
}

if ($args.Count -gt 0) {
    # Launched by Steam via %command%: run the game ourselves and wait for it.
    $exe  = $args[0]
    $rest = @($args | Select-Object -Skip 1)
    if ($rest.Count -gt 0) {
        Start-Process -FilePath $exe -ArgumentList $rest -WorkingDirectory (Split-Path $exe) -Wait
    } else {
        Start-Process -FilePath $exe -WorkingDirectory (Split-Path $exe) -Wait
    }
} else {
    # Launched from the desktop shortcut: ask Steam to start the game, then wait for it.
    Start-Process 'steam://rungameid/690790'
    $deadline = (Get-Date).AddMinutes(3)
    while (-not (Get-Process dirtrally2 -ErrorAction SilentlyContinue) -and (Get-Date) -lt $deadline) {
        Start-Sleep -Seconds 2
    }
    while (Get-Process dirtrally2 -ErrorAction SilentlyContinue) {
        Start-Sleep -Seconds 3
    }
}

# Game closed. The bridge has already released the triggers (no telemetry for 1 s),
# so it is safe to stop it.
if ($startedBridge) {
    Start-Sleep -Seconds 2
    Get-Bridge | ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
}
