[CmdletBinding()]
param(
    [string]$Port = 'COM4',
    [ValidateRange(1200, 3000000)]
    [int]$BaudRate = 115200
)

$ErrorActionPreference = 'Stop'
$logDirectory = Join-Path $PSScriptRoot 'logs'
$timestamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$safePort = $Port -replace '[^A-Za-z0-9_-]', '_'
$logPath = Join-Path $logDirectory "$timestamp-s3-line-$safePort.log"
$utf8NoBom = [System.Text.UTF8Encoding]::new($false)
$serial = [System.IO.Ports.SerialPort]::new(
    $Port,
    $BaudRate,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One
)

New-Item -ItemType Directory -Path $logDirectory -Force | Out-Null
$serial.NewLine = "`n"
$serial.ReadTimeout = 500

try {
    $serial.Open()
    $heading = '# Serial capture started {0:o}; port={1}; baud={2}' -f `
        (Get-Date), $Port, $BaudRate
    [System.IO.File]::AppendAllText(
        $logPath,
        $heading + [Environment]::NewLine,
        $utf8NoBom
    )
    Write-Host "Saving all serial output to $logPath"
    Write-Host 'Press Ctrl+C to stop logging.'

    while ($true) {
        try {
            $line = $serial.ReadLine().TrimEnd("`r")
            $entry = '{0:yyyy-MM-dd HH:mm:ss.fff} {1}' -f (Get-Date), $line
            Write-Host $entry
            [System.IO.File]::AppendAllText(
                $logPath,
                $entry + [Environment]::NewLine,
                $utf8NoBom
            )
        }
        catch [System.TimeoutException] {
            # Keep waiting so Ctrl+C can stop an otherwise quiet capture.
        }
    }
}
finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
    Write-Host "Serial log closed: $logPath"
}
