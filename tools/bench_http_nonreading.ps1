# One bounded bench request; JP runs this on a PC connected to the iPhone hotspot.
# Reads HTTP headers only, then deliberately does not read the response body for 15 s.
param(
    [string]$DeviceIp = '172.20.10.2',
    [ValidateRange(1, 4294967295)][long]$Archive = 21,
    [switch]$Current
)
$ErrorActionPreference = 'Stop'
$client = [Net.Sockets.TcpClient]::new()
try {
    $client.ReceiveBufferSize = 1024
    $connect = $client.ConnectAsync($DeviceIp, 80)
    if (-not $connect.Wait(5000)) { throw 'Connection timed out; check hotspot and ACTIVE mode.' }
    [void]$connect.GetAwaiter().GetResult()
    $stream = $client.GetStream()
    $stream.ReadTimeout = 10000
    $stream.WriteTimeout = 5000
    $route = if ($Current) { '/f/current' } else { '/f/{0:D8}' -f $Archive }
    $request = [Text.Encoding]::ASCII.GetBytes("GET $route HTTP/1.1`r`nHost: $DeviceIp`r`nConnection: close`r`n`r`n")
    $stream.Write($request, 0, $request.Length)
    $timer = [Diagnostics.Stopwatch]::StartNew()
    $header = [Text.StringBuilder]::new()
    while ($header.Length -lt 8192) {
        if ($timer.ElapsedMilliseconds -gt 15000) { throw 'Header deadline exceeded.' }
        $one = $stream.ReadByte()
        if ($one -lt 0) { throw 'Connection closed before complete headers.' }
        [void]$header.Append([char]$one)
        if ($header.ToString().EndsWith("`r`n`r`n")) { break }
    }
    $head = $header.ToString()
    if (-not $head.EndsWith("`r`n`r`n")) { throw 'Header length limit exceeded.' }
    Write-Output $head.TrimEnd()
    if ($head -notmatch '^HTTP/1\.[01] 200 ') { throw 'Expected HTTP 200; no non-reading test performed.' }
    if ($head -notmatch '(?im)^Content-Length:\s*(\d+)\s*$') { throw 'Missing Content-Length.' }
    $expected = [long]$Matches[1]
    Write-Output ("{0:HH:mm:ss.fff} HEADERS RECEIVED; receive_buffer={1}; expected={2}; NO BODY READS for 15 seconds" -f (Get-Date), $client.ReceiveBufferSize, $expected)
    Start-Sleep -Seconds 15
    Write-Output ("{0:HH:mm:ss.fff} DRAIN START (does not change the earlier no-read interval)" -f (Get-Date))
    $stream.ReadTimeout = 2000
    $buffer = New-Object byte[] 4096
    $received = 0L
    $outcome = 'drain_deadline'
    $drain = [Diagnostics.Stopwatch]::StartNew()
    while ($drain.ElapsedMilliseconds -lt 10000 -and $received -lt 3145728) {
        try { $count = $stream.Read($buffer, 0, $buffer.Length) }
        catch [IO.IOException] { $outcome = 'read_error_or_timeout: ' + $_.Exception.Message; break }
        if ($count -eq 0) { $outcome = 'peer_closed'; break }
        $received += $count
    }
    Write-Output ("BODY_RECEIVED={0} EXPECTED={1} COMPLETE={2} OUTCOME={3}" -f $received, $expected, ($received -eq $expected), $outcome)
    Write-Output 'Use the device HTTP_GET_END/CLOSE records for the stall decision and cleanup timing. No output file was saved.'
} catch {
    Write-Output ('BENCH_ERROR: ' + $_.Exception.Message)
} finally {
    $client.Dispose()
}
