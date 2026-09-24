# JP runs on a PC connected to the iPhone hotspot. No other requests or panel touches.
# Revision 2: 16 KiB receive window, drain up to 8 KiB every 2 s (~4 KiB/s).
# Avoid the tiny-window pacing that triggered the 5 s stall guard in revision 1.
# At 300 s, <= 1.23 MB read plus bounded socket buffering is below the 2 MiB archive.
param([string]$DeviceIp = '172.20.10.2', [ValidateRange(1,4294967295)][long]$Archive = 21)
$ErrorActionPreference = 'Stop'
$client = [Net.Sockets.TcpClient]::new()
try {
    $client.ReceiveBufferSize = 16384
    $connect = $client.ConnectAsync($DeviceIp,80)
    if (-not $connect.Wait(5000)) { throw 'Connection timeout; check hotspot and ACTIVE mode.' }
    [void]$connect.GetAwaiter().GetResult()
    $stream = $client.GetStream()
    $stream.ReadTimeout = 7000
    $stream.WriteTimeout = 5000
    $route = '/f/{0:D8}' -f $Archive
    $request = [Text.Encoding]::ASCII.GetBytes("GET $route HTTP/1.1`r`nHost: $DeviceIp`r`nConnection: close`r`n`r`n")
    $stream.Write($request,0,$request.Length)
    $timer = [Diagnostics.Stopwatch]::StartNew()
    $header = [Text.StringBuilder]::new()
    while ($header.Length -lt 8192) {
        if ($timer.ElapsedMilliseconds -gt 15000) { throw 'Header deadline exceeded.' }
        $one = $stream.ReadByte()
        if ($one -lt 0) { throw 'Closed before complete headers.' }
        [void]$header.Append([char]$one)
        if ($header.ToString().EndsWith("`r`n`r`n")) { break }
    }
    $head = $header.ToString()
    if (-not $head.EndsWith("`r`n`r`n")) { throw 'Header limit exceeded.' }
    Write-Output $head.TrimEnd()
    if ($head -notmatch '^HTTP/1\.[01] 200 ') { throw 'Expected HTTP 200.' }
    if ($head -notmatch '(?im)^Content-Length:\s*(\d+)\s*$') { throw 'Missing Content-Length.' }
    $expected = [long]$Matches[1]
    if ($expected -lt 1800000) { throw 'Use a large archive (at least 1800000 bytes) for the five-minute case.' }
    Write-Output ("{0:HH:mm:ss.fff} SLOW READ START v2; expected={1}; receive_buffer={2}; block=8192 period_ms=2000; no panel touches or Safari requests" -f (Get-Date),$expected,$client.ReceiveBufferSize)
    $buffer = New-Object byte[] 8192
    $received = 0L
    $nextReport = 30000L
    $outcome = 'client_360s_limit'
    while ($timer.ElapsedMilliseconds -lt 360000) {
        Start-Sleep -Milliseconds 2000
        try { $count = $stream.Read($buffer,0,$buffer.Length) }
        catch [IO.IOException] { $outcome = 'read_error_or_timeout: ' + $_.Exception.Message; break }
        if ($count -eq 0) { $outcome = 'peer_closed'; break }
        $received += $count
        if ($received -ge $expected) { $outcome = 'complete'; break }
        if ($timer.ElapsedMilliseconds -ge $nextReport) {
            Write-Output ("elapsed_ms={0} body_received={1}" -f $timer.ElapsedMilliseconds,$received)
            $nextReport = $timer.ElapsedMilliseconds + 30000
        }
    }
    Write-Output ("elapsed_ms={0} BODY_RECEIVED={1} EXPECTED={2} OUTCOME={3}" -f $timer.ElapsedMilliseconds,$received,$expected,$outcome)
    Write-Output 'Use device records to distinguish idle expiry from a stall or connection error. No file saved.'
} catch { Write-Output ('BENCH_ERROR: ' + $_.Exception.Message) }
finally { $client.Dispose() }
