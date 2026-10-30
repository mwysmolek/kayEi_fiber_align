# Run in an ADMIN PowerShell:  powershell -ExecutionPolicy Bypass -File C:\dev\hydra\find_hydra.ps1
# 1) Captures packets on Ethernet 2 while you power-cycle the Hydra (it usually ARPs its own IP at boot).
# 2) Temporarily adds addresses on common factory subnets to Ethernet 2 and scans each for TCP port 400.
$nic = "Ethernet 2"
$etl = "C:\dev\hydra\cap.etl"; $txt = "C:\dev\hydra\cap.txt"; $out = "C:\dev\hydra\find_result.txt"
Start-Transcript -Path $out -Force | Out-Null

pktmon stop 2>$null | Out-Null
pktmon filter remove | Out-Null
pktmon start --capture --comp nics --pkt-size 128 -f $etl | Out-Null
Write-Host "Capturing. Power-cycle the Hydra NOW, wait ~30 s after it boots, then press Enter." -ForegroundColor Yellow
Read-Host | Out-Null
pktmon stop | Out-Null
pktmon etl2txt $etl -o $txt | Out-Null
Write-Host "`n--- ARP / IP traffic not from the home LAN ---"
Select-String -Path $txt -Pattern "ARP|IP " | ForEach-Object { $_.Line } |
    Where-Object { $_ -notmatch "192\.168\.178\." } | Select-Object -First 40

Write-Host "`n--- Scanning common subnets for port 400 ---"
$subnets = "192.168.0","192.168.1","192.168.2","192.168.10","192.168.100","10.0.0","10.0.1","172.16.0"
foreach ($sn in $subnets) {
    $me = "$sn.250"
    New-NetIPAddress -InterfaceAlias $nic -IPAddress $me -PrefixLength 24 -SkipAsSource $true -ErrorAction SilentlyContinue | Out-Null
    Start-Sleep 2
    $tasks = 1..254 | Where-Object { $_ -ne 250 } | ForEach-Object {
        $c = New-Object System.Net.Sockets.TcpClient
        [pscustomobject]@{ ip = "$sn.$_"; c = $c; t = $c.ConnectAsync("$sn.$_", 400) }
    }
    Start-Sleep -Milliseconds 1500
    $tasks | Where-Object { $_.t.Status -eq "RanToCompletion" } | ForEach-Object { Write-Host "FOUND Hydra candidate: $($_.ip):400" -ForegroundColor Green }
    $tasks | ForEach-Object { $_.c.Dispose() }
    Remove-NetIPAddress -InterfaceAlias $nic -IPAddress $me -Confirm:$false -ErrorAction SilentlyContinue
    Write-Host "  $sn.x done"
}
Stop-Transcript | Out-Null
Write-Host "Saved to $out"
