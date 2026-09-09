param(
    [ValidateSet('configure','reconfigure','build','flash','monitor','chip','security','flash-id','partition-info','partition-scan')]
    [string]$Action='build',
    [string]$Port=''
)
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
$project=Join-Path $root 'firmware\esp_mosaico'
$build=Join-Path $root 'artifacts\mosaico\build'
$logs=Join-Path $root 'artifacts\mosaico\logs'
New-Item -ItemType Directory -Force $logs | Out-Null
# Query EIM's environment without executing its global profile/selection updates.
& 'C:\Espressif\tools\Microsoft.v6.1.PowerShell_profile.ps1' -e 6>&1 | ForEach-Object {
    if ($_ -match '^([^=]+)=(.*)$') { Set-Item -Path "Env:$($Matches[1])" -Value $Matches[2] }
}
$env:IDF_PY_BUILD_JOBS='2'
# The bootloader is a nested CMake build occupying one outer Ninja slot.
# Restrict that child to one job so total compiler concurrency stays <=2.
$env:CMAKE_BUILD_PARALLEL_LEVEL='1'
$env:PYTHONUNBUFFERED='1'
$env:PYTHONUTF8='1'
$env:PYTHONIOENCODING='utf-8'
$env:PATH='C:\Program Files\Git\cmd;'+$env:PATH
$python='C:\Espressif\tools\python\v6.1\venv\Scripts\python.exe'
$idf='C:\esp\v6.1\esp-idf\tools\idf.py'
$log=Join-Path $logs ((Get-Date -Format 'yyyyMMdd-HHmmss')+"-$Action.txt")
if ($Action -in @('flash','monitor','chip','security','flash-id','partition-info','partition-scan')) {
    # Win32_SerialPort can hang while a USB CDC endpoint is recovering. PnP
    # inventory does not open/probe every serial port (including unrelated ones).
    $ports=@(Get-PnpDevice -PresentOnly -Class Ports | Where-Object {$_.InstanceId -like 'USB\VID_303A*'} | ForEach-Object {
        if($_.FriendlyName -match '\((COM\d+)\)$'){[PSCustomObject]@{DeviceID=$Matches[1];PNPDeviceID=$_.InstanceId}}
    })
    if (!$Port) {
        if ($ports.Count -ne 1) { throw 'Connect exactly one identified Espressif USB serial device, or supply -Port.' }
        $Port=$ports[0].DeviceID
    }
    if ($Port -notin $ports.DeviceID) { throw 'Port is not a currently enumerated Espressif USB device.' }
}
switch ($Action) {
    'configure' { & $python $idf --preview -C $project -B $build set-target esp32s31 2>&1 | Tee-Object $log }
    'build' { & $python $idf --preview -C $project -B $build build 2>&1 | Tee-Object $log }
    'reconfigure' { & $python $idf --preview -C $project -B $build reconfigure 2>&1 | Tee-Object $log }
    'flash' {
        & $python $idf --preview -C $project -B $build build 2>&1 | Tee-Object $log
        if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
        # Kconfig can silently replace an out-of-range mailbox request with its
        # default. Never flash a burst-budget configuration different from source.
        foreach ($key in @('CONFIG_LWIP_UDP_RECVMBOX_SIZE','CONFIG_LWIP_TCPIP_RECVMBOX_SIZE')) {
            $expected=Get-Content (Join-Path $project 'sdkconfig.defaults') | Where-Object { $_ -like "$key=*" } | Select-Object -Last 1
            $actual=Get-Content (Join-Path $project 'sdkconfig') | Where-Object { $_ -like "$key=*" } | Select-Object -Last 1
            if ($expected -and $actual -ne $expected) { throw "Refusing mismatched runtime configuration: expected $expected, got $actual" }
        }
        # Do not silently flash a PSRAM clock different from the selected source.
        $expectedRam=Get-Content (Join-Path $project 'sdkconfig.defaults') | Where-Object {$_ -match '^CONFIG_SPIRAM_SPEED_\d+M=y$'} | Select-Object -Last 1
        $actualRam=Get-Content (Join-Path $project 'sdkconfig') | Where-Object {$_ -match '^CONFIG_SPIRAM_SPEED_\d+M=y$'} | Select-Object -Last 1
        if($expectedRam -and $expectedRam -ne $actualRam){throw "Refusing mismatched PSRAM setting: $expectedRam versus $actualRam"}
        # This preview SDK defaults to ROM/no-stub in bringup mode. Our verified
        # USB metadata probes use a RAM stub: never send ROM-only commands to it.
        $flashArgs=Get-Content (Join-Path $build 'app-flash_args')
        $images=@($flashArgs | Where-Object { $_ -match '^0x' })
        if ($images.Count -ne 1 -or $images[0] -notmatch '^0x20000 openframetap_mosaico.bin$') {
            throw 'Refusing a flash argument file outside the verified application partition.'
        }
        Push-Location $build
        try {
            & $python -m esptool --chip esp32s31 --port $Port --before no-reset --after hard-reset write-flash '@app-flash_args' 2>&1 | Tee-Object -Append $log
        } finally { Pop-Location }
    }
    'monitor' { & $python $idf --preview -C $project -B $build -p $Port monitor --no-reset 2>&1 | Tee-Object $log }
    'partition-info' {
        $table=Join-Path $logs 'original-partition-table.bin'
        if (Test-Path $table) { throw 'Partition metadata already recorded; refusing to overwrite.' }
        & $python -m esptool --port $Port --before no-reset --after no-reset read-flash 0x8000 0x1000 $table 2>&1 | Tee-Object $log
        if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
        & $python 'C:\esp\v6.1\esp-idf\components\partition_table\gen_esp32part.py' $table 2>&1 | Tee-Object -Append $log
    }
    'partition-scan' {
        $prefix=Join-Path $logs 'flash-layout-prefix.bin'
        if (Test-Path $prefix) { throw 'Layout prefix already captured; refusing to overwrite.' }
        & $python -m esptool --port $Port --before no-reset --after no-reset read-flash 0 0x40000 $prefix 2>&1 | Tee-Object $log
        if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
        & $python (Join-Path $PSScriptRoot 'mosaico-partitions.py') $prefix 2>&1 | Tee-Object -Append $log
    }
    default {
        $command=switch ($Action) { 'chip' {'chip-id'} 'security' {'get-security-info'} 'flash-id' {'flash-id'} }
        & $python -m esptool --port $Port --before no-reset --after no-reset $command 2>&1 | Tee-Object $log
    }
}
$code=$LASTEXITCODE
Write-Host "[OpenFrameTap] $Action exit=$code log=$log"
exit $code
