param([Parameter(Mandatory)][string]$Path,[double]$SkipSeconds=5)
$ErrorActionPreference='Stop'
if($SkipSeconds -lt 0){throw 'SkipSeconds must not be negative'}
$hash=(Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash
$capture_end=0L
$frames=@(Get-Content -LiteralPath $Path | ForEach-Object {
    if($_ -match '^OFT_BOOT .*uptime_ms=(\d+)'){$capture_end=[long]$Matches[1]*1000}
    if($_ -match '^OFT_VIDEO_PRESENT generation=(\d+) source_us=(\d+) ui_us=(\d+)'){
        [pscustomobject]@{generation=[long]$Matches[1];source=[long]$Matches[2];ui=[long]$Matches[3]}
    }
})
if($frames.Count -lt 2){throw 'Not enough GUI presentation timestamps'}
$start=$frames[0].ui+[long]($SkipSeconds*1e6)
$frames=@($frames|Where-Object {$_.ui -ge $start})
if($frames.Count -lt 2){throw 'Not enough frames after warmup'}
$gaps=@();$ages=@();$sources=@();$area=0.0;$peak=0.0
for($i=0;$i -lt $frames.Count;$i++){
    $f=$frames[$i]
    if($f.ui -lt $f.source){throw 'Invalid clock ordering'}
    $ages+=,(($f.ui-$f.source)/1000.0)
    if(!$i){continue}
    $previous=$frames[$i-1];$dt=($f.ui-$previous.ui)/1000.0
    if($dt -le 0 -or $f.generation -le $previous.generation -or $f.source -lt $previous.source){throw 'Non-monotonic evidence'}
    $gaps+=,$dt;$sources+=,(($f.source-$previous.source)/1000.0)
    $old_age=($previous.ui-$previous.source)/1000.0
    # Integrate age of the last GUI image while it is held, including pauses.
    $area+=$dt*($old_age+$dt/2.0)
    $peak=[Math]::Max($peak,$old_age+$dt)
}
$capture_end=[Math]::Max($capture_end,$frames[-1].ui)
$tail=($capture_end-$frames[-1].ui)/1000.0
$last_age=($frames[-1].ui-$frames[-1].source)/1000.0
$area+=$tail*($last_age+$tail/2.0);$peak=[Math]::Max($peak,$last_age+$tail)
$duration=($capture_end-$frames[0].ui)/1000.0
function P95($values){$sorted=@($values|Sort-Object);return $sorted[[Math]::Ceiling($sorted.Count*.95)-1]}
if((Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash -ne $hash){throw 'Raw evidence changed during analysis'}
[ordered]@{
    file=(Split-Path $Path -Leaf);sha256=$hash.ToLowerInvariant();skip_seconds=$SkipSeconds
    timestamp_definition='MCU AU-completing packet receive to GUI invalidation; NOT camera exposure or panel scanout'
    frames=$frames.Count;window_seconds=$duration/1000;fps=($frames.Count-1)*1000/$duration
    output_age_mean_ms=($ages|Measure-Object -Average).Average;output_age_p95_ms=(P95 $ages)
    held_image_age_time_mean_ms=$area/$duration;held_image_age_peak_ms=$peak
    update_gap_mean_ms=($gaps|Measure-Object -Average).Average;update_gap_p95_ms=(P95 $gaps);update_gap_max_ms=($gaps|Measure-Object -Maximum).Maximum
    trailing_hold_ms=$tail
    source_gap_p95_ms=(P95 $sources)
} | ConvertTo-Json
