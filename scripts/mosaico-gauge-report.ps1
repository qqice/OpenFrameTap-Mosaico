param([Parameter(Mandatory)][string[]]$Logs,[Parameter(Mandatory)][string]$Output)
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
$destination=[IO.Path]::GetFullPath((Join-Path $root $Output))
$allowed=[IO.Path]::GetFullPath((Join-Path $root 'artifacts'))+[IO.Path]::DirectorySeparatorChar
if(!$destination.StartsWith($allowed,[StringComparison]::OrdinalIgnoreCase)){throw 'Output must stay in artifacts'}
if(Test-Path -LiteralPath $destination){throw 'Do not overwrite an existing evidence report'}
$samples=[Collections.Generic.SortedDictionary[long,object]]::new()
$sources=@();$invalid=0
foreach($path in $Logs){
    $full=(Resolve-Path -LiteralPath (Join-Path $root $path)).Path
    $hash=(Get-FileHash -LiteralPath $full -Algorithm SHA256).Hash.ToLowerInvariant()
    foreach($line in [IO.File]::ReadLines($full)){
        if(!$line.StartsWith('OFT_GAUGE_SAMPLE ')){continue}
        $values=@{};foreach($match in [regex]::Matches($line,'(\w+)=([^\s]+)')){$values[$match.Groups[1].Value]=$match.Groups[2].Value}
        try{
            foreach($field in @('us','valid','soc','mv','raw_ma','raw_design','raw_rm','raw_fcc')){if(!$values.ContainsKey($field)){throw 'Incomplete row'}}
            $sample=[pscustomobject]@{us=[long]$values.us;valid=[int]$values.valid;soc=[int]$values.soc;mv=[int]$values.mv;raw_ma=[int]$values.raw_ma;raw_design=[int]$values.raw_design;raw_rm=[int]$values.raw_rm;raw_fcc=[int]$values.raw_fcc;status=$values.status;flags=$values.flags}
            if($sample.valid -ne 1 -or $sample.raw_design -ne 130 -or $sample.soc -notin 0..100){throw 'Invalid or differently scaled row'}
            if($samples.ContainsKey($sample.us)){
                if(($samples[$sample.us]|ConvertTo-Json -Compress) -ne ($sample|ConvertTo-Json -Compress)){throw 'Conflicting timestamp; do not combine different MCU boots'}
            }else{$samples.Add($sample.us,$sample)}
        }catch{$invalid++;throw "Cannot use sample in ${path}: $line ($_)"}
    }
    if((Get-FileHash -LiteralPath $full -Algorithm SHA256).Hash.ToLowerInvariant() -ne $hash){throw 'Input changed while being read'}
    $sources+=@{path=$path;sha256=$hash}
}
$rows=@($samples.Values);if($rows.Count -lt 2){throw 'At least two timestamped samples required'}
$mah=0.0;$maxGap=0.0;$omitted=0.0
for($i=1;$i -lt $rows.Count;$i++){
    $seconds=($rows[$i].us-$rows[$i-1].us)/1e6;$maxGap=[Math]::Max($maxGap,$seconds)
    if($seconds -gt 45){$omitted+=$seconds;continue}
    $mah+=($rows[$i].raw_ma+$rows[$i-1].raw_ma)/4.0*$seconds/3600
}
$result=@{
    generated_at_utc=[DateTime]::UtcNow.ToString('o');analysis_git_head=(git -C $root rev-parse HEAD)
    sources=$sources;raw_evidence_unchanged=$true;sample_count=$rows.Count;invalid_rows=$invalid
    scale=2;span_seconds=($rows[-1].us-$rows[0].us)/1e6;max_gap_seconds=$maxGap;omitted_gap_seconds=$omitted
    signed_charge_integral_mah=$mah;soc_start=$rows[0].soc;soc_end=$rows[-1].soc
    fcc_start_mah=$rows[0].raw_fcc/2.0;fcc_end_mah=$rows[-1].raw_fcc/2.0
    rm_start_mah=$rows[0].raw_rm/2.0;rm_end_mah=$rows[-1].raw_rm/2.0
    voltage_min_mv=($rows|Measure-Object mv -Minimum).Minimum;voltage_max_mv=($rows|Measure-Object mv -Maximum).Maximum
    final_current_ma=$rows[-1].raw_ma/2.0;final_voltage_mv=$rows[-1].mv
    confidence='Gauge-scaled charging observation, not an independent capacity measurement or discharge calibration'
    samples=$rows
}
[IO.Directory]::CreateDirectory([IO.Path]::GetDirectoryName($destination))|Out-Null
$result|ConvertTo-Json -Depth 8|Set-Content -LiteralPath $destination -Encoding UTF8
$result.Remove('samples');$result|ConvertTo-Json -Depth 5
