<#
.SYNOPSIS
  Interactive helper for authoring xArm motion-graph edges by demonstration.

  Holds ONE cooperative claim for the whole session (with a background
  heartbeat) and wraps the /control/graph/* endpoints so each edge is a
  short command instead of full Invoke-RestMethod boilerplate.

.USAGE
  . .\tools\xarm-record.ps1          # dot-source to load the functions
  Connect-Xarm                       # claim + heartbeat + advisory mode
  Get-XarmGraph                      # verify your nodes are LIVE (see note)
  Pin  home                          # declare where the arm physically is
  Step cytation_approach -Comment "home->approach"        # move + record (joint)
  Step cytation_high_empty
  Step cytation_low_empty -Linear -Speed 15 -Pre gripper_empty   # vertical: linear
  ...
  Set-XarmMode strict                # enforce once the station is trusted
  Disconnect-Xarm                    # release the claim

.NOTES
  * The running controller loads motion_graph.yaml at BOOT and reloads it
    only as a side effect of a successful Record. If you uncommented /
    edited nodes after the service started, RESTART the 'xarm' service
    (elevated: nssm restart xarm) and confirm them with Get-XarmGraph
    BEFORE recording - there is no reload endpoint.
  * Record stores an EDGE (from, to, mode, speed) - NOT a trajectory.
    -Linear stamps the edge linear (straight-line TCP at replay); without
    it, joint-list poses default to joint (curved arc). edge.mode is
    authoritative on replay; edge.speed is a max cap.
  * Record CANNOT create the grip/release (payload-changing) edges - those
    need an action block. Uncomment them from the template in
    motion_graph.yaml instead; they go live on the next Record or restart.
  * SAFETY: a discovery Move always runs as JOINT (no edge exists yet), so a
    vertical descent demoed via Move/Step takes a curved path even if you
    pass -Linear (which only affects the *recorded* edge). For descents over
    labware, prefer uncommenting the pre-authored linear edge, or verify the
    curved demo path is clear first.
  * All /control/graph/* calls are claim-gated. Don't also click "Take
    Control" in /web/ - it would hold the only claim and your calls 423.
#>

$script:XG = $null

function _Need {
    if (-not $script:XG) { throw "not connected - run Connect-Xarm first" }
}

# Extract HTTP status + body from a thrown WebException (Windows PowerShell 5.1).
function _XGErr($e) {
    $code = $null; $body = $null
    $resp = $e.Exception.Response
    if ($resp) {
        try { $code = [int]$resp.StatusCode } catch {}
        try {
            $sr = New-Object IO.StreamReader($resp.GetResponseStream())
            $body = $sr.ReadToEnd(); $sr.Close()
        } catch {}
    }
    if (-not $body) { $body = $e.Exception.Message }
    return "HTTP $code $body"
}

function Connect-Xarm {
    [CmdletBinding()]
    param(
        [string]$BaseUrl = 'http://127.0.0.1:8000',
        [string]$Owner   = 'yang:authoring',
        [double]$Ttl     = 120,
        [ValidateSet('off','advisory','strict')][string]$Mode = 'advisory',
        [switch]$NoHeartbeat
    )
    $sid  = [guid]::NewGuid().ToString()
    $body = @{ owner = $Owner; session_id = $sid; ttl_s = $Ttl } | ConvertTo-Json
    try {
        $resp = Invoke-RestMethod -Method Post -Uri "$BaseUrl/control/claim" `
            -ContentType application/json -Body $body
    } catch { Write-Error "claim failed: $(_XGErr $_)"; return }

    $script:XG = [ordered]@{
        BaseUrl   = $BaseUrl
        Token     = $resp.claim_token
        Headers   = @{ 'X-Claim-Token' = $resp.claim_token }
        SessionId = $sid
        Owner     = $Owner
        Hb        = $null
    }

    $sleep = 0
    if (-not $NoHeartbeat) {
        $interval = [double]$resp.heartbeat_interval_s
        if (-not $interval -or $interval -le 0) { $interval = $Ttl / 2 }
        $sleep = [Math]::Max(2, [int]($interval / 2))
        $script:XG.Hb = Start-Job -Name 'xarm-hb' -ScriptBlock {
            param($uri, $tok, $sleep)
            while ($true) {
                Start-Sleep -Seconds $sleep
                try {
                    Invoke-RestMethod -Method Post -Uri "$uri/control/heartbeat" `
                        -Headers @{ 'X-Claim-Token' = $tok } | Out-Null
                } catch {}
            }
        } -ArgumentList $BaseUrl, $resp.claim_token, $sleep
    }

    try { Set-XarmMode $Mode | Out-Null } catch { Write-Warning "could not set mode: $_" }

    Write-Host "claimed ($Owner / $($sid.Substring(0,8))) mode=$Mode" -ForegroundColor Green
    if ($sleep) { Write-Host "heartbeat job 'xarm-hb' every ${sleep}s" -ForegroundColor DarkGray }
    Write-Host "Reminder: if you authored nodes after boot, restart 'xarm' and check Get-XarmGraph before recording." -ForegroundColor Yellow
}

function Disconnect-Xarm {
    if (-not $script:XG) { Write-Warning 'not connected'; return }
    if ($script:XG.Hb) {
        Stop-Job   $script:XG.Hb -ErrorAction SilentlyContinue
        Remove-Job $script:XG.Hb -Force -ErrorAction SilentlyContinue
    }
    try {
        Invoke-RestMethod -Method Post -Uri "$($script:XG.BaseUrl)/control/release" `
            -Headers $script:XG.Headers | Out-Null
        Write-Host 'released claim.' -ForegroundColor Green
    } catch { Write-Warning "release failed: $(_XGErr $_)" }
    $script:XG = $null
}

function Set-XarmMode {
    param([Parameter(Mandatory)][ValidateSet('off','advisory','strict')]$Mode)
    _Need
    $b = @{ mode = $Mode } | ConvertTo-Json
    Invoke-RestMethod -Method Post -Uri "$($script:XG.BaseUrl)/control/graph/mode" `
        -Headers $script:XG.Headers -ContentType application/json -Body $b
}

# GET /graph - no claim needed. Pipe to `ConvertTo-Json -Depth 5` for full adjacency.
function Get-XarmGraph {
    $base = if ($script:XG) { $script:XG.BaseUrl } else { 'http://127.0.0.1:8000' }
    Invoke-RestMethod -Uri "$base/graph"
}

function Get-XarmNearest {
    param([double]$JointTol = 10, [double]$RailTol = 2)
    $base = if ($script:XG) { $script:XG.BaseUrl } else { 'http://127.0.0.1:8000' }
    Invoke-RestMethod -Uri "$base/graph/nearest?joint_tolerance_deg=$JointTol&rail_tolerance_mm=$RailTol"
}

function Pin-XarmNode {
    param([Parameter(Mandatory)][string]$Node, [switch]$Force)
    _Need
    $b = @{ node_id = $Node; force = [bool]$Force } | ConvertTo-Json
    try {
        $r = Invoke-RestMethod -Method Post -Uri "$($script:XG.BaseUrl)/control/graph/recover_to" `
            -Headers $script:XG.Headers -ContentType application/json -Body $b
        Write-Host "pinned -> $($r.current_node)" -ForegroundColor Cyan
        $r
    } catch { Write-Error "pin failed: $(_XGErr $_)" }
}

function Move-XarmNode {
    param([Parameter(Mandatory)][string]$Node, [double]$Speed)
    _Need
    $h = @{ node_id = $Node }
    if ($PSBoundParameters.ContainsKey('Speed')) { $h.speed = $Speed }
    $b = $h | ConvertTo-Json
    try {
        $r = Invoke-RestMethod -Method Post -Uri "$($script:XG.BaseUrl)/control/graph/move_to" `
            -Headers $script:XG.Headers -ContentType application/json -Body $b
        Write-Host "moved -> $($r.current_node)" -ForegroundColor Cyan
        $r
    } catch { Write-Error "move failed: $(_XGErr $_)" }
}

function Record-XarmEdge {
    param(
        [switch]$Linear,
        [double]$Speed,
        [string]$Comment,
        [string[]]$Pre
    )
    _Need
    $h = @{}
    if ($Linear) { $h.mode = 'linear' }
    if ($PSBoundParameters.ContainsKey('Speed')) { $h.speed = $Speed }
    if ($Comment) { $h.comment = $Comment }
    if ($Pre)     { $h.preconditions = $Pre }
    $b = $h | ConvertTo-Json
    try {
        $r = Invoke-RestMethod -Method Post -Uri "$($script:XG.BaseUrl)/control/graph/record" `
            -Headers $script:XG.Headers -ContentType application/json -Body $b
        $e = $r.recorded
        Write-Host "recorded: $($e.from) -> $($e.to) [$($e.mode)$(if($e.speed){" @$($e.speed)"})]" -ForegroundColor Green
        $r
    } catch { Write-Error "record failed: $(_XGErr $_)" }
}

# move + record in one shot - the authoring workhorse.
function Step-XarmEdge {
    param(
        [Parameter(Mandatory)][string]$To,
        [switch]$Linear,
        [double]$Speed,
        [string]$Comment,
        [string[]]$Pre
    )
    _Need
    if ($PSBoundParameters.ContainsKey('Speed')) {
        $mv = Move-XarmNode -Node $To -Speed $Speed
    } else {
        $mv = Move-XarmNode -Node $To
    }
    if (-not $mv) { return }   # move failed; don't record a stale transition

    $rec = @{}
    if ($Linear) { $rec.Linear = $true }
    if ($PSBoundParameters.ContainsKey('Speed')) { $rec.Speed = $Speed }
    if ($Comment) { $rec.Comment = $Comment }
    if ($Pre)     { $rec.Pre = $Pre }
    Record-XarmEdge @rec
}

# Ergonomic short aliases for the interactive hot path.
Set-Alias graph Get-XarmGraph
Set-Alias near  Get-XarmNearest
Set-Alias gmode Set-XarmMode
Set-Alias pin   Pin-XarmNode
Set-Alias step  Step-XarmEdge
Set-Alias rec   Record-XarmEdge

Write-Host "xarm-record helpers loaded. Connect-Xarm to begin; Step <node> to author edges." -ForegroundColor Cyan
