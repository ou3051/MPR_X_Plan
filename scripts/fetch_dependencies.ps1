param(
    [ValidateSet("googletest", "dcmtk", "all")]
    [string]$Dependency = "googletest"
)

$ErrorActionPreference = "Stop"
$repoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$thirdParty = Join-Path $repoRoot "third_party"
New-Item -ItemType Directory -Force $thirdParty | Out-Null

function Clone-PinnedRepo {
    param(
        [string]$Name,
        [string]$Url,
        [string]$Tag,
        [string]$ExpectedCommit
    )

    $target = Join-Path $thirdParty $Name
    if (Test-Path $target) {
        Write-Host "$Name already exists at $target"
        return
    }

    git clone --branch $Tag --depth 1 $Url $target
    if ($LASTEXITCODE -ne 0) {
        throw "Failed to clone $Name from $Url at $Tag"
    }

    Push-Location $target
    $actual = git rev-parse HEAD
    if ($LASTEXITCODE -ne 0) {
        Pop-Location
        throw "Failed to resolve commit for $Name"
    }
    Pop-Location

    if ($actual -ne $ExpectedCommit) {
        throw "$Name commit mismatch. Expected $ExpectedCommit, got $actual"
    }
}

if ($Dependency -eq "googletest" -or $Dependency -eq "all") {
    Clone-PinnedRepo `
        -Name "googletest" `
        -Url "https://github.com/google/googletest.git" `
        -Tag "v1.14.0" `
        -ExpectedCommit "f8d7d77c06936315286eb55f8de22cd23c188571"
}

if ($Dependency -eq "dcmtk" -or $Dependency -eq "all") {
    Clone-PinnedRepo `
        -Name "dcmtk" `
        -Url "https://github.com/DCMTK/dcmtk.git" `
        -Tag "DCMTK-3.7.0" `
        -ExpectedCommit "ccfd10b84ff3c9a40b7b331698aedf06d421fc43"
}
