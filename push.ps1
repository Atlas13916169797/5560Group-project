param(
    [string]$Message = "Update project files"
)

$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot

$gitTool = Get-Command git -ErrorAction SilentlyContinue
if ($gitTool) {
    $gitExe = $gitTool.Source
} else {
    $candidates = @(
        (Join-Path $env:LOCALAPPDATA 'Programs\Git\cmd\git.exe'),
        'C:\Program Files\Git\cmd\git.exe'
    )
    $gitExe = $candidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
}
if (-not $gitExe) { throw 'Git was not found. Install Git or restart PowerShell.' }

$repoRoot = & $gitExe rev-parse --show-toplevel
if ($LASTEXITCODE -ne 0) { throw 'This project has not been initialized as a Git repository.' }
$expectedRoot = [IO.Path]::GetFullPath($PSScriptRoot).TrimEnd('\', '/')
$actualRoot = [IO.Path]::GetFullPath($repoRoot).TrimEnd('\', '/')
if ($actualRoot -ne $expectedRoot) { throw 'Repository root does not match the project folder.' }

& $gitExe status --short
& $gitExe add --all
if ($LASTEXITCODE -ne 0) { throw 'Could not stage the project changes.' }

& $gitExe diff --cached --quiet
$diffExit = $LASTEXITCODE
if ($diffExit -eq 1) {
    & $gitExe commit -m $Message
    if ($LASTEXITCODE -ne 0) { throw 'Commit failed. The changes have not been pushed.' }
} elseif ($diffExit -ne 0) {
    throw 'Could not inspect staged changes.'
} else {
    Write-Output 'No new local changes to commit.'
}

& $gitExe push
if ($LASTEXITCODE -ne 0) {
    throw 'Push failed. Local commits are preserved. Check authentication or remote changes; no force push was used.'
}
Write-Output 'Project changes have been pushed successfully.'
