param(
    [string]$OutputDirectory = (Join-Path $PSScriptRoot 'dist')
)

$ErrorActionPreference = 'Stop'
$manifestPath = Join-Path $PSScriptRoot 'BoxJoint.manifest'
$manifest = Get-Content -LiteralPath $manifestPath -Raw | ConvertFrom-Json -AsHashtable
$packageName = "AutoBoxJoint-v$($manifest['version'])-windows.zip"
$resolvedOutput = [System.IO.Path]::GetFullPath($OutputDirectory)
$stagingRoot = Join-Path ([System.IO.Path]::GetTempPath()) ("AutoBoxJoint-package-" + [guid]::NewGuid().ToString('N'))
$stagedAddIn = Join-Path $stagingRoot 'BoxJoint'
$archivePath = Join-Path $resolvedOutput $packageName

$files = @(
    'auto_box_joint_layout.py',
    'BoxJoint.manifest',
    'BoxJoint.py',
    'fusion_auto_box_joint.py',
    'fusion_brep_util.py',
    'fusion_util.py',
    'LICENSE',
    'README.md',
    'INSTALL.zh-CN.txt',
    'build_package.ps1'
)

try {
    New-Item -ItemType Directory -Path $stagedAddIn -Force | Out-Null
    New-Item -ItemType Directory -Path $resolvedOutput -Force | Out-Null
    foreach ($file in $files) {
        Copy-Item -LiteralPath (Join-Path $PSScriptRoot $file) -Destination $stagedAddIn
    }
    Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'Resources') -Destination $stagedAddIn -Recurse
    Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'docs') -Destination $stagedAddIn -Recurse
    if (Test-Path -LiteralPath $archivePath) {
        Remove-Item -LiteralPath $archivePath -Force
    }
    Compress-Archive -LiteralPath $stagedAddIn -DestinationPath $archivePath -CompressionLevel Optimal
    Write-Output $archivePath
}
finally {
    if (Test-Path -LiteralPath $stagingRoot) {
        Remove-Item -LiteralPath $stagingRoot -Recurse -Force
    }
}
