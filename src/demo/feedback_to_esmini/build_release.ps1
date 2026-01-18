$ErrorActionPreference = "Stop"

# Define paths
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Definition
$RepoRoot = Resolve-Path "$ScriptDir/../../.."
$BuildDir = Join-Path $RepoRoot "build"

Write-Host "Repository Root: $RepoRoot"
Write-Host "Build Directory: $BuildDir"

# Move to the repository root directory
Push-Location $RepoRoot

try {
    # Configure CMake
    Write-Host "`n[1/2] Configuring CMake (Release)..."
    cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
    if ($LASTEXITCODE -ne 0) { throw "CMake configuration failed." }

    # Build the specific target
    Write-Host "`n[2/2] Building Target: esmini_drive_chrono_feedback..."
    cmake --build build --target esmini_drive_chrono_feedback --config Release
    if ($LASTEXITCODE -ne 0) { throw "Build failed." }

    Write-Host "`nBuild Complete!"
}
catch {
    Write-Error $_
}
finally {
    Pop-Location
}
