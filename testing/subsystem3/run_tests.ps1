$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$sourceRoot = Join-Path $projectRoot 'app_program/subsystem3'
$testExe = Join-Path ([System.IO.Path]::GetTempPath()) ('s3-tests-' + [guid]::NewGuid() + '.exe')
try {
    & gcc -std=c99 -Wall -Wextra -Werror -Wconversion -Wshadow -pedantic `
        -I $sourceRoot (Join-Path $sourceRoot 's3_ir.c') `
        (Join-Path $sourceRoot 's3_bar.c') `
        (Join-Path $sourceRoot 's3_nav.c') `
        (Join-Path $PSScriptRoot 'test_main.c') -o $testExe
    if ($LASTEXITCODE -ne 0) { throw 'Host compilation failed.' }
    & $testExe
    if ($LASTEXITCODE -ne 0) { throw 'Subsystem 3 tests failed.' }
} finally {
    if (Test-Path -LiteralPath $testExe) {
        Remove-Item -LiteralPath $testExe
    }
}
