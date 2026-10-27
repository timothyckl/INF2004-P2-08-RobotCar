param([string] $Compiler = 'gcc')

$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$outputDirectory = Join-Path ([System.IO.Path]::GetTempPath()) (
    'robotcar-s1-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $outputDirectory | Out-Null
$testExecutable = Join-Path $outputDirectory 's1_logic.exe'
$sourceFiles = @(
    'app_program/subsystem1/command_parser.c'
    'app_program/subsystem1/telemetry_encoder.c'
    'app_program/subsystem1/comms_timer.c'
    'app_program/subsystem1/comms_test.c'
    'testing/subsystem1/logic_main.c'
) | ForEach-Object { Join-Path $projectRoot $_ }

if ([System.IO.Path]::GetFileNameWithoutExtension($Compiler) -eq 'cl') {
    $compilerArguments = @('/nologo', '/std:c11', '/W4', '/WX', '/analyze',
        '/DAPP_TEST=1', '/DS1_HOST_TEST', ('/Fo' + $outputDirectory + '\'),
        ('/Fe' + $testExecutable))
} else {
    $compilerArguments = @('-std=c99', '-Wall', '-Wextra', '-Werror',
        '-Wconversion', '-Wsign-conversion', '-Wshadow', '-pedantic',
        '-DAPP_TEST=1', '-DS1_HOST_TEST', '-o', $testExecutable)
}

& $Compiler @compilerArguments @sourceFiles
if ($LASTEXITCODE -ne 0) { throw 'Subsystem 1 test compilation failed.' }
& $testExecutable
if ($LASTEXITCODE -ne 0) { throw 'Subsystem 1 logic tests failed.' }
Write-Output "Test executable: $testExecutable"
