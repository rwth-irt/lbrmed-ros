# get path of this script
$scriptpath = $MyInvocation.MyCommand.Path
$script_dir = Split-Path $scriptpath
Write-host "My directory is $script_dir"

New-Item -ItemType SymbolicLink -Path "src" -Target "$script_dir\src"
New-Item -ItemType SymbolicLink -Path "ROSJavaLib" -Target "$script_dir\ROSJavaLib"
