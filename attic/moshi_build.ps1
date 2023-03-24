# Copyright 2019-2022, Mesa contributors
# Copyright 2022, Collabora, Ltd.
# SPDX-License-Identifier: MIT
# Based on https://gitlab.freedesktop.org/mesa/mesa/-/blob/8396df5ad90aeb6ab2267811aba2187954562f81/.gitlab-ci/windows/mesa_build.ps1


$vcpkgdir = "c:\dev\vcpkg"
$toolchainfile = Join-Path $vcpkgdir "scripts/buildsystems/vcpkg.cmake"


# Grr this should work
$vcpkgexe = Join-Path $vcpkgdir "vcpkg.exe"

# What? how does this make any freaking sense lol
# c:\dev\vcpkg\vcpkg.exe install cjson:x64-windows eigen3:x64-windows wil:x64-windows pthreads:x64-windows glslang:x64-windows libusb:x64-windows hidapi:x64-windows opencv:x64-windows

c:\dev\vcpkg\vcpkg.exe install

$env:PYTHONUTF8 = 1

Get-Date
Write-Host "Compiling Monado"
$sourcedir = (Resolve-Path "$PSScriptRoot/..")
$builddir = Join-Path $sourcedir "build"
$installdir = Join-Path $sourcedir "install"


Remove-Item -Recurse -Force $installdir -ErrorAction SilentlyContinue

Write-Output "builddir:$builddir"
Write-Output "installdir:$installdir"
Write-Output "sourcedir:$sourcedir"
Write-Output "toolchainfile:$toolchainfile"

# I don't understand this at all. This string ends up finding VS 2019 on my machine (but *NOT* VS 2022, which is what we want), and may work for yours. Patches very welcome.
$version = '[16.0,17.0]'
$installPath = & "C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe" -version $version -property installationpath
Write-Output "vswhere.exe installPath: $installPath"
# $installPath = "C:\BuildTools"
# Write-Output "Final installPath: $installPath"

# Note that we can't have $ErrorActionPreference as "Stop" here:
# it "errors" (not finding some shared tool because of our mini build tools install)
# but the error doesn't matter for our use case.
Import-Module (Join-Path $installPath "Common7\Tools\Microsoft.VisualStudio.DevShell.dll")
Enter-VsDevShell -VsInstallPath $installPath -SkipAutomaticLocation -DevCmdArguments '-arch=x64 -no_logo -host_arch=amd64'

Push-Location $sourcedir
$cmakeArgs = @(
    "-S"
    "."
    "-B"
    "$builddir"
    "-GNinja"
    "-DCMAKE_BUILD_TYPE=RelWithDebInfo"
    "-DCMAKE_TOOLCHAIN_FILE=$toolchainfile"
    "-DCMAKE_INSTALL_PREFIX=$installdir"
)
cmake @cmakeArgs

ninja -C $builddir
# ninja -C $builddir install test

$buildstatus = $?
Pop-Location

Get-Date

if (!$buildstatus) {
    Write-Host "Monado build or test failed"
    Exit 1
}
