# SteamVR driver for Mercury hand tracking!

This is very in-development. Check back later!

## Build Instructions

Prerequisites:
- vcpkg
- CMake
- Ninja
- clang
- Visual Studio 2019, or 2022

### Clone the repository
```
git clone https://github.com/moshimeow/mercury_steamvr_driver.git --recursive
cd mercury_steamvr_driver
git submodule init
```

### Download onnx
```
powershell .\attic\moshi_get_onnxruntime.ps1
```

### Set your vcpkg path
Edit the `vcpkgdir` variable in [attic/moshi_build.ps1](attic/moshi_build.ps1) to reflect your vcpkg path.

### Build the project
```
powershell .\attic\moshi_build.ps1
```

Once this has completed, copy `onnxruntime.dll` from the `deps` folder to `build/mercury/bin/win64/`.
