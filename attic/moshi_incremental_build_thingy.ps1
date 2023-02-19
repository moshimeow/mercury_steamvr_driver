# Copyright 2023, Moshi Turner. All rights reserved.

ninja -C build
if ($?) {
    .\build\THINGY.exe
}