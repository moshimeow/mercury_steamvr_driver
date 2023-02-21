param(
    $exec_file
)

ninja -C build


if ($?){
    & $exec_file
}