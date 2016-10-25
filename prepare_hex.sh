proj_dir=$PWD
hex_dir=../../../hex
dirs=($(find . -name armgcc -type d))

for dir in "${dirs[@]}"; do
    echo
    pushd "$dir"
    echo
    # make clean
    make
    hex_name=$(echo $dir | sed "s|./||; s|$PWD/[^/]*/||; s|/armgcc|.hex|; s|/|_|g")
    mkdir -pv $hex_dir
    echo
    cp -pvu _build/nrf52832_xxaa.hex $hex_dir/$hex_name
    popd
    echo
    echo ------------------------
done