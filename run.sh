#!/bin/bash

# flash funtion 
flash_pico() {

    echo " Building Pico "
    cd build
    cmake ..
    make -j4 || { echo  "Build failed"; exit 1; }

    echo
    echo " Waiting for Pico to enter BOOTSEL mode "
    while [ ! -d /Volumes/RPI-RP2 ]; do
        echo "   Waiting... (Hold BOOTSEL and plug in Pico)"
        sleep 1
    done

    echo
    echo " Flashing firmware "
    UF2_FILE=$(ls *.uf2 | head -n 1)

    if [ -z "$UF2_FILE" ]; then
        echo "ERROR: No UF2 file found!"
        exit 1
    fi

    cp "$UF2_FILE" /Volumes/RPI-RP2
    echo "Firmware flashed"

    echo
    echo " Waiting for Pico to reboot "
    sleep 3

    cd ..
}

run_python() {
    echo
    echo " Running Python simulation "
    python3 tools/Imu_sim.py
}

# Cmake-Configure
configure() {
    echo "=== Configuring CMake ==="
    mkdir -p build
    cd build
    cmake ..
    cd ..
    echo "=== CMake configure done ==="
}

#Cmake build
build() {
    echo "=== Building firmware with CMake ==="
    cmake --build build || { echo " Build failed"; exit 1; }
    echo "=== Build complete ==="
}

clean() {
    echo "=== Cleaning build folder ==="
    rm -rf build
    mkdir build
    echo "=== Build folder cleaned ==="
}

echo "=================================="
echo "         LAUNCH MENU"
echo "=================================="
echo "1 = Configure CMake"
echo "2 = Build firmware"
echo "3 = Flash Pico firmware"
echo "4 = Run Python simulation/logger"
echo "5 = Clean build folder"
echo "6 = Build + Flash"
echo "7 = Build + Flash + Run Python"
echo "=================================="
read -p "Choose option (1–7): " choice
echo ""

# --------------------------------
# EXECUTE CHOICE
# --------------------------------

case $choice in
    1)
        configure
        ;;

    2)
        build
        ;;

    3)
        flash_pico
        ;;

    4)
        run_python
        ;;

    5)
        clean
        ;;

    6)
        configure
        build
        flash_pico
        ;;

    7)
        configure
        build
        flash_pico
        run_python
        ;;

    *)
        echo "Invalid option"
        ;;
esac