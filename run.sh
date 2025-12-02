#!/bin/bash


# MENU
echo "==============================="
echo " Select an option:"
echo " 1 = Run Pico code"
echo " 2 = Run Python simulation"
echo " 3 = Run Pico code AND run Python"
echo "==============================="
read -p "Enter your choice (1/2/3): " choice
echo

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

case $choice in

    1)
        flash_pico
        ;;

    2)
        run_python
        ;;

    3)
        flash_pico
        run_python
        ;;

    *)
        echo "Invalid choice. Please enter 1, 2, or 3."
        ;;
esac
