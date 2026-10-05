#!/usr/bin/env bash
set -e

PORT=""

while getopts "p:" opt; do
    case "$opt" in
        p) PORT="$OPTARG" ;;
        *)
            echo "Usage: $0 -p <COM port number>"
            echo "Example: $0 -p 15"
            exit 1
            ;;
    esac
done

if [ -z "$PORT" ]; then
    echo "Usage: $0 -p <COM port number>"
    echo "Example: $0 -p 15"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/src"

PYTHON="/c/Users/Super/AppData/Local/Programs/Python/Python313/python.exe"

echo "========================================"
echo " Torque Dolly ELRS TX"
echo " Build + Flash"
echo " Port: COM${PORT}"
echo "========================================"

PLATFORMIO_BUILD_FLAGS='-D TORQUE_DOLLY_RF_TEST=1 -D TORQUE_DOLLY_HANDSET=1' \
ELRS_UNIFIED_CONFIG='diy.tx_2400.torque-dolly-s3-f27' \
"$PYTHON" -m platformio run \
    -e Unified_ESP32S3_2400_TX_via_UART \
    -t upload \
    --upload-port "COM${PORT}"

echo
echo "========================================"
echo " Build + Flash complete"
echo "========================================"
