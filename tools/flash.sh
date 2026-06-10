#!/usr/bin/env bash
# Build + flash the motor controller over ST-Link, then optionally open the serial menu.
#   ./tools/flash.sh            # build Debug + flash
#   ./tools/flash.sh release    # build Release + flash
#   ./tools/flash.sh -s         # flash Debug, then attach serial menu (115200, USART2)
set -euo pipefail
cd "$(dirname "$0")/.."

CONF=Debug
SERIAL=0
for a in "$@"; do
  case "$a" in
    release|Release) CONF=Release ;;
    -s|--serial)     SERIAL=1 ;;
  esac
done

echo ">> Building $CONF ..."
make -j"$(nproc)" -C "$CONF"

echo ">> Flashing $CONF/motorcontrol.elf via ST-Link ..."
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program $CONF/motorcontrol.elf verify reset exit"

if [[ "$SERIAL" == "1" ]]; then
  PORT="$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | head -1 || true)"
  if [[ -z "$PORT" ]]; then
    echo "!! No /dev/ttyACM* or /dev/ttyUSB* found. Plug in the USB-serial adapter (USART2)."
    exit 1
  fi
  echo ">> Opening $PORT @ 115200 (Ctrl-A then K to quit screen)"
  exec screen "$PORT" 115200
fi
