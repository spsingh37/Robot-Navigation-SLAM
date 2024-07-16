#!/bin/bash

if [ $# -eq 0 ]; then
  echo "Usage: $0 <elf_file>"
  exit 1
fi

ELF_FILE=$1

openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program $ELF_FILE verify reset exit"

