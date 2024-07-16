#!/bin/bash
#Creates a GDB server for the Pico.

if [ $# -eq 0 ]; then
  echo "Usage: $0 <elf_file>"
  exit 1
fi

ELF_FILE=$1

openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg
gdb-multiarch $ELF_FILE --init-command="target remote localhost:3333"
