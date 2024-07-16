#!/bin/bash
# Attaches to the GDB server created in debug.sh.

if [ $# -eq 0 ]; then
  echo "Usage: $0 <elf_file>"
  exit 1
fi

ELF_FILE=$1

gdb-multiarch --quiet \
        -ex "target remote localhost:3333"\
        -ex "load"\
        -ex "monitor reset init"\
        -ex "break main"\
        -ex "continue"\
        -ex "layout src" $ELF_FILE
