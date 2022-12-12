#!/bin/bash -axe
openocd -d2 -f interface/stlink.cfg -c "transport select hla_swd" -f target/stm32f1x.cfg -c "reset_config none" -c "program {./firmware.elf}  verify reset; shutdown;"
