# Rust Modbus context example with stm32 (stm32f103c8t6) &amp; RS485

Very simple example of Modbus context on stm32 via RS485/RTU.

## Hardware

* stm32f103c8t6

* UART TTL to RS485 (MAX485)

* ST-LINK/V2 (for flashing)

* any USB RS485 whistle (for testing from PC)

## Wiring

![Connection scheme](scheme.png?raw=true "Connection scheme")

## Software

* any Modbus client (e.g. https://github.com/favalex/modbus-cli for tests)

## Libraries

* https://github.com/stm32-rs/stm32f1xx-hal/ for STM32F1 HAL

* https://github.com/alttch/rmodbus for Modbus

## Flashing

```shell
cargo install cargo-flash # if not installed yed
cargo flash --chip stm32f103C8 --release
```

## What can it do

Just the demo. You can read/write Modbus context with any Modbus/RTU client,
get/set any Modbus registers.

Input registers 0 and 1 contain processed frame counter (big-endian u32).

Rmodbus used with "smallcontext" feature, so only registers 0-999 (all types)
are accessible.

Enjoy!
