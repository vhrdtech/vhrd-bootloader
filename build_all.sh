set -e

cargo +nightly build --release  --features stm32f072,reg-can --color=always --target thumbv6m-none-eabi
arm-none-eabi-objcopy -O binary ./target/thumbv6m-none-eabi/release/vhrd-bootloader ./target/thumbv6m-none-eabi/release/vhrd-bootloader-f072.bin

cargo +nightly build --release --features stm32f051,spi-can  --color=always --target thumbv6m-none-eabi
arm-none-eabi-objcopy -O binary ./target/thumbv6m-none-eabi/release/vhrd-bootloader ./target/thumbv6m-none-eabi/release/vhrd-bootloader-f051.bin

cargo +nightly build --release --features stm32f405,reg-can --color=always --target thumbv7em-none-eabihf
arm-none-eabi-objcopy -O binary ./target/thumbv7em-none-eabihf/release/vhrd-bootloader ./target/thumbv7em-none-eabihf/release/vhrd-bootloader-f405.bin
