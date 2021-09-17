set -e

cargo +nightly build --release  --features stm32f072,reg-can --color=always --target thumbv6m-none-eabi
arm-none-eabi-objcopy -O binary ./target/thumbv6m-none-eabi/release/vhrd-bootloader ./target/thumbv6m-none-eabi/release/vhrd-bootloader-f072.bin

cargo +nightly build --release --features stm32f051,spi-can  --color=always --target thumbv6m-none-eabi
arm-none-eabi-objcopy -O binary ./target/thumbv6m-none-eabi/release/vhrd-bootloader ./target/thumbv6m-none-eabi/release/vhrd-bootloader-f051.bin

cargo +nightly build --release --features stm32f405,reg-can --color=always --target thumbv7em-none-eabihf
arm-none-eabi-objcopy -O binary ./target/thumbv6m-none-eabi/release/vhrd-bootloader ./target/thumbv6m-none-eabi/release/vhrd-bootloader-f405.bin

rsync -avz ./target/thumbv6m-none-eabi/release/*.bin pi@10.4.219.11:/home/pi/module-firmwares/