#![no_std]
#![no_main]
//#![deny(warnings)]
#![feature(array_methods)]
#![feature(const_option)]

use cortex_m::asm;
use cortex_m::asm::delay;
use cortex_m_rt::{entry, exception, ExceptionFrame};

mod peripherals;
use vhrd_flash_writer::flash::{flash_read, flash_read_slice, stm32_device_signature, FlashWriter, FlashWriterError, flash_size_bytes};
use peripherals::stm32p::*;
use rtt_target::{rprintln, rtt_init_print};

use core::panic::PanicInfo;
use core::sync::atomic::{self, compiler_fence, Ordering};
use vhrd_flash_writer::mem_ext::MemExt;

use vhrd_module_nvconfig::{NVConfig, NV_CONFIG_START_ADDR, SIZE_OF_NVCONFIG};
use cortex_m::peripheral::syst::SystClkSource;

use stm32f0xx_hal as hal;

use crate::hal::{delay::Delay, pac, prelude::*};
use vhrdcan::{FrameId, RawFrame, RawFrameRef};
use core::borrow::BorrowMut;
use stm32f0xx_hal::pac::syscfg::cfgr1::MEM_MODE_A::SRAM;
use crc::{Crc, Algorithm, CRC_32_AUTOSAR};
use mcp25625::{McpPriority, McpReceiveBuffer};
use crate::CanState::WaitingOfTransfer;
use crate::State::CheckNVConfig;

mod ticks_time;

fn blink_led(led: &mut UsrLedPin, amount_of_blinks: u8){
    for _ in 0..amount_of_blinks{
        led.set_high().unwrap();
        delay(1_000_000);
        led.set_low();
        delay(1_000_000);
    }
}

fn get_crc(data: &[u8]) -> u64{
    Crc::<u32>::new(&CRC_32_AUTOSAR).checksum(data) as u64
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
enum State {
    CheckNVConfig,
    CheckBootloaderValidity,
    CheckForFirmware,
    CheckFirmwareValidity,
    WaitingNewCommandTimeout,
    WaitingNewCommandEndless,
    Boot,
    Reboot,
    Error
}
const RX_NV_CONFIG: FrameId = FrameId::new_extended(0x0000_0001).unwrap();
const RX_NEW_FIRMWARE: FrameId = FrameId::new_extended(0x0000_0010).unwrap();

const BOOTLOADER_KEY: u32 = 0xDEADBEEF;

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
enum CanState{
    WaitingOfTransfer,
    StartOfTransfer,
//    CRCTransfer,
    DataTransfer,
    EndOfTransfer
}

fn can_parser<'a>(expecting_id: &FrameId, prev_can_state: CanState, rx_frame: &'a RawFrame) -> (CanState, &'a [u8]) {
    if rx_frame.id == *expecting_id{
        if prev_can_state == CanState::WaitingOfTransfer && rx_frame.len == 4 && u32::from_be_bytes([rx_frame.data[0],rx_frame.data[1],rx_frame.data[2],rx_frame.data[3]]) == BOOTLOADER_KEY{
            return (CanState::StartOfTransfer, &[]);
        }
        else if rx_frame.len != 0 && (prev_can_state == CanState::StartOfTransfer || prev_can_state == CanState::DataTransfer){
            return (CanState::DataTransfer, &rx_frame.data[0..rx_frame.len as usize]);
        }
        else if rx_frame.len == 0 && prev_can_state == CanState::DataTransfer{
            return (CanState::EndOfTransfer, &[]);
        }
    }
    (CanState::WaitingOfTransfer, &[])
}



#[entry]
fn main() -> ! {
    rtt_init_print!();
    let mut state = State::CheckNVConfig;
    let mut prev_state = state;
    let mut p = peripherals::setup_peripherals();
    let mut nv_config_range = FlashWriter::new(NV_CONFIG_START_ADDR as u32..(NV_CONFIG_START_ADDR as u32 + SIZE_OF_NVCONFIG as u32)).unwrap();
    let mut firmware_range = FlashWriter::new(
        (NV_CONFIG_START_ADDR as u32 + SIZE_OF_NVCONFIG as u32)
            ..(0x0800_0000 + flash_size_bytes())).unwrap();

    let nv_config = NVConfig::get();
    let mut pass_time_us = 0u64;
    rprintln!("Start");
    let mut can_state = CanState::WaitingOfTransfer;
    loop {
         state = match state{
             State::CheckNVConfig => {
                 prev_state = State::CheckNVConfig;
                 match get_crc(flash_read_slice::<u8>(NV_CONFIG_START_ADDR as u32 + 8u32, SIZE_OF_NVCONFIG - 8)) == nv_config.config_crc{
                     true => { State::CheckBootloaderValidity }
                     false => {
                         rprintln!("0x{:08x} != 0x{:08x}", nv_config.config_crc, get_crc(flash_read_slice::<u8>(NV_CONFIG_START_ADDR as u32 + 8u32, SIZE_OF_NVCONFIG - 8)));
                         State::Error
                     }
                 }
             }
             State::CheckBootloaderValidity => {
                 prev_state = State::CheckBootloaderValidity;
                 if nv_config.board_config.bootloader_size + 0x0800_0000 < NV_CONFIG_START_ADDR as u32 && nv_config.board_config.bootloader_size != 0 {
                     match get_crc(flash_read_slice::<u8>(0x0800_0000, nv_config.board_config.bootloader_size as usize)) == nv_config.board_config.bootloader_crc {
                         true => { State::CheckForFirmware }
                         false => { State::Error }
                     }
                 }
                 else { State::Error }
             }
             State::CheckForFirmware => {
                 prev_state = State::CheckForFirmware;
                 if nv_config.board_config.fw_size > 0 && nv_config.board_config.fw_size < (flash_size_bytes() - SIZE_OF_NVCONFIG as u32 - 10u32.kb())  { State::CheckFirmwareValidity } // 10240 - max bootloader size
                 else { State::Error }
             }
             State::CheckFirmwareValidity => {
                 prev_state = State::CheckFirmwareValidity;
                 match get_crc(flash_read_slice::<u8>(firmware_range.get_start_address(), nv_config.board_config.fw_size as usize)) == nv_config.board_config.fw_crc {
                     true => {
                         p.3.count_time_from_now();
                         State::WaitingNewCommandTimeout
                     }
                     false => { State::WaitingNewCommandEndless }
                 }
             }
             State::WaitingNewCommandTimeout => {
                 prev_state = State::WaitingNewCommandTimeout;
                 match nv_config.board_config.bootloader_timeout_ms <= (pass_time_us / 1000) as u16{
                     true => {
                         State::Boot
                     }
                     false => {
                         let mut state = State::WaitingNewCommandTimeout;
                         if p.5.interrupt_flags().rx0if_is_set() {
                             let rx_frame = p.5.receive(McpReceiveBuffer::Buffer0);
                             match rx_frame.id{
                                 RX_NV_CONFIG => {
                                     prev_state = State::CheckNVConfig;
                                     state = State::WaitingNewCommandEndless;
                                 }
                                 RX_NEW_FIRMWARE =>{
                                     prev_state = State::CheckForFirmware;
                                     state = State::WaitingNewCommandEndless;
                                 }
                                 _ => {
                                     state = State::WaitingNewCommandTimeout;
                                 }
                             }
                         }
                         pass_time_us += p.3.get_delta_time_us() as u64;
                         state
                     }
                 }
             }
             State::WaitingNewCommandEndless => {
                 match prev_state{
                     State::CheckNVConfig | State::CheckBootloaderValidity => {
                         if p.5.interrupt_flags().rx0if_is_set() {
                             let rx_frame = p.5.receive(McpReceiveBuffer::Buffer0);
                             p.5.reset_interrupt_flags(0xFF);
                             let can_res = can_parser(&RX_NV_CONFIG, can_state, &rx_frame);
                             can_state = can_res.0;
                             match can_res.0 {
                                 CanState::WaitingOfTransfer => {
                                     rprintln!("ET");
                                     State::Error
                                 }
                                 CanState::StartOfTransfer => {
                                     rprintln!("ST");
                                     nv_config_range.erase(p.1.borrow_mut()).unwrap();
                                     p.5.send(RawFrameRef{
                                         id: RX_NV_CONFIG,
                                         data: &[],
                                     }, McpPriority::LowIntermediate);
                                     State::WaitingNewCommandEndless
                                 }
                                 CanState::DataTransfer => {
                                     rprintln!("T");
                                     nv_config_range.write(p.1.borrow_mut(), can_res.1).unwrap();
                                     p.5.send(RawFrameRef{
                                         id: RX_NV_CONFIG,
                                         data: &[],
                                     }, McpPriority::LowIntermediate);
                                     State::WaitingNewCommandEndless
                                 }
                                 CanState::EndOfTransfer => {
                                     rprintln!("EndT");
                                     p.5.send(RawFrameRef{
                                         id: RX_NV_CONFIG,
                                         data: &[],
                                     }, McpPriority::LowIntermediate);
                                     nv_config_range.flush(p.1.borrow_mut());
                                     State::Reboot
                                 }
                             }
                         }else if can_state == CanState::WaitingOfTransfer{
                             p.5.send(RawFrameRef{
                                 id: RX_NV_CONFIG,
                                 data: &[],
                             }, McpPriority::LowIntermediate);
                             State::Error
                         }
                         else{
                             State::WaitingNewCommandEndless
                         }
                     }
                     State::CheckForFirmware | State::CheckFirmwareValidity => {
                         if p.5.interrupt_flags().rx0if_is_set() {
                             let rx_frame = p.5.receive(McpReceiveBuffer::Buffer0);
                             p.5.reset_interrupt_flags(0xFF);
                             let can_res = can_parser(&RX_NEW_FIRMWARE, can_state, &rx_frame);
                             can_state = can_res.0;
                             match can_res.0 {
                                 CanState::WaitingOfTransfer => {
                                     rprintln!("ET");
                                     State::Error
                                 }
                                 CanState::StartOfTransfer => {
                                     rprintln!("ST");
                                     firmware_range.erase(p.1.borrow_mut()).unwrap();
                                     p.5.send(RawFrameRef{
                                         id: RX_NEW_FIRMWARE,
                                         data: &[],
                                     }, McpPriority::LowIntermediate);
                                     State::WaitingNewCommandEndless
                                 }
                                 CanState::DataTransfer => {
                                     rprintln!("T");
                                     firmware_range.write(p.1.borrow_mut(), can_res.1).unwrap();
                                     p.5.send(RawFrameRef{
                                         id: RX_NEW_FIRMWARE,
                                         data: &[],
                                     }, McpPriority::LowIntermediate);
                                     State::WaitingNewCommandEndless
                                 }
                                 CanState::EndOfTransfer => {
                                     rprintln!("EndT");
                                     firmware_range.flush(p.1.borrow_mut());
                                     p.5.send(RawFrameRef{
                                         id: RX_NEW_FIRMWARE,
                                         data: &[],
                                     }, McpPriority::LowIntermediate);
                                     State::Reboot
                                 }
                             }
                         }else if can_state == CanState::WaitingOfTransfer{
                             p.5.send(RawFrameRef{
                                 id: RX_NEW_FIRMWARE,
                                 data: &[],
                             }, McpPriority::LowIntermediate).ok();
                             State::Error
                         }
                         else{
                             State::WaitingNewCommandEndless
                         }
                     }
                     _ =>{
                         prev_state = State::WaitingNewCommandEndless;
                         State::Error
                     }
                 }
             }
             State::Error => {
                 let state = match prev_state {
                     State::CheckNVConfig => {
                         blink_led(&mut p.0, 1);
                        State::WaitingNewCommandEndless
                     }
                     State::CheckBootloaderValidity => {
                         blink_led(&mut p.0, 2);
                         //State::Error
                         State::WaitingNewCommandEndless
                     }
                     State::CheckForFirmware => {
                         blink_led(&mut p.0, 3);
                         State::WaitingNewCommandEndless
                     }
                     State::CheckFirmwareValidity => {
                         blink_led(&mut p.0, 4);
                         State::WaitingNewCommandEndless
                     }
                     _ => {
                         blink_led(&mut p.0, 5);
                         State::Error
                     }
                 };
                 asm::delay(8_000_000);
                 state
             }
             State::Boot => {
                 rprintln!("Boot");
                 #[cfg(not(feature = "cortex-m0"))]
                     unsafe {
                     let rv: usize = *(firmware_range.get_start_address() + 0x04 as *const usize);
                     scb.vtor.write(firmware_range.get_start_address());
                     let function = core::mem::transmute::<usize, extern "C" fn() -> !>(rv);
                     function();
                 }
                 #[cfg(feature = "cortex-m0")]
                 unsafe {
                     cortex_m::interrupt::disable();
                        core::ptr::copy_nonoverlapping::<u32>(firmware_range.get_start_address() as *const u32,0x2000_0000 as *mut u32, 48);
                        p.6.cfgr1.modify(|_,w|w.mem_mode().sram());
                     cortex_m::interrupt::enable();

                     let rv: usize = *((firmware_range.get_start_address() + 4) as *const usize);

                     peripherals::deinit_peripherals();
                     let function = core::mem::transmute::<usize, extern "C" fn() -> !>(rv);
                     function();
                 };
             }
             State::Reboot => {
                 let aircr = 0xE000ED0C as *mut u32;
                 unsafe { *aircr = (0x5FA << 16) | (1 << 2) };
                 State::CheckNVConfig
             }
         }
    }
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    //rprintln!("{:#?}", ef);
    loop {
        asm::delay(10_000)
    }
}

#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    //rprintln!("Panic: {:?}", _info);
    loop {
        atomic::compiler_fence(Ordering::SeqCst);
    }
}
