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
use vhrdcan::{FrameId, FrameRef};
use core::borrow::BorrowMut;
use stm32f0xx_hal::pac::syscfg::cfgr1::MEM_MODE_A::SRAM;
use crc::{Crc, Algorithm, CRC_32_AUTOSAR};
use mcp25625::{McpPriority, McpReceiveBuffer, TxBufferChoice};
use crate::CanState::WaitingOfTransfer;
use crate::State::CheckNVConfig;

use uavcan_llr::types::{TransferId, CanId, NodeId, SubjectId, Priority, TransferKind, Service, ServiceId};
use core::convert::TryFrom;
use uavcan_llr::tailbyte::TailByte;


mod ticks_time;

fn blink_led(led: &mut UsrLedPin, amount_of_blinks: u8){
    for _ in 0..amount_of_blinks{
        led.set_high().ok();
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
    WaitingCommand,
    Boot,
    Reboot,
    Error
}
const RX_NV_CONFIG: FrameId = FrameId::new_extended(0x0000_0001).unwrap();
const RX_NEW_FIRMWARE: FrameId = FrameId::new_extended(0x0000_0010).unwrap();
const WAITING_CMD: FrameId = FrameId::new_extended(0x0000_0100).unwrap();

const BOOTLOADER_KEY: u32 = 0xDEADBEEF;

const FLASHER_NODE_ID: NodeId = NodeId::new(127).unwrap();

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
enum CanState{
    WaitingOfTransfer,
    StartOfTransfer,
    DataTransfer,
    EndOfTransfer
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
enum DataTransferState{
    StartOfTransfer,
    DataTransfer,
    EndOfTransfer
}
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
enum BootloaderReadOptions{
    Config,
    Firmware,
    RawAddress,
    Bootloader,
}

enum BootloaderWriteOptions{
    Config,
    Firmware
}

const READ_SERVICE: ServiceId = ServiceId::new(0).unwrap();
const WRITE_CONFIG_SERVICE: ServiceId = ServiceId::new(1).unwrap();
const WRITE_FIRMWARE_SERVICE: ServiceId = ServiceId::new(2).unwrap();
const UNLOCK_BOOTLOADER: ServiceId = ServiceId::new(3).unwrap();

const ERROR_MSG: SubjectId = SubjectId::new(0).unwrap();
const HARD_BIT_MSG: SubjectId = SubjectId::new(10).unwrap();
const REBOOT_MSG: SubjectId = SubjectId::new(2).unwrap();
const BOOT_MSG: SubjectId = SubjectId::new(1).unwrap();

const READ_CONFIG_CMD: u8 = 0x01;
const READ_FIRMWARE_CMD: u8 = 0x02;
const READ_BOOTLOADER_CMD: u8 = 0x03;

const DEFAULT_NODE_ID: NodeId = NodeId::new(0).unwrap();

enum CommandError{
    WrongServiceId,
    NoCanData,
}

enum CommandEvent<'a>{
    Read(BootloaderReadOptions, &'a [u8]),
    Write(BootloaderWriteOptions, DataTransferState, &'a [u8]),
    UnlockBootloader,
    Error(CommandError),
}

fn can_transmit(can_iface: &mut Mcp25625Instance, frame_id: FrameId, data: &[u8]){
    //rprintln!("Can_tx");
    if data.is_empty(){
        can_iface.send(
            FrameRef {
                id: frame_id,
                data: &[]
            },
            TxBufferChoice::Any,
            McpPriority::Low
        ).ok();
    }
    else{
        let chunks = data.chunks_exact(7);
        let reminder = chunks.remainder();
        let tr_id = TransferId::new(0).unwrap();
        let mut tail_byte = TailByte::multi_frame_transfer(tr_id, chunks.len() + if reminder.is_empty(){ 0usize } else { 1usize });
        let mut can_data = [0u8;8];
        //rprintln!("Before send");
        for msg in chunks.into_iter(){
            can_data[0..7].copy_from_slice(&msg[0..7]);
            can_data[7] = u8::try_from(tail_byte.next().unwrap()).unwrap();
            can_iface.send(
                FrameRef {
                    id: frame_id,
                    data: can_data.as_slice()
                },
                TxBufferChoice::Any,
                McpPriority::Low
            ).ok();
        }
        if !reminder.is_empty(){
            can_data[0..reminder.len()].copy_from_slice(&reminder[0..reminder.len()]);
            can_data[reminder.len()] = u8::try_from(tail_byte.next().unwrap()).unwrap();
            can_iface.send(
                FrameRef {
                    id: frame_id,
                    data: &can_data[0..reminder.len() + 1]
                },
                TxBufferChoice::Any,
                McpPriority::Low
            ).ok();
        }
    }
}

enum CanWorkerResult{
    FirmwareReceived,
    NvConfigReceived,
    None
}

fn can_worker<'a, R: FnMut(CommandEvent, &NodeId, &NodeId)->(FrameId, &'a [u8])>(can_iface: &mut Mcp25625Instance, src_node_id: &NodeId, mut r: R) -> Option<CanWorkerResult> {
    let mut cnt = 0u16;
    while cnt < 4000{
        cnt += 1;
        if can_iface.interrupt_flags().rx0if_is_set() {
            let rx_frame = can_iface.receive(McpReceiveBuffer::Buffer0);
            let uavcan_id = CanId::try_from(rx_frame.id).unwrap();
            if uavcan_id.source_node_id == FLASHER_NODE_ID {
                match uavcan_id.transfer_kind {
                    TransferKind::Message(m) => {}
                    TransferKind::Service(s) => {
                        if s.is_request {
                            let mut res = CanWorkerResult::None;
                            if rx_frame.data().len() != 0 {
                                let tail_byte = TailByte::from(*rx_frame.data().last().unwrap());
                                let data_transfer_state =
                                    if tail_byte.start_of_transfer == true && tail_byte.end_of_transfer == false && tail_byte.toggle_bit == true { DataTransferState::StartOfTransfer } else if tail_byte.start_of_transfer == false && tail_byte.end_of_transfer == true { DataTransferState::EndOfTransfer } else { DataTransferState::DataTransfer };

                                let res = match s.service_id {
                                    READ_SERVICE => {
                                        let read_option = if rx_frame.data[0] == READ_CONFIG_CMD && rx_frame.data().len() == 2 { BootloaderReadOptions::Config } else if rx_frame.data[0] == READ_FIRMWARE_CMD && rx_frame.data().len() == 2 { BootloaderReadOptions::Firmware } else if rx_frame.data[0] == READ_BOOTLOADER_CMD && rx_frame.data().len() == 2 { BootloaderReadOptions::Bootloader } else { BootloaderReadOptions::RawAddress };

                                        //rprintln!("{:?}",read_option);
                                        r(CommandEvent::Read(read_option, &rx_frame.data()[0..rx_frame.data().len()]), src_node_id, &s.destination_node_id)
                                    }
                                    WRITE_CONFIG_SERVICE => {
                                        r(CommandEvent::Write(BootloaderWriteOptions::Config, data_transfer_state, &rx_frame.data()[0..(rx_frame.data().len() - 1)]), src_node_id, &s.destination_node_id)
                                    }
                                    WRITE_FIRMWARE_SERVICE => {
                                        if data_transfer_state == DataTransferState::EndOfTransfer{
                                            res = CanWorkerResult::FirmwareReceived;
                                        }
                                        r(CommandEvent::Write(BootloaderWriteOptions::Firmware, data_transfer_state, &rx_frame.data()[0..(rx_frame.data().len() - 1)]), src_node_id, &s.destination_node_id)
                                    }
                                    UNLOCK_BOOTLOADER => {
                                        r(CommandEvent::UnlockBootloader, src_node_id, &s.destination_node_id)
                                    }
                                    _ => {
                                        r(CommandEvent::Error(CommandError::WrongServiceId), src_node_id, &s.destination_node_id)
                                    }
                                };
                                //rprintln!("tx_sl");
                                can_transmit(can_iface, res.0, res.1);
                            } else {
                                let res = r(CommandEvent::Error(CommandError::NoCanData), src_node_id, &s.destination_node_id);
                                can_transmit(can_iface, res.0, res.1);
                            }
                            return Some(res);
                        }
                    }
                }
            }
        }
    }
    None
}


#[entry]
fn main() -> ! {
    rtt_init_print!();
    let mut state = State::CheckNVConfig;
    let mut prev_state = state;
    let mut p = peripherals::setup_peripherals();
    let mut flash_regs = p.1;
    let mut nv_config_range = match FlashWriter::new(NV_CONFIG_START_ADDR as u32..(NV_CONFIG_START_ADDR as u32 + SIZE_OF_NVCONFIG as u32)){
        Ok(f) => {f}
        Err(_) => { panic!() }
    };
    let mut firmware_range = match FlashWriter::new(
        (NV_CONFIG_START_ADDR as u32 + SIZE_OF_NVCONFIG as u32)
            ..(0x0800_0000 + flash_size_bytes())){
        Ok(f) => {f}
        Err(_) => { panic!() }
    };

    let nv_config = NVConfig::get();
    let mut pass_time_us = 0u64;
    let mut timeout_limit = 10_000u16;
    let mut can_state = CanState::WaitingOfTransfer;
    let mut node_id = DEFAULT_NODE_ID;
    p.3.count_time_from_now();
    loop {
       // rprintln!("{:?}",state);
         state = match state{
             State::CheckNVConfig => {
                 prev_state = State::CheckNVConfig;
                 match get_crc(flash_read_slice::<u8>(NV_CONFIG_START_ADDR as u32 + 8u32, SIZE_OF_NVCONFIG - 8)) == nv_config.config_crc{
                     true => {
                         node_id = unsafe{NodeId::new_unchecked(nv_config.board_config.uavcan_node_id)};
                         State::CheckBootloaderValidity
                     }
                     false => {
                         //rprintln!("0x{:08x} != 0x{:08x}", nv_config.config_crc, get_crc(flash_read_slice::<u8>(NV_CONFIG_START_ADDR as u32 + 8u32, SIZE_OF_NVCONFIG - 8)));
                         State::Error
                     }
                 }
             }
             State::CheckBootloaderValidity => {
                 prev_state = State::CheckBootloaderValidity;
                 if nv_config.board_config.bootloader_size + 0x0800_0000 <= NV_CONFIG_START_ADDR as u32 && nv_config.board_config.bootloader_size != 0 {
                     match get_crc(flash_read_slice::<u8>(0x0800_0000, nv_config.board_config.bootloader_size as usize)) == nv_config.board_config.bootloader_crc {
                         true => { State::CheckForFirmware }
                         false => {
                             //rprintln!("0x{:08x} != 0x{:08x}", nv_config.board_config.bootloader_crc, get_crc(flash_read_slice::<u8>(0x0800_0000, nv_config.board_config.bootloader_size as usize)));
                             State::Error
                         }
                     }
                 }
                 else {
                     //rprintln!("0x{:08x} != 0x{:08x}", nv_config.board_config.bootloader_crc, get_crc(flash_read_slice::<u8>(0x0800_0000, nv_config.board_config.bootloader_size as usize)));
                     State::Error
                 }
             }
             State::CheckForFirmware => {
                 prev_state = State::CheckForFirmware;
                 if nv_config.board_config.fw_size > 0 && nv_config.board_config.fw_size < (flash_size_bytes() - SIZE_OF_NVCONFIG as u32 - 10u32.kb()) {
                     State::CheckFirmwareValidity
                 } // 10240 - max bootloader size
                 else {
                     //rprintln!("Firmware_size:{}", nv_config.board_config.fw_size);
                     State::Error
                 }
             }
             State::CheckFirmwareValidity => {
                 prev_state = State::CheckFirmwareValidity;
                 match get_crc(flash_read_slice::<u8>(firmware_range.get_start_address(), nv_config.board_config.fw_size as usize)) == nv_config.board_config.fw_crc {
                     true => {
                         State::WaitingNewCommandTimeout
                     }
                     false => {
                         //rprintln!("0x{:08x} != 0x{:08x}", nv_config.board_config.fw_crc, get_crc(flash_read_slice::<u8>(firmware_range.get_start_address(), nv_config.board_config.fw_size as usize)));
                         State::Error
                     }
                 }
             }
             State::WaitingNewCommandTimeout => {
                 timeout_limit = nv_config.board_config.bootloader_timeout_ms;
                 //rprintln!("to: {}", timeout_limit);
                 prev_state = State::WaitingNewCommandTimeout;
                 //p.3.count_time_from_now();
                 State::WaitingCommand
             }
             State::WaitingNewCommandEndless => {
                 timeout_limit = 20_000;
                 prev_state = State::WaitingNewCommandEndless;
                 //p.3.count_time_from_now();
                 State::WaitingCommand
             }
             State::WaitingCommand => {
                 match timeout_limit <= (pass_time_us / 1000) as u16 {
                     true => {
                         if prev_state == State::WaitingNewCommandTimeout{
                             State::Boot
                         }else{
                             State::Reboot
                         } }
                     false => {
                         match can_worker(p.5.borrow_mut(), node_id.borrow_mut() , |command, src_node_id, dst_node_id|{
                             match command{
                                 CommandEvent::Read(br, data) => {
                                     //rprintln!("in read");
                                     let res_slice = match br{
                                         BootloaderReadOptions::Config => { flash_read_slice::<u8>(NV_CONFIG_START_ADDR as u32, SIZE_OF_NVCONFIG) }
                                         BootloaderReadOptions::Firmware => { flash_read_slice::<u8>(firmware_range.get_start_address(), (flash_size_bytes() - (firmware_range.get_start_address() - 0x0800_0000)) as usize) }
                                         BootloaderReadOptions::Bootloader => { flash_read_slice::<u8>(0x0800_0000u32, 10.kb()) }
                                         //BootloaderReadOptions::RawAddress => { core::ptr::slice_from_raw_parts() }
                                         _ => {&[]}
                                     };
                                     //rprintln!("Send res");
                                     (FrameId::new_extended(unsafe{CanId::new_service_kind(*src_node_id,  *dst_node_id, READ_SERVICE,false, Priority::High).into()}).unwrap(), res_slice)
                                 }
                                 CommandEvent::Write(bw, tr, data) => {
                                     let (mut flash_region, service_id) = match bw{
                                         BootloaderWriteOptions::Config => { (nv_config_range.borrow_mut(), WRITE_CONFIG_SERVICE) }
                                         BootloaderWriteOptions::Firmware => { (firmware_range.borrow_mut(), WRITE_FIRMWARE_SERVICE) }
                                     };
                                     match tr{
                                         DataTransferState::StartOfTransfer => {
                                             flash_region.erase(flash_regs.borrow_mut()).ok();
                                             flash_region.write(flash_regs.borrow_mut(), data).ok();
                                         }
                                         DataTransferState::DataTransfer => {
                                             flash_region.write(flash_regs.borrow_mut(), data).ok();
                                         }
                                         DataTransferState::EndOfTransfer => {
                                             flash_region.write(flash_regs.borrow_mut(), data).ok();
                                             flash_region.flush(flash_regs.borrow_mut()).ok();
                                             //rprintln!("Transfer DOne!!!");
                                         }
                                     }
                                     (FrameId::new_extended(unsafe{CanId::new_service_kind(*src_node_id,  *dst_node_id, service_id,false, Priority::High).into()}).unwrap(), &[])
                                 }
                                 //CommandEvent::UnlockBootloader => {}
                                 CommandEvent::Error(er) => {
                                     (FrameId::new_extended(unsafe{CanId::new_message_kind(*src_node_id, ERROR_MSG, false, Priority::High).into()}).unwrap(), &[])
                                 }
                                 _=>{
                                     (FrameId::new_extended(unsafe{CanId::new_message_kind(*src_node_id, ERROR_MSG, false, Priority::High).into()}).unwrap(), &[])
                                 }
                             }
                         })
                         {
                             None => {
                                 can_transmit(p.5.borrow_mut(), (FrameId::new_extended(unsafe{CanId::new_message_kind(node_id, HARD_BIT_MSG, false, Priority::High).into()}).unwrap()), &[]);
                                 pass_time_us += p.3.get_delta_time_us() as u64;
                                 State::Error
                             }
                             Some(r) => {
                                 match r{
                                     CanWorkerResult::FirmwareReceived => {
                                         State::Reboot
                                     }
                                     _ => {
                                        pass_time_us = 0;
                                        State::WaitingCommand
                                     }
                                 }

                             }
                         }
                     }
                 }
             }
             State::Error => {
                 let state = match prev_state {
                     State::WaitingNewCommandTimeout | State::WaitingNewCommandEndless => {
                         blink_led(&mut p.0, 1);
                         //asm::delay(100_000);
                         prev_state
                     }
                     State::CheckNVConfig => {
                         blink_led(&mut p.0, 2);
                         asm::delay(8_000_000);
                        State::WaitingNewCommandEndless
                     }
                     State::CheckBootloaderValidity => {
                         blink_led(&mut p.0, 3);
                         asm::delay(8_000_000);
                         //State::Error
                         State::WaitingNewCommandEndless
                     }
                     State::CheckForFirmware => {
                         blink_led(&mut p.0, 4);
                         asm::delay(8_000_000);
                         State::WaitingNewCommandEndless
                     }
                     State::CheckFirmwareValidity => {
                         blink_led(&mut p.0, 5);
                         asm::delay(8_000_000);
                         State::WaitingNewCommandEndless
                     }
                     _ => {
                         blink_led(&mut p.0, 6);
                         asm::delay(8_000_000);
                         State::Error
                     }
                 };
                 state
             }
             State::Boot => {
                 unsafe{ can_transmit(p.5.borrow_mut(), FrameId::new_extended(CanId::new_message_kind(node_id, BOOT_MSG, false, Priority::High).into()).unwrap(), &[])};
                 //rprintln!("Boot");
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

                 state = CheckNVConfig;
                 unsafe{ can_transmit(p.5.borrow_mut(), FrameId::new_extended(CanId::new_message_kind(node_id, REBOOT_MSG, false, Priority::High).into()).unwrap(), &[])};
                 let aircr = 0xE000ED0C as *mut u32;
                 unsafe { *aircr = (0x5FA << 16) | (1 << 2) };
                 State::CheckNVConfig
             }
         }
    }
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
   // rprintln!("{:#?}", ef);
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