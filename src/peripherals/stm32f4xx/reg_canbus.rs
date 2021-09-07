
use crate::config::*;
pub use bxcan::Data as BxData;

pub fn reg_can_init(
    can_peripheral: stm32f4xx_hal::stm32::CAN1,
    can_tx: CanTx,
    can_rx: CanRx,
) -> CanStmInstance {
    use bxcan::filter::{BankConfig, Mask32};

    let can = stm32f4xx_hal::can::Can::new(can_peripheral, (can_tx, can_rx));
    let mut can = bxcan::Can::new(can);

    //use stm32f0xx_hal::can::bxcan::Interrupt;
    // can.enable_interrupt(Interrupt::Fifo0MessagePending);
    // can.enable_interrupt(Interrupt::Fifo1MessagePending);
    //can.enable_interrupt(Interrupt::Fifo0Full);
    // can.enable_interrupt(Interrupt::Fifo1Full); // endless interrupt
    //can.enable_interrupt(Interrupt::TransmitMailboxEmpty);
    can
}

pub fn reg_can_configure(can: &mut CanStmInstance, cfg: &bxcan::filter::BankConfig){
    use bxcan::filter::{BankConfig, Mask32};
    can.modify_config()
        .set_loopback(false)
        .set_silent(false)
        .set_bit_timing(0x00050000);
    {
        let mut filters = can.modify_filters();
        filters.enable_bank(0, *cfg);
    }
    can.enable().ok();
}