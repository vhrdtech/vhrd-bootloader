use crate::peripherals::{CAN_RX_PIN,CAN_TX_PIN};
use stm32f0xx_hal::{
    stm32::{CAN, RCC},
    prelude::*,
};
use cortex_m::asm::delay;
use rtt_target::rprintln;
use stm32f0xx_hal::time::Hertz;

fn update_reg_by_bit_pos(src: u32, pos: u32, val: u32) -> u32{
    let tmp = (src >> pos) | val;
    ((tmp << pos) + src)
}

pub struct CanFrame{
    pub id: u32,
    pub id_type: IdType,
    pub data_len: usize,
    pub data: [u8; 8]
}

pub enum CanMode{
    NormalMode = 0,
    SilentMode = 1,
    LoopBackMode = 2,
    LoopBackSilentMode = 3
}

pub enum IdType{
    Standard = 0,
    Extended = 1
}

pub enum AutomaticRetransmission{
    Enabled = 0,
    Disabled = 1
}

pub enum AutomaticBussOffManagement{
    Enabled = 0,
    Disabled = 1
}

pub enum AutomaticWakeUpMode{
    Enabled = 0,
    Disabled = 1
}

#[derive(Debug)]
pub enum CanError{
    ReadTimeout,
    BussOff,
    ErrorPassive,
    Warning,
    NoError,
}

pub enum BitRate{
    _1Mbs = 0,
    _500Kbs = 1,
    _100Kbs = 2
}

pub enum FilterMode{
    MaskMode = 0,
    ListMode = 1
}

pub enum FilterScaleConfiguration{
    _32BitSingleConfig = 0,
    _16BitDualConfig = 1
}

pub struct Filter{
    pub(crate) mode: FilterMode,
    pub(crate) scale_config: FilterScaleConfiguration,
    pub(crate) id_or_mask: u32,
    pub(crate) enable: bool,
    pub(crate) id_type: IdType,
    pub(crate) rtr: bool
}

pub struct CanParams{
    pub(crate) work_mode: CanMode,
    pub(crate) automatic_retransmission: AutomaticRetransmission,
    pub(crate) automatic_busoff_management : AutomaticBussOffManagement,
    pub(crate) auto_wake_up : AutomaticWakeUpMode,
    pub(crate) pclk_Hz: Hertz,
    pub(crate) bitrate: BitRate
}

pub struct Can {
    tx_pin : CAN_TX_PIN,
    rx_pin : CAN_RX_PIN,
    can_reg : CAN,
    available_tx_mailbox: [bool; 3],
    pub(crate) receive_flag: bool
}

pub type Result<T> = core::result::Result<T, CanError>;

impl Can{
    pub fn new(tx_pin: CAN_TX_PIN, rx_pin: CAN_RX_PIN, can_reg: CAN, can_params: CanParams, filters: &[Filter])->Can{
        let mut cfg_can_timeout = 1_000_000_u32;

        can_reg.mcr.modify(|_,w| w.inrq().set_bit());
        while can_reg.msr.read().inak().bit_is_clear() {
            cfg_can_timeout = cfg_can_timeout - 1;
            if cfg_can_timeout == 0 {
                rprintln!("CAN: inrq enable fail\n");
                break;
            }
        }
        can_reg.mcr.modify(|_, w| w.sleep().clear_bit());
        cfg_can_timeout = 1_000_000_u32;
        while can_reg.msr.read().slak().bit_is_set() {
            cfg_can_timeout = cfg_can_timeout - 1;
            if cfg_can_timeout == 0 {
                rprintln!("CAN: sleep fail\n");
                break;
            }
        }

        match can_params.automatic_busoff_management{
            AutomaticBussOffManagement::Enabled => can_reg.mcr.modify(|_, w| w.abom().set_bit()),
            AutomaticBussOffManagement::Disabled => can_reg.mcr.modify(|_, w| w.abom().clear_bit())
        }

        match can_params.automatic_retransmission{
            AutomaticRetransmission::Enabled => can_reg.mcr.modify(|_, w| w.nart().clear_bit()),
            AutomaticRetransmission::Disabled => can_reg.mcr.modify(|_, w| w.nart().set_bit())
        }

        match can_params.auto_wake_up{
            AutomaticWakeUpMode::Enabled => can_reg.mcr.modify(|_, w| w.awum().set_bit()),
            AutomaticWakeUpMode::Disabled => can_reg.mcr.modify(|_, w| w.awum().clear_bit())
        }

        match can_params.work_mode{
            CanMode::LoopBackMode => can_reg.btr.modify(|_,w| w.lbkm().enabled().silm().normal()),
            CanMode::LoopBackSilentMode => can_reg.btr.modify(|_,w| w.lbkm().enabled().silm().silent()),
            CanMode::SilentMode => can_reg.btr.modify(|_,w| w.lbkm().disabled().silm().silent()),
            CanMode::NormalMode => can_reg.btr.modify(|_,w| w.lbkm().disabled().silm().normal())
        }
        match can_params.bitrate{
            BitRate::_1Mbs => can_reg.btr.modify(|_,w| unsafe{w.sjw().bits(1).ts1().bits(2).ts2().bits(3).brp().bits(0)}),
            BitRate::_500Kbs => can_reg.btr.modify(|_,w| unsafe{w.sjw().bits(3).ts1().bits(4).ts2().bits(1).brp().bits(0)}),
            BitRate::_100Kbs => can_reg.btr.modify(|_,w| unsafe{w.sjw().bits(4).ts1().bits(3).ts2().bits(1).brp().bits(0)}),
        }

        let brp = ((can_params.pclk_Hz.0 / Hertz(8_000_000).0) as u16 - 1) as u16;
        can_reg.btr.modify(|_,w| unsafe{w.brp().bits(brp)});

        /*can_reg.ier.modify(|_,w|w.errie().disabled());
        can_reg.ier.modify(|_,w|w.bofie().disabled());
        can_reg.ier.modify(|_,w|w.epvie().disabled());
        can_reg.ier.modify(|_,w|w.ewgie().disabled());
        can_reg.ier.modify(|_,w|w.lecie().disabled());
        can_reg.ier.modify(|_,w|w.fmpie0().enabled());
        can_reg.ier.modify(|_,w|w.fmpie1().enabled());
        can_reg.ier.modify(|_,w|w.tmeie().enabled());*/

        can_reg.mcr.modify(|_,w| w.inrq().clear_bit());
        cfg_can_timeout = 1_000_000_u32;
        while can_reg.msr.read().inak().bit_is_set() {
            cfg_can_timeout = cfg_can_timeout - 1;
            if cfg_can_timeout == 0 {
                rprintln!("CAN: inrq dis fail\n");
                break;
            }
        }

        if filters.len() > 0 && filters.len() < 14{
            for i in 0..filters.len() {
                can_reg.fmr.modify(|_, w| w.finit().set_bit());
                can_reg.fa1r.modify(|r, w| unsafe{w.bits(update_reg_by_bit_pos(r.bits(), i as u32, (filters[i].enable as u32) & 0xFFFFFFFE))});

                match filters[i].scale_config {
                  FilterScaleConfiguration::_16BitDualConfig => can_reg.fs1r.modify(|r, w| unsafe{w.bits(update_reg_by_bit_pos(r.bits(), i as u32, 0x00))}),
                  FilterScaleConfiguration::_32BitSingleConfig => can_reg.fs1r.modify(|r, w| unsafe{w.bits(update_reg_by_bit_pos(r.bits(), i as u32, 0x01))})
                }

                match filters[i].mode{
                    FilterMode::MaskMode => can_reg.fm1r.modify(|r, w| unsafe{w.bits(update_reg_by_bit_pos(r.bits(), i as u32, 0x00))}),
                    FilterMode::ListMode => can_reg.fm1r.modify(|r, w| unsafe{w.bits(update_reg_by_bit_pos(r.bits(), i as u32, 0x01))})
                }

                let mut id_or_mask: u32 = 0;

                match filters[i].id_type{
                    IdType::Standard => id_or_mask = (filters[i].id_or_mask << 3) | (0x00 << 2) | ((filters[i].rtr as u32) << 1),
                    IdType::Extended => id_or_mask = (filters[i].id_or_mask << 3) | (0x01 << 2) | ((filters[i].rtr as u32) << 1)
                }

                can_reg.fb[i].fr1.modify(|_, w| unsafe { w.bits(id_or_mask)});
                can_reg.fb[i].fr2.modify(|_, w| unsafe { w.bits(id_or_mask)});

                can_reg.fa1r.modify(|r, w| unsafe{w.bits(update_reg_by_bit_pos(r.bits(), i as u32, (filters[i].enable as u32)))});

            }

            can_reg.fmr.modify(|_, w| w.finit().clear_bit());
        }
        else if filters.len() == 0{
            can_reg.fmr.modify(|_, w| w.finit().set_bit());
            can_reg.fa1r.modify(|_, w|w.fact0().clear_bit());
            can_reg.fs1r.modify(|_, w| w.fsc0().clear_bit());
            can_reg.fm1r.modify(|_, w| w.fbm0().clear_bit());
            can_reg.fb[0].fr1.modify(|_, w| unsafe { w.bits(0x00000000)});
            can_reg.fb[0].fr2.modify(|_, w| unsafe { w.bits(0x00000000)});
            can_reg.fa1r.modify(|_, w|w.fact0().set_bit());
            can_reg.fmr.modify(|_, w| w.finit().clear_bit());
        }


        ////////////////////////////////////////

        Can{
            tx_pin,
            rx_pin,
            can_reg,
            available_tx_mailbox:[true, true, true],
            receive_flag: false
        }
    }

    pub fn write_to_mailbox(&mut self, id_type: IdType, transmit_id: u32, data: &[u8]){
        for i in 0..self.available_tx_mailbox.len() {
            match self.available_tx_mailbox[i]{
                false => continue,
                true => {
                    self.can_reg.tx[i].tir.modify(|_, w| w.rtr().data());
                    match id_type {
                        IdType::Standard => {
                            self.can_reg.tx[i].tir.modify(|_, w| w.ide().standard());
                            self.can_reg.tx[i].tir.modify(|_, w| unsafe { w.stid().bits(transmit_id as u16) });
                        },
                        IdType::Extended => {
                            self.can_reg.tx[i].tir.modify(|_, w| w.ide().extended());
                            self.can_reg.tx[i].tir.modify(|_, w| unsafe { w.stid().bits((transmit_id >> 18) as u16) });
                            self.can_reg.tx[i].tir.modify(|_, w| unsafe { w.exid().bits(transmit_id) });
                        }
                    }
                    let dlc = data.len() as u8;
                    self.can_reg.tx[i].tdtr.write(|w| unsafe { w.dlc().bits(dlc) });
                    if dlc > 4 {
                        let data = data.as_ptr() as *const _ as *const u64;
                        self.can_reg.tx[i].tdhr.write(|w| unsafe { w.bits((*data >> 32) as u32) });
                        self.can_reg.tx[i].tdlr.write(|w| unsafe { w.bits(*data as u32) });
                    } else if dlc > 0 {
                        let data = data.as_ptr() as *const _ as *const u32;
                        self.can_reg.tx[i].tdlr.write(|w| unsafe { w.bits(*data as u32) });
                    }
                    self.can_reg.tx[i].tir.modify(|_, w| w.txrq().set_bit());
                    self.available_tx_mailbox[i] = false;
                    break;
                }
            }

        }
    }

    pub fn read_blocking_timeout(&mut self) -> Result<CanFrame> {
        let mut cnt: u16 = 0;
        while cnt < u16::max_value() - 1 {
            let fifo_rx_pending: [u8; 2] = [self.can_reg.rfr[0].read().fmp().bits(), self.can_reg.rfr[1].read().fmp().bits()];
            for i in 0..=1 {
                if fifo_rx_pending[i] != 0b00 {
                    let mut rx_id: u32 = 0;
                    let mut id_type: IdType = IdType::Extended;
                    match self.can_reg.rx[i].rir.read().ide().bits()
                    {
                        true => { rx_id = self.can_reg.rx[i].rir.read().bits() >> 3; },
                        false => {
                            rx_id = self.can_reg.rx[i].rir.read().stid().bits() as u32;
                            id_type = IdType::Standard;
                        }
                    }
                    let rx_dlc = self.can_reg.rx[i].rdtr.read().dlc().bits() as usize;
                    let data_raw: u64 = (self.can_reg.rx[i].rdhr.read().bits() as u64) << 32 | (self.can_reg.rx[i].rdlr.read().bits()) as u64;
                    self.can_reg.rfr[i].modify(|_, w| w.rfom().release());
                    self.receive_flag = true;
                    return Ok(
                        CanFrame {
                            id: rx_id,
                            id_type: id_type,
                            data_len: rx_dlc,
                            data: data_raw.to_le_bytes()
                        }
                    );
                }
            }
            cnt += 1;
        }
        return Err(CanError::ReadTimeout);
    }
}