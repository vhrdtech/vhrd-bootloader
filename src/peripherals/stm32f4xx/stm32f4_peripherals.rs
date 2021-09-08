
#[cfg(feature = "reg-can")]
use crate::reg_canbus::*;
#[cfg(feature = "reg-can")]
pub type CanInstance = CanStmInstance;

use stm32f4xx_hal::stm32::{FLASH, SCB, SYSCFG};
use crate::ticks_time::TicksTime;
use stm32f4xx_hal::delay::Delay;
use stm32f4xx_hal::stm32;
use cortex_m::interrupt::{CriticalSection};
use stm32f4xx_hal::time::U32Ext;
use stm32f4xx_hal::rcc::RccExt;
use embedded_hal::digital::v2::OutputPin;
use crate::peripherals::stm32f4xx::CanStmInstance;
use stm32f4xx_hal::gpio::gpioc::PC6;
use stm32f4xx_hal::gpio::{Output, PushPull, GpioExt};


pub type UsrLedPin = PC6<Output<PushPull>>;

pub fn setup_peripherals() -> (UsrLedPin, FLASH, SCB, TicksTime, Delay, CanInstance, SYSCFG) {
    let mut dp= match stm32::Peripherals::take(){
        None => {panic!()}
        Some(dp) => { dp }
    };
    let cs = unsafe {CriticalSection::new()};
    let mut rcc = dp.RCC;
    let mut cp = match stm32::CorePeripherals::take(){
        None => {panic!()}
        Some(cp) => {cp}
    };

    rcc.apb1enr.modify(|_, w| w.can1en().enabled()); // can time enb
    //rcc.apb2enr.modify(|_, w| w.syscfgen().enabled());

    let mut rcc = rcc.constrain();
    /*let mut clock = rcc
        .configure()
        .sysclk(8.mhz())
        .freeze(&mut dp.FLASH);*/
    let mut clock = rcc.cfgr.sysclk(8.mhz()).use_hse(8.mhz()).hclk(8.mhz()).freeze();

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    let mut can_stby = gpiob.pb3.into_push_pull_output();
    can_stby.set_low().unwrap();

    let usr_led = gpioc.pc6.into_push_pull_output();


    let ticks_time = TicksTime::new(&mut cp.SYST, clock.hclk().0);
    let delay = Delay::new(cp.SYST, clock);

    let can_rx = gpiob.pb8.into_alternate_af9();
    let can_tx = gpiob.pb9.into_alternate_af9();
    let can_iface = reg_can_init(dp.CAN1, can_tx, can_rx);
    (usr_led, dp.FLASH, cp.SCB, ticks_time, delay, can_iface, dp.SYSCFG)
}
