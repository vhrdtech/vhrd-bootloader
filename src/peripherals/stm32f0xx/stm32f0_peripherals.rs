use cfg_if::cfg_if;
use stm32f0xx_hal as hal;
use hal::{
    stm32,
    delay::Delay,
    pac,
};
use cortex_m::interrupt::{CriticalSection};
use stm32f0xx_hal::stm32::RCC;

use stm32f0xx_hal::{
    gpio::gpioa::{PA6},
    gpio::gpiob::{PB3},
    gpio::{Alternate, Output, PushPull},
};

use pac::{Interrupt,SPI1};

use crate::config::*;

#[cfg(feature = "spi-can")]
use crate::mcp_canbus::*;
#[cfg(feature = "spi-can")]
pub type CanInstance = Mcp25625Instance;

#[cfg(feature = "reg-can")]
use crate::reg_canbus::*;
#[cfg(feature = "reg-can")]
pub type CanInstance = CanStmInstance;

pub use stm32f0xx_hal::stm32::FLASH;
pub use stm32f0xx_hal::stm32::SYST;
pub use stm32f0xx_hal::prelude::*;

use crate::ticks_time::TicksTime;

use stm32f0xx_hal::pac::{SYSCFG, SCB, Peripherals};


pub type UsrLedPin = PA6<Output<PushPull>>;

pub fn setup_peripherals() -> (UsrLedPin, FLASH, SCB, TicksTime, Delay, CanInstance, SYSCFG){

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

    rcc.apb1enr.modify(|_, w| w.canen().enabled()); // can time enb
    rcc.apb2enr.modify(|_, w| w.syscfgen().enabled());

    let mut clock = rcc
        .configure()
        .sysclk(8.mhz())
        .freeze(&mut dp.FLASH);

    let gpioa = dp.GPIOA.split(&mut clock);
    let gpiob = dp.GPIOB.split(&mut clock);
    let gpioc = dp.GPIOC.split(&mut clock);

    let mut can_stby = gpioa.pa15.into_push_pull_output(&cs);
    can_stby.set_low().ok();

    cfg_if! {
        if #[cfg(feature = "pi-en")] {
            let mut pi_en = gpiob.pb0.into_push_pull_output(&cs);
            pi_en.set_high().ok();
        }
    }

    let usr_led = gpioa.pa6.into_push_pull_output(&cs);

    let ticks_time = TicksTime::new(&mut cp.SYST, clock.clocks.hclk().0);
    let delay = Delay::new(cp.SYST, &clock);

    cfg_if! {
    if #[cfg(feature = "spi-can")]{
        let sck: Mcp25625Sck = gpiob.pb3.into_alternate_af0(&cs);//.into_analog(&cs);
        let mosi: Mcp25625Mosi = gpiob.pb5.into_alternate_af0(&cs);//.into_analog(&cs);
        let miso: Mcp25625Miso = gpiob.pb4.into_alternate_af0(&cs);//.into_analog(&cs);
        let cs_pin: Mcp25625Cs = gpioc.pc14.into_push_pull_output(&cs);
        let irq: Mcp25625Irq = gpioc.pc15.into_pull_up_input(&cs);


        match mcp25625_init(dp.SPI1, sck, miso, mosi, cs_pin, &mut clock){
                Ok(can_iface) => {
                    (usr_led, dp.FLASH, cp.SCB, ticks_time, delay, can_iface, dp.SYSCFG)
                }
                Err(_) => {panic!()}
            }
        }
        else if #[cfg(feature = "reg-can")]{
            let can_rx = gpioa.pa11.into_alternate_af4(&cs);
            let can_tx = gpioa.pa12.into_alternate_af4(&cs);
            let can_iface = reg_can_init(dp.CAN, can_tx, can_rx, &mut clock);
            (usr_led, dp.FLASH, cp.SCB, ticks_time, delay, can_iface, dp.SYSCFG)
        }
    }




}
pub fn deinit_peripherals() {
    unsafe{
        let mut dp = stm32::Peripherals::steal();
        let cp = stm32::CorePeripherals::steal();
        let mut flash = dp.FLASH;
        flash.ar.write(|w|unsafe {w.bits(0)} );
        flash.cr.write(|w|unsafe {w.bits(0x0000_0080)} );
        flash.sr.write(|w|unsafe {w.bits(0)} );
        flash.acr.write(|w|unsafe {w.bits(0)} );

        rcc_deinit(dp.RCC);

        let mut syst = cp.SYST;
        syst.disable_counter();
        syst.set_reload(0);

        syst.disable_interrupt();

    }
}

fn rcc_deinit(rcc: RCC){
    let mut timeout: u8 = 0;
    /* Set HSION bit, HSITRIM[4:0] bits to the reset value*/
    rcc.cr.modify(|_, w| w.hsion().on().hsitrim().bits(16));
    /* Wait till HSI is ready */
    while rcc.cr.read().hsirdy().bit_is_clear() || timeout < 200 { timeout += 1;}
    /* Reset SW[1:0], HPRE[3:0], PPRE[2:0] and MCOSEL[2:0] bits */
    rcc.cfgr.modify(|r,w| unsafe {w.sw().hsi().hpre().bits(0).ppre().bits(0).mcopre().bits(r.mco().bits())} );
    /* Wait till HSI as SYSCLK status is enabled */
    timeout = 0;
    while rcc.cfgr.read().sws().bits() != 0 || timeout < 200 { timeout += 1;}
    /* Reset HSEON, CSSON, PLLON bits */
    rcc.cr.modify(|_, w| w.hseon().clear_bit().csson().clear_bit().pllon().clear_bit());
    /* Reset HSEBYP bit */
    rcc.cr.modify(|_, w| w.hsebyp().set_bit());
    /* Wait till PLLRDY is cleared */
    timeout = 0;
    while rcc.cr.read().pllrdy().bit_is_set() || timeout < 200 { timeout += 1;}
    /* Reset CFGR register */
    rcc.cfgr.write(|w|unsafe {w.bits(0)} );
    /* Reset CFGR2 register */
    rcc.cfgr2.write(|w|unsafe {w.bits(0)} );
    /* Reset CFGR3 register */
    rcc.cfgr3.write(|w|unsafe {w.bits(0)} );
    /* Disable all interrupts */
    rcc.cir.write(|w|unsafe {w.bits(0)} );
    /* Clear all reset flags */
    rcc.csr.modify(|_, w| w.rmvf().set_bit());
}
