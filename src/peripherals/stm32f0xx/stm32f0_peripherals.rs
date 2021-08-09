
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
use mcp25625;

pub type Mcp25625Sck = PB3<Alternate<AF0>>;
pub const MCP25625SPI_FREQ: MegaHertz = MegaHertz(1);
pub type Mcp25625Miso = PB4<Alternate<AF0>>;
pub type Mcp25625Mosi = PB5<Alternate<AF0>>;
pub type Mcp25625Cs = PC14<Output<PushPull>>;
pub type Mcp25625Spi = SPI1;
pub type Mcp25625Instance = mcp25625::MCP25625<hal::spi::Spi<Mcp25625Spi, Mcp25625Sck, Mcp25625Miso, Mcp25625Mosi, EightBit>, Mcp25625Cs>;
pub type Mcp25625Irq = PC15<Input<PullUp>>;
pub const MCP25625_IRQ_HANDLER: Interrupt = Interrupt::EXTI4_15;

pub use stm32f0xx_hal::stm32::FLASH;
pub use stm32f0xx_hal::stm32::SYST;
pub use stm32f0xx_hal::prelude::*;

use crate::ticks_time::TicksTime;
use stm32f0xx_hal::pac::{SYSCFG, SCB};
use stm32f0xx_hal::gpio::{Input, PullUp, AF0};
use stm32f0xx_hal::spi::EightBit;
use mcp25625::{MCP25625Config, McpOperationMode, FiltersConfig, McpErrorKind};
use stm32f0xx_hal::gpio::gpiob::{PB4, PB5};
use stm32f0xx_hal::gpio::gpioc::{PC15, PC14};
use stm32f0xx_hal::time::MegaHertz;

pub type UsrLedPin = PA6<Output<PushPull>>;

fn mcp25625_init(
    spi: Mcp25625Spi,
    sck: Mcp25625Sck,
    miso: Mcp25625Miso,
    mosi: Mcp25625Mosi,
    cs: Mcp25625Cs,
    rcc: &mut hal::rcc::Rcc,
) -> Result<Mcp25625Instance, mcp25625::McpErrorKind> {
    let spi = hal::spi::Spi::spi1(
        spi,
        (sck, miso, mosi),
        embedded_hal::spi::MODE_0,
        MCP25625SPI_FREQ,
        rcc
    );
    let mut mcp25625 = mcp25625::MCP25625::new(
        spi,
        cs,
        MCP25625SPI_FREQ.0 * 1_000_000,
        rcc.clocks.sysclk().0
    );
    //mcp25625_configure(&mut mcp25625)?;
    Ok(mcp25625)
}

pub fn mcp25625_configure(mcp25625: &mut Mcp25625Instance, filter_cfg: FiltersConfig) -> Result<(), McpErrorKind> {
    // let filters_buffer0 = FiltersConfigBuffer0 {
    //     mask: FiltersMask::AllExtendedIdBits,
    //     filter0: config::,
    //     filter1: None
    // };
    // let filters_buffer1 = FiltersConfigBuffer1 {
    //     mask: FiltersMask::OnlyStandardIdBits,
    //     filter2: config::,
    //     filter3: None,
    //     filter4: None,
    //     filter5: None,
    // };
    // let filters_config = FiltersConfig::Filter(filters_buffer0, Some(filters_buffer1));filter_cfg
    let mcp_config = MCP25625Config {
        brp: 0, // Fosc=16MHz
        prop_seg: 3,
        ph_seg1: 2,
        ph_seg2: 2,
        sync_jump_width: 2,
        rollover_to_buffer1: true,
        // filters_config: FiltersConfig::ReceiveAll,
        operation_mode: McpOperationMode::Normal,
        filters_config: filter_cfg
    };
    mcp25625.apply_config(mcp_config)?;
    mcp25625.enable_interrupts(0b0001_1111);
    Ok(())
}

pub fn setup_peripherals() -> (UsrLedPin, FLASH, SCB, TicksTime, Delay, Mcp25625Instance, SYSCFG){

    let mut dp= stm32::Peripherals::take().unwrap();
    let cs = unsafe {CriticalSection::new()};
    let mut rcc = dp.RCC;
    let mut cp = stm32::CorePeripherals::take().unwrap();

    rcc.apb1enr.modify(|_, w| w.canen().enabled()); // can time enb
    rcc.apb2enr.modify(|_, w| w.syscfgen().enabled());

    let mut clock = rcc
        .configure()
        .sysclk(8.mhz())
        .freeze(&mut dp.FLASH);

    let gpioa = dp.GPIOA.split(&mut clock);
    let gpiob = dp.GPIOB.split(&mut clock);
    let gpioc = dp.GPIOC.split(&mut clock);



    let usr_led = gpioa.pa6.into_push_pull_output(&cs);

    let mcp_config = MCP25625Config {
        brp: 0, // Fosc=16MHz
        prop_seg: 0,
        ph_seg1: 1,
        ph_seg2: 2,
        sync_jump_width: 1,
        rollover_to_buffer1: false,
        //filters_config,
        filters_config: FiltersConfig::ReceiveAll,
        operation_mode: McpOperationMode::Normal
    };

    let sck: Mcp25625Sck = gpiob.pb3.into_alternate_af0(&cs);//.into_analog(&cs);
    let mosi: Mcp25625Mosi = gpiob.pb5.into_alternate_af0(&cs);//.into_analog(&cs);
    let miso: Mcp25625Miso = gpiob.pb4.into_alternate_af0(&cs);//.into_analog(&cs);
    let cs_pin: Mcp25625Cs = gpioc.pc14.into_push_pull_output(&cs);
    let irq: Mcp25625Irq = gpioc.pc15.into_pull_up_input(&cs);
    let mut can_stby = gpioa.pa15.into_push_pull_output(&cs);
    can_stby.set_low().ok();

    let mcp25625 = match mcp25625_init(dp.SPI1, sck, miso, mosi, cs_pin, &mut clock) {
        Ok(mcp25625) => {
            Some(mcp25625)
        }
        Err(e) => {
            None
        }
    };

    let ticks_time = TicksTime::new(&mut cp.SYST, &clock);
    let delay = Delay::new(cp.SYST, &clock);
    let can_rx = gpioa.pa11.into_alternate_af4(&cs);
    let can_tx = gpioa.pa12.into_alternate_af4(&cs);

    (usr_led, dp.FLASH, cp.SCB, ticks_time, delay, mcp25625.unwrap(), dp.SYSCFG)
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
