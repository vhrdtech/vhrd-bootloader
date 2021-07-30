use cortex_m::peripheral::SYST;
use cortex_m::peripheral::syst::SystClkSource;
use stm32f0xx_hal::rcc::Rcc;

const SYSTICK_RANGE: u32 = 0x0100_0000;

pub struct TicksTime{
    prev_tics: u32,
    clock_f: u32
}

impl TicksTime{
    pub fn new(syst: &mut SYST, clock: &Rcc) -> TicksTime{
        syst.set_clock_source(SystClkSource::Core);

        syst.set_reload(SYSTICK_RANGE - 1);
        syst.clear_current();
        syst.enable_counter();

        assert!(clock.clocks.hclk().0 >= 1_000_000);

        TicksTime{
            prev_tics: SYST::get_current(),
            clock_f: clock.clocks.hclk().0
        }
    }

    pub fn count_time_from_now(&mut self) {
        self.prev_tics = SYST::get_current();
    }

    pub fn get_delta_time_ms(&mut self) -> u32{
        let curr_ticks = SYST::get_current();
        let delta_ticks = (SYST::get_reload() / 2) - ((curr_ticks as i64 - self.prev_tics as i64).abs() -  (SYST::get_reload() / 2) as i64).abs() as u32;
        self.prev_tics = curr_ticks;
        (delta_ticks as u64 * 1_000u64 / self.clock_f as u64) as u32
    }

    pub fn get_delta_time_us(&mut self) -> u32{
        let curr_ticks = SYST::get_current();
        let delta_ticks = (SYST::get_reload() / 2) - ((curr_ticks as i64 - self.prev_tics as i64).abs() -  (SYST::get_reload() / 2) as i64).abs() as u32;
        self.prev_tics = curr_ticks;
        (delta_ticks as u64 * 1_000_000u64 / self.clock_f as u64) as u32
    }
}

