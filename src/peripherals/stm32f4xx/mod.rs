pub mod stm32f4_peripherals;
pub use stm32f4_peripherals::*;
pub mod config;
pub use config::*;

#[cfg(feature = "reg-can")]
pub mod reg_canbus;
#[cfg(feature = "reg-can")]
pub use reg_canbus::*;