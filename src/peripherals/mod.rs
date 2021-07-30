#[cfg(feature = "stm32f0xx")]
pub mod stm32f0xx;
#[cfg(feature = "stm32f0xx")]
pub use stm32f0xx as stm32p;
#[cfg(feature = "stm32f0xx")]
pub use stm32p::*;
