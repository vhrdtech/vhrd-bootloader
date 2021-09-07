#[cfg(feature = "stm32f0xx-hal")]
pub mod stm32f0xx;
#[cfg(feature = "stm32f0xx-hal")]
pub use stm32f0xx as stm32p;
#[cfg(feature = "stm32f0xx-hal")]
pub use stm32p::*;

#[cfg(feature = "stm32f4xx-hal")]
pub mod stm32f4xx;
#[cfg(feature = "stm32f4xx-hal")]
pub use stm32f4xx as stm32p;
#[cfg(feature = "stm32f4xx-hal")]
pub use stm32p::*;
