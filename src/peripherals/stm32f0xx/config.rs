use cfg_if::cfg_if;
use vhrdcan::FrameId;
cfg_if! {
     if #[cfg(feature = "spi-can")]{
        use stm32f0xx_hal::gpio::gpiob::PB3;
        use stm32f0xx_hal::gpio::{Alternate, Output, PushPull};
        use stm32f0xx_hal::pac::{SYSCFG, SCB, Interrupt};
        use stm32f0xx_hal::gpio::{Input, PullUp, AF0};
        use stm32f0xx_hal::spi::EightBit;
        use stm32f0xx_hal::gpio::gpiob::{PB4, PB5};
        use stm32f0xx_hal::gpio::gpioc::{PC15, PC14};
        use stm32f0xx_hal::time::MegaHertz;

        pub use mcp25625;
        pub use mcp25625::{MCP25625Config, McpOperationMode, FiltersConfig, McpErrorKind};
        pub use mcp25625::FiltersConfig::Filter;
        pub use mcp25625::{McpPriority, McpReceiveBuffer, TxBufferChoice};


        pub type Mcp25625Sck = PB3<Alternate<AF0>>;
        pub const MCP25625SPI_FREQ: MegaHertz = MegaHertz(1);
        pub type Mcp25625Miso = PB4<Alternate<AF0>>;
        pub type Mcp25625Mosi = PB5<Alternate<AF0>>;
        pub type Mcp25625Cs = PC14<Output<PushPull>>;
        pub type Mcp25625Spi = stm32f0xx_hal::pac::SPI1;
        pub type Mcp25625Instance = mcp25625::MCP25625<stm32f0xx_hal::spi::Spi<Mcp25625Spi, Mcp25625Sck, Mcp25625Miso, Mcp25625Mosi, EightBit>, Mcp25625Cs>;
        pub type Mcp25625Irq = PC15<Input<PullUp>>;
        pub const MCP25625_IRQ_HANDLER: Interrupt = Interrupt::EXTI4_15;
    }
    else if #[cfg(feature = "reg-can")]{
            use stm32f0xx_hal::gpio::{Alternate, AF4, gpioa::{PA11, PA12}};
            pub use bxcan::filter::BankConfig;
            pub use stm32f0xx_hal::can::bxcan::Frame as BxFrame;

            pub type CanTx = PA12<Alternate<AF4>>;
            pub type CanRx = PA11<Alternate<AF4>>;
            pub type CanStmInstance = stm32f0xx_hal::can::bxcan::Can<stm32f0xx_hal::can::CanInstance<CanTx, CanRx>>;

            pub fn vhrdcanid2bxcanid(id: FrameId) -> crate::hal::can::bxcan::Id {
               use stm32f0xx_hal::can::bxcan::{Id, StandardId, ExtendedId};

               match id {
                    FrameId::Standard(sid) => { Id::Standard(StandardId::new(sid.id()).unwrap()) }
                    FrameId::Extended(eid) => { Id::Extended(ExtendedId::new(eid.id()).unwrap()) }
                }
            }
            pub fn bxcanid2vhrdcanid(id: crate::hal::can::bxcan::Id) -> FrameId {
                use stm32f0xx_hal::can::bxcan::Id;
                use vhrdcan::id::{StandardId, ExtendedId};
                match id {
                    Id::Standard(sid) => { FrameId::Standard( unsafe { StandardId::new_unchecked(sid.as_raw()) } )}
                    Id::Extended(eid) => { FrameId::Extended( unsafe { ExtendedId::new_unchecked(eid.as_raw()) } )}
                }
            }
    }
}