use cfg_if::cfg_if;
use vhrdcan::FrameId;
cfg_if! {
    if #[cfg(feature = "reg-can")]{
            use stm32f4xx_hal::gpio::{Alternate, AF9, gpiob::{PB8, PB9}};
            pub use bxcan::filter::BankConfig;
            pub use bxcan::Frame as BxFrame;

            pub type CanTx = PB9<Alternate<AF9>>;
            pub type CanRx = PB8<Alternate<AF9>>;
            pub type CanStmInstance = bxcan::Can<stm32f4xx_hal::can::Can<stm32f4xx_hal::stm32::CAN1>>;

            pub fn vhrdcanid2bxcanid(id: FrameId) -> bxcan::Id {
               use bxcan::{Id, StandardId, ExtendedId};

               match id {
                    FrameId::Standard(sid) => { Id::Standard(StandardId::new(sid.id()).unwrap()) }
                    FrameId::Extended(eid) => { Id::Extended(ExtendedId::new(eid.id()).unwrap()) }
                }
            }
            pub fn bxcanid2vhrdcanid(id: bxcan::Id) -> FrameId {
                use bxcan::Id;
                use vhrdcan::id::{StandardId, ExtendedId};
                match id {
                    Id::Standard(sid) => { FrameId::Standard( unsafe { StandardId::new_unchecked(sid.as_raw()) } )}
                    Id::Extended(eid) => { FrameId::Extended( unsafe { ExtendedId::new_unchecked(eid.as_raw()) } )}
                }
            }
    }
}