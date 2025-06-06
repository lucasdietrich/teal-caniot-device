use core::mem::MaybeUninit;

use zephyr::embedded_can::{blocking::Can, Error, ExtendedId, Frame, Id, StandardId};
use zephyr_sys::can_frame;

// TODO how to let bindgen generate these constants from #include <zephyr/drivers/can.h>
// 1. Add to build.rs:         .allowlist_item("CAN_FRAME_.*") ?
pub(crate) const CAN_FRAME_IDE: u8 = 0x01;
pub(crate) const CAN_FRAME_RTR: u8 = 0x02;
pub(crate) const CAN_FRAME_FDF: u8 = 0x04;
pub(crate) const CAN_FRAME_BRS: u8 = 0x08;
pub(crate) const CAN_FRAME_ESI: u8 = 0x10;

// TODO how to derive Clone and Copy, using derive_copy! derive_clone! ??
pub struct CanFrame(pub(crate) can_frame);

impl CanFrame {
    fn build_frame(
        id: impl Into<Id>,
        dlc: usize,
        mut flags: u8,
        data: Option<&[u8]>,
    ) -> Option<Self> {
        // TODO add support for FD frames
        if dlc > 8 {
            return None;
        }

        let id: Id = id.into();
        let id_raw = match id {
            Id::Standard(id) => id.as_raw() as u32,
            Id::Extended(id) => {
                flags |= CAN_FRAME_IDE;
                id.as_raw()
            }
        };

        let mut frame = MaybeUninit::<can_frame>::uninit();
        let frame_ptr = frame.as_mut_ptr();

        let zframe = unsafe {
            (*frame_ptr).id = id_raw;
            (*frame_ptr).dlc = dlc as u8;
            (*frame_ptr).flags = flags;
            // reserved field is intentionally left uninitialized

            // data field is intentionally left uninitialized for rtr frames
            if let Some(data) = data {
                (*frame_ptr)
                    .__bindgen_anon_1
                    .data
                    .as_mut()
                    .copy_from_slice(data);
            }

            frame.assume_init()
        };

        Some(CanFrame(zframe))
    }
}

impl Frame for CanFrame {
    fn new(id: impl Into<Id>, data: &[u8]) -> Option<Self> {
        Self::build_frame(id, data.len(), 0, Some(data))
    }

    fn new_remote(id: impl Into<Id>, dlc: usize) -> Option<Self> {
        Self::build_frame(id, dlc, CAN_FRAME_RTR, None)
    }

    fn data(&self) -> &[u8] {
        unsafe { self.0.__bindgen_anon_1.data.as_ref() }
    }

    fn dlc(&self) -> usize {
        self.0.dlc as usize
    }

    fn id(&self) -> Id {
        if self.is_extended() {
            let id = unsafe { ExtendedId::new_unchecked(self.0.id) };
            Id::Extended(id)
        } else {
            let id = unsafe { StandardId::new_unchecked(self.0.id as u16) };
            Id::Standard(id)
        }
    }

    fn is_data_frame(&self) -> bool {
        self.0.flags == 0
    }

    fn is_extended(&self) -> bool {
        self.0.flags & CAN_FRAME_IDE != 0
    }

    fn is_standard(&self) -> bool {
        !self.is_extended()
    }

    fn is_remote_frame(&self) -> bool {
        self.0.flags & CAN_FRAME_RTR != 0
    }
}
