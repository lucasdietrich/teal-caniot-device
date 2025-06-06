use core::ptr::null_mut;

use zephyr::{
    embedded_can::{self, blocking::Can},
    raw,
    sys::K_FOREVER,
};

use super::frame::CanFrame;

pub struct CanDevice(pub(crate) *const raw::device);

impl CanDevice {}

impl Can for CanDevice {
    type Frame = CanFrame;
    type Error = zephyr::Error;

    fn transmit(&mut self, frame: &Self::Frame) -> Result<(), Self::Error> {
        let ret = unsafe { raw::can_send(self.0, &frame.0, K_FOREVER, None, null_mut()) };

        if ret == 0 {
            Ok(())
        } else {
            Err(zephyr::Error(-ret as u32))
        }
    }

    fn receive(&mut self) -> Result<Self::Frame, Self::Error> {
        todo!()
    }
}
