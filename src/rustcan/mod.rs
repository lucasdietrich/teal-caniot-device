use core::{ffi::c_int, mem::MaybeUninit};
pub mod can;

use can::blocking::CanDevice;
use zephyr::{
    embedded_can::{self, blocking::Can, Frame},
    printkln,
    time::{sleep, Duration, Timeout},
};
use zephyr_sys::{can_frame, device, k_msgq, k_msgq_get};

extern "C" {
    fn can_init() -> c_int;
    static mut can_msgq: k_msgq;
    static dev_can: *const device;
}

pub fn rust_can_task() {
    unsafe {
        can_init();
    }

    let duration = Duration::millis_at_least(500);

    let mut can_device = unsafe { CanDevice(dev_can) };

    loop {
        let mut data = MaybeUninit::<can_frame>::uninit();
        let timeout: Timeout = Duration::millis_at_least(1000).into();
        let ret = unsafe { k_msgq_get(&raw mut can_msgq, &raw mut data as *mut _, timeout.0) };

        if ret == 0 {
            unsafe {
                let frame = data.assume_init();

                // Successfully received a message
                printkln!(
                    "Received CAN message id: {:x} dlc: {} flags: {} data: {:x?}",
                    frame.id,
                    frame.dlc,
                    frame.flags,
                    frame.__bindgen_anon_1.data.as_ref()
                );
            };
        } else {
            // Handle error or timeout
            printkln!("Failed to receive CAN message, error code: {}", ret);
        }

        // sleep(duration);

        // let id = embedded_can::StandardId::new(0x123).expect("Failed to create StandardId");
        // let frame =
        //     can::frame::CanFrame::new(id, &[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08])
        //         .expect("Failed to create CAN frame");

        // printkln!(
        //     "Sending CAN message id: {:?} dlc: {} data: {:x?}",
        //     frame.id(),
        //     frame.dlc(),
        //     frame.data()
        // );

        // can_device
        //     .transmit(&frame)
        //     .expect("Failed to transmit CAN frame");

        // printkln!("CAN frame transmitted successfully");
    }
}
