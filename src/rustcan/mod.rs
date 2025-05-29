use core::{ffi::c_int, mem::MaybeUninit};

use zephyr::{
    printkln,
    time::{sleep, Duration, Timeout},
};
use zephyr_sys::{can_frame, k_msgq, k_msgq_get};

extern "C" {
    fn can_init() -> c_int;
    static mut can_msgq: k_msgq;
}

pub fn rust_can_task() {
    unsafe {
        can_init();
    }

    let duration = Duration::millis_at_least(500);

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

        sleep(duration);
    }
}
