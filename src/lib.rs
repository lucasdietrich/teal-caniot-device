#![no_std]
#![allow(unexpected_cfgs)]

use core::ffi::c_int;

use zephyr::printkln;

use log::info;

#[cfg(CONFIG_TEST)]
extern "C" {
    fn test_main() -> c_int;
}

#[no_mangle]
extern "C" fn rust_main() {
    unsafe {
        zephyr::set_logger().unwrap();
    }

    printkln!("Hello world from Rust on {}", zephyr::kconfig::CONFIG_BOARD);

    info!("Hello world from Rust on {}", zephyr::kconfig::CONFIG_BOARD);

    #[cfg(CONFIG_TEST)]
    unsafe {
        test_main();
    }
}