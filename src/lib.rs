// Copyright (c) 2024 Linaro LTD
// SPDX-License-Identifier: Apache-2.0

#![no_std]

use zephyr::printkln;

// Reference the Zephyr crate so that the panic handler gets used.  This is only needed if no
// symbols from the crate are directly used.
extern crate zephyr;

pub struct MyData {
    number: u32,
}

#[no_mangle]
extern "C" fn rust_main() {
    printkln!("Hello world from Rust on {}",
              zephyr::kconfig::CONFIG_BOARD);

    let data = MyData { number: 42 };

    printkln!("Number is {}", data.number);

    let var = zephyr::sync::Mutex::new(data);
    let guard = var.lock();
    
    if let Ok(mut guard) = guard {
        guard.number += 1;
        printkln!("Number is {}", guard.number);
    } else {
        printkln!("Mutex lock failed");
    }
}
