#![no_std]
#![no_main]

use cortex_m_rt::entry;
use km3_rs as _; // memory layout + panic handler

#[entry]
fn main() -> ! {
    assert!(false, "TODO: Write actual tests");

    km3_rs::exit();
}
