#![no_std]

extern crate alloc;

use embedded_alloc::LlffHeap as Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

pub fn init_heap() {
    unsafe {
        embedded_alloc::init!(HEAP, 1024);
    }
}

pub mod aht20;
pub mod config;
pub mod lora;
