pub mod rcc_setup;

use embedded_alloc::TlsfHeap as Heap;

#[global_allocator]
pub static ALLOCATOR: Heap = Heap::empty();
