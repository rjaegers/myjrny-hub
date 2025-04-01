pub mod double_buffer;
pub mod mt48lc4m32b2;
pub mod rcc_setup;

use embedded_alloc::Heap;

#[global_allocator]
pub static ALLOCATOR: Heap = Heap::empty();
