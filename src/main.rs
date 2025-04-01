#![no_std]
#![no_main]

extern crate alloc;

mod mcu;
mod slint_backend;

use alloc::{boxed::Box, rc::Rc};
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    fmc::Fmc,
    gpio::{AfType, Flex, Level, Output, OutputType, Speed},
    i2c::I2c,
    ltdc::{
        self, DePin, Ltdc, LtdcConfiguration, LtdcLayer, LtdcLayerConfig, PolarityActive,
        PolarityEdge,
    },
    peripherals,
    time::Hertz,
};
use embassy_time::Timer;

use mcu::{double_buffer::DoubleBuffer, mt48lc4m32b2, rcc_setup, ALLOCATOR};
use slint::{
    platform::software_renderer::{MinimalSoftwareWindow, RepaintBufferType, Rgb565Pixel},
    ComponentHandle,
};
use slint_backend::{StmBackend, TargetPixelType, DISPLAY_HEIGHT, DISPLAY_WIDTH};
use slint_generated::MainWindow;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    LTDC => ltdc::InterruptHandler<peripherals::LTDC>;
});

const HEAP_SIZE: usize = 200 * 1024;

#[link_section = ".frame_buffer"]
static mut FB1: [TargetPixelType; DISPLAY_WIDTH * DISPLAY_HEIGHT] =
    [Rgb565Pixel(0); DISPLAY_WIDTH * DISPLAY_HEIGHT];
#[link_section = ".frame_buffer"]
static mut FB2: [TargetPixelType; DISPLAY_WIDTH * DISPLAY_HEIGHT] =
    [Rgb565Pixel(0); DISPLAY_WIDTH * DISPLAY_HEIGHT];
#[link_section = ".heap"]
static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

#[embassy_executor::task()]
async fn display_task(
    window: Rc<MinimalSoftwareWindow>,
    mut double_buffer: DoubleBuffer,
    mut ltdc: Ltdc<'static, peripherals::LTDC>,
) -> ! {
    use embassy_stm32::pac::LTDC;

    info!("Display task started");

    // Re-configure the LTDC layer to match the framebuffer
    // the driver is incorrect for the DISCOVERY board
    LTDC.layer(0).cfblr().write(|w| {
        w.set_cfbp(DISPLAY_WIDTH as u16 * 4_u16);
        w.set_cfbll((DISPLAY_WIDTH as u16 * 4_u16) + 3);
    });

    loop {
        slint::platform::update_timers_and_animations();

        // blocking render
        let is_dirty = window.draw_if_needed(|renderer| {
            let buffer = double_buffer.current();
            renderer.render(buffer, DISPLAY_WIDTH);
        });

        if is_dirty {
            // async transfer of frame buffer to lcd
            double_buffer.swap(&mut ltdc).await.unwrap();
        } else {
            Timer::after_millis(10).await
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = rcc_setup::stm32f746ng_init();
    info!("Starting...");

    // Config SDRAM
    // ----------------------------------------------------------
    // Configure MPU for external SDRAM (64 Mbit = 8 Mbyte)
    // MPU is disabled by default

    let mut delay = embassy_time::Delay;

    let mut sdram = Fmc::sdram_a12bits_d16bits_4banks_bank1(
        p.FMC,
        // A0-A11
        p.PF0,
        p.PF1,
        p.PF2,
        p.PF3,
        p.PF4,
        p.PF5,
        p.PF12,
        p.PF13,
        p.PF14,
        p.PF15,
        p.PG0,
        p.PG1,
        // BA0-BA1
        p.PG4,
        p.PG5,
        // D0-D15
        p.PD14,
        p.PD15,
        p.PD0,
        p.PD1,
        p.PE7,
        p.PE8,
        p.PE9,
        p.PE10,
        p.PE11,
        p.PE12,
        p.PE13,
        p.PE14,
        p.PE15,
        p.PD8,
        p.PD9,
        p.PD10,
        // NBL0 - NBL1
        p.PE0,
        p.PE1,
        p.PC3,  // SDCKE0
        p.PG8,  // SDCLK
        p.PG15, // SDNCAS
        p.PH3,  // SDNE0 (!CS)
        p.PF11, // SDRAS
        p.PH5,  // SDNWE
        mt48lc4m32b2::mt48lc4m32b2_6::Mt48lc4m32b2 {},
    );

    // Initialise controller and SDRAM
    let ram_ptr: *mut u32 = sdram.init(&mut delay) as *mut _;

    info!("SDRAM Initialized at {:x}", ram_ptr as *const _);

    let heap = unsafe { &mut *core::ptr::addr_of_mut!(HEAP) };
    let heap_size = core::mem::size_of_val(heap);
    crate::assert!(heap_size > 0);

    unsafe {
        ALLOCATOR.init(heap as *const u8 as usize, heap_size);
    }

    let mut i2c = I2c::new_blocking(p.I2C3, p.PH7, p.PH8, Hertz(100_000), Default::default());
    let mut touch = ft5336::Ft5336::new(&i2c, 0x38, &mut delay).unwrap();

    // set up the LTDC peripheral to send data to the LCD screen
    // setting timing for RK043FN48H
    // Details taken from: https://github.com/STMicroelectronics/stm32-rk043fn48h/blob/main/rk043fn48h.h
    const RK043FN48H_HSYNC: u16 = 41; // Horizontal synchronization
    const RK043FN48H_HBP: u16 = 13; // Horizontal back porch
    const RK043FN48H_HFP: u16 = 32; // Horizontal front porch
    const RK043FN48H_VSYNC: u16 = 10; // Vertical synchronization
    const RK043FN48H_VBP: u16 = 2; // Vertical back porch
    const RK043FN48H_VFP: u16 = 2; // Vertical front porch

    let ltdc_config = LtdcConfiguration {
        active_width: DISPLAY_WIDTH as _,
        active_height: DISPLAY_HEIGHT as _,
        h_back_porch: RK043FN48H_HBP,
        h_front_porch: RK043FN48H_HFP,
        v_back_porch: RK043FN48H_VBP,
        v_front_porch: RK043FN48H_VFP,
        h_sync: RK043FN48H_HSYNC,
        v_sync: RK043FN48H_VSYNC,
        h_sync_polarity: PolarityActive::ActiveLow,
        v_sync_polarity: PolarityActive::ActiveLow,
        data_enable_polarity: PolarityActive::ActiveLow,
        pixel_clock_polarity: PolarityEdge::RisingEdge,
    };

    let ltdc_de_af = p.PK7.af_num();
    let mut ltdc_de = Flex::new(p.PK7);
    ltdc_de.set_as_af_unchecked(ltdc_de_af, AfType::output(OutputType::PushPull, Speed::Low));
    let mut _ltdc_disp_ctrl = Output::new(p.PI12, Level::High, Speed::Low);
    let mut _ltdc_bl_ctrl = Output::new(p.PK3, Level::High, Speed::Low);

    let mut ltdc = Ltdc::new_with_pins(
        p.LTDC, // PERIPHERAL
        Irqs,   // IRQS
        p.PI14, // CLK
        p.PI10, // HSYNC
        p.PI9,  // VSYNC
        // B: PE4, PJ13..15, PG12, PK4..6, 8 bits
        p.PE4,  // B0
        p.PJ13, // B1
        p.PJ14, // B2
        p.PJ15, // B3
        p.PG12, // B4
        p.PK4,  // B5
        p.PK5,  // B6
        p.PK6,  // B7
        // G: PJ7..11, PK0..2, 8 bits
        p.PJ7,  // G0
        p.PJ8,  // G1
        p.PJ9,  // G2
        p.PJ10, // G3
        p.PJ11, // G4
        p.PK0,  // G5
        p.PK1,  // G6
        p.PK2,  // G7
        // R: PI15, PJ0..6, 8 bits
        p.PI15, // R0
        p.PJ0,  // R1
        p.PJ1,  // R2
        p.PJ2,  // R3
        p.PJ3,  // R4
        p.PJ4,  // R5
        p.PJ5,  // R6
        p.PJ6,  // R7
    );
    ltdc.init(&ltdc_config);

    let layer_config = LtdcLayerConfig {
        pixel_format: ltdc::PixelFormat::RGB565,
        layer: LtdcLayer::Layer1,
        window_x0: 0,
        window_x1: DISPLAY_WIDTH as _,
        window_y0: 0,
        window_y1: DISPLAY_HEIGHT as _,
    };

    ltdc.init_layer(&layer_config, None);

    let (fb1, fb2) = unsafe {
        (
            &mut *core::ptr::addr_of_mut!(FB1),
            &mut *core::ptr::addr_of_mut!(FB2),
        )
    };

    info!(
        "Display buffers allocated at {:x}, {:x}",
        fb1 as *const _, fb2 as *const _
    );

    let double_buffer = DoubleBuffer::new(fb1, fb2, layer_config);

    // create a slint window and register it with slint
    let window = MinimalSoftwareWindow::new(RepaintBufferType::SwappedBuffers);
    window.set_size(slint::PhysicalSize::new(
        DISPLAY_WIDTH as u32,
        DISPLAY_HEIGHT as u32,
    ));
    let backend = Box::new(StmBackend::new(window.clone()));
    slint::platform::set_platform(backend).expect("backend already initialized");
    info!("slint gui setup complete");

    unwrap!(spawner.spawn(display_task(window, double_buffer, ltdc)));

    let main_window = MainWindow::new().unwrap();
    main_window.show().expect("unable to show main window");

    let mut led = Output::new(p.PI1, Level::High, Speed::Low);

    loop {
        let t = touch.detect_touch(&mut i2c);
        let mut num: u8 = 0;
        match t {
            Err(e) => info!("Error {} from fetching number of touches", e),
            Ok(n) => {
                num = n;
                if num != 0 {
                    info!("Number of touches: {}", num)
                };
            }
        }

        if num > 0 {
            let t = touch.get_touch(&mut i2c, 1);
            match t {
                Err(_e) => info!("Error fetching touch data"),
                Ok(n) => info!(
                    "Touch: {}x{} - weight: {} misc: {}",
                    n.x, n.y, n.weight, n.misc
                ),
            }
        }

        //led.set_high();
        //Timer::after_millis(1000).await;

        //led.set_low();
        //Timer::after_millis(1000).await;
    }
}
