#![no_std]
#![no_main]

extern crate alloc;

mod display_target;
mod mcu;

use core::mem::MaybeUninit;

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
    mode, peripherals,
    time::Hertz,
};
use embassy_time::Timer;
use embedded_graphics::{
    mono_font::{self, ascii},
    pixelcolor::Rgb565,
    prelude::{Point, RgbColor, Size, WebColors},
};
use kolibri_embedded_gui::{
    checkbox::Checkbox,
    iconbutton::IconButton,
    icons::size48px,
    label::Label,
    style::{Spacing, Style},
    ui::{Interaction, Ui},
};
use mcu::{double_buffer::DoubleBuffer, mt48lc4m32b2, rcc_setup, ALLOCATOR};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    LTDC => ltdc::InterruptHandler<peripherals::LTDC>;
});

const HEAP_SIZE: usize = 8 * 1024;
const DISPLAY_WIDTH: usize = 480;
const DISPLAY_HEIGHT: usize = 272;
pub type TargetPixelType = u16;

#[link_section = ".frame_buffer"]
static mut FB1: [TargetPixelType; DISPLAY_WIDTH * DISPLAY_HEIGHT] =
    [0; DISPLAY_WIDTH * DISPLAY_HEIGHT];
#[link_section = ".frame_buffer"]
static mut FB2: [TargetPixelType; DISPLAY_WIDTH * DISPLAY_HEIGHT] =
    [0; DISPLAY_WIDTH * DISPLAY_HEIGHT];
#[link_section = ".heap"]
static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

#[embassy_executor::task()]
async fn display_task(
    mut double_buffer: DoubleBuffer,
    mut ltdc: Ltdc<'static, peripherals::LTDC>,
    mut i2c: I2c<'static, mode::Blocking>,
    mut touch: ft5336::Ft5336<'static, I2c<'static, mode::Blocking>>,
) -> ! {
    use embassy_stm32::pac::LTDC;

    info!("Display task started");

    // Re-configure the LTDC layer to match the framebuffer
    // the driver is incorrect for the DISCOVERY board
    LTDC.layer(0).cfblr().write(|w| {
        w.set_cfbp(DISPLAY_WIDTH as u16 * 4_u16);
        w.set_cfbll((DISPLAY_WIDTH as u16 * 4_u16) + 3);
    });

    let mut i: u8 = 0;
    let mut is_touched = false;
    let mut was_touched = false;
    let mut touch_location = Point::new(0, 0);

    pub fn jrny_style() -> Style<Rgb565> {
        Style {
            background_color: Rgb565::new(0x4, 0x8, 0x4), // pretty dark gray
            item_background_color: Rgb565::new(0x2, 0x4, 0x2), // darker gray
            highlight_item_background_color: Rgb565::new(0x1, 0x2, 0x1),
            border_color: Rgb565::WHITE,
            highlight_border_color: Rgb565::WHITE,
            primary_color: Rgb565::CSS_DARK_CYAN,
            secondary_color: Rgb565::YELLOW,
            icon_color: Rgb565::WHITE,
            text_color: Rgb565::WHITE,
            default_widget_height: 50,
            border_width: 0,
            highlight_border_width: 1,
            default_font: mono_font::iso_8859_10::FONT_9X15,
            spacing: Spacing {
                item_spacing: Size::new(8, 4),
                button_padding: Size::new(5, 5),
                default_padding: Size::new(1, 1),
                window_border_padding: Size::new(3, 3),
            },
        }
    }

    loop {
        let mut display = display_target::DisplayBuffer {
            buf: double_buffer.current(),
            width: DISPLAY_WIDTH as i32,
            height: DISPLAY_HEIGHT as i32,
        };

        // create UI (needs to be done each frame)
        let mut ui = Ui::new_fullscreen(&mut display, jrny_style());

        match (is_touched, was_touched, touch_location) {
            (true, false, loc) => {
                ui.interact(Interaction::Click(loc));
            }
            (true, true, loc) => {
                ui.interact(Interaction::Drag(loc));
            }
            (false, true, loc) => {
                ui.interact(Interaction::Release(loc));
            }
            (false, false, _) => {
                //ui.interact(Interaction::Hover(loc));
            }
        }
        was_touched = is_touched;

        // clear UI background (for non-incremental redrawing framebuffered applications)
        ui.clear_background().ok();

        // === ACTUAL UI CODE STARTS HERE ===

        ui.add(Label::new("Basic Example").with_font(ascii::FONT_10X20));

        if ui
            .add_horizontal(IconButton::new(size48px::actions::Minus))
            .clicked()
        {
            i = i.saturating_sub(1);
        }
        ui.add_horizontal(Label::new(alloc::format!("Clicked {} times", i).as_ref()));
        if ui
            .add_horizontal(IconButton::new(size48px::actions::Plus))
            .clicked()
        {
            i = i.saturating_add(1);
        }

        ui.new_row();

        let mut checked = true;
        ui.add(Checkbox::new(&mut checked));

        ui.add(IconButton::new(size48px::system::Settings));

        double_buffer.swap(&mut ltdc).await.unwrap();

        handle_touch(&mut touch, &mut i2c, &mut touch_location, &mut is_touched);

        Timer::after_millis(20).await;
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

    static DELAY: StaticCell<embassy_time::Delay> = StaticCell::new();
    let delay = DELAY.init(embassy_time::Delay);

    let ram_ptr: *mut u32 = sdram.init(delay) as *mut _;
    info!("SDRAM Initialized at {:x}", ram_ptr as *const _);

    unsafe { ALLOCATOR.init(&raw mut HEAP as usize, HEAP_SIZE) }

    let i2c = I2c::new_blocking(p.I2C3, p.PH7, p.PH8, Hertz(100_000), Default::default());
    let touch = ft5336::Ft5336::new(&i2c, 0x38, delay).unwrap();

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

    unwrap!(spawner.spawn(display_task(double_buffer, ltdc, i2c, touch)));

    let mut led = Output::new(p.PI1, Level::High, Speed::Low);

    loop {
        led.set_high();
        Timer::after_millis(1000).await;

        led.set_low();
        Timer::after_millis(1000).await;
    }
}

pub fn handle_touch(
    touch: &mut ft5336::Ft5336<'static, I2c<'static, mode::Blocking>>,
    i2c: &mut I2c<'static, mode::Blocking>,
    point: &mut Point,
    is_touched: &mut bool,
) {
    *is_touched = false;
    let t = touch.detect_touch(i2c);
    let mut num: u8 = 0;
    match t {
        Err(e) => info!("Error {} retrieving number of touches", e),
        Ok(n) => {
            num = n;
            if num != 0 {
                info!("Number of touches: {}", num)
            };
        }
    }

    if num > 0 {
        let t = touch.get_touch(i2c, 1);
        match t {
            Err(_e) => info!("Error fetching touch data"),
            Ok(n) => {
                info!(
                    "Touch: {}x{} - weight: {} misc: {}",
                    n.x, n.y, n.weight, n.misc
                );

                *is_touched = true;
                point.x = n.y as i32;
                point.y = n.x as i32;
            }
        }
    }
}
