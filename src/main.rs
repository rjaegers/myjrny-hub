#![no_std]
#![no_main]

mod display_target;
mod mcu;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    fmc::Fmc,
    gpio::{Level, Output, Speed},
    ltdc::{
        self, Ltdc, LtdcConfiguration, LtdcLayer, LtdcLayerConfig, PolarityActive, PolarityEdge,
    },
    pac::ltdc::vals::{Bf1, Bf2, Imr, Pf},
    peripherals,
};
use embassy_time::Timer;
use embedded_graphics::{
    geometry::Size,
    mono_font::{self, ascii},
    pixelcolor::{Rgb565, RgbColor, WebColors},
};
use kolibri_embedded_gui::{button::Button, checkbox::Checkbox, label::Label, ui::Ui};
use mcu::{mt48lc4m32b2, rcc_setup};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    LTDC => ltdc::InterruptHandler<peripherals::LTDC>;
});

const DISPLAY_WIDTH: usize = 480;
const DISPLAY_HEIGHT: usize = 272;

#[link_section = ".frame_buffer"]
static mut FB1: [u16; DISPLAY_WIDTH * DISPLAY_HEIGHT] = [0; DISPLAY_WIDTH * DISPLAY_HEIGHT];
#[link_section = ".frame_buffer"]
static mut FB2: [u16; DISPLAY_WIDTH * DISPLAY_HEIGHT] = [0; DISPLAY_WIDTH * DISPLAY_HEIGHT];

#[embassy_executor::task()]
async fn display_task() -> ! {
    use embassy_stm32::pac::LTDC;

    info!("Display task started");

    const LCD_X_SIZE: u16 = 480;
    const LCD_Y_SIZE: u16 = 272;

    /* Initialize the LCD pixel width and pixel height */
    const WINDOW_X0: u16 = 0;
    const WINDOW_X1: u16 = LCD_X_SIZE; // 480 for ferris
    const WINDOW_Y0: u16 = 0;
    const WINDOW_Y1: u16 = LCD_Y_SIZE; // 800 for ferris
    const PIXEL_FORMAT: Pf = Pf::RGB565;
    //const FBStartAdress: u16 = FB_Address;
    const ALPHA: u8 = 255;
    const ALPHA0: u8 = 0;
    const BACKCOLOR_BLUE: u8 = 0;
    const BACKCOLOR_GREEN: u8 = 0;
    const BACKCOLOR_RED: u8 = 0;
    const IMAGE_WIDTH: u16 = LCD_X_SIZE; // 480 for ferris
    const IMAGE_HEIGHT: u16 = LCD_Y_SIZE; // 800 for ferris

    const PIXEL_SIZE: u8 = match PIXEL_FORMAT {
        Pf::ARGB8888 => 4,
        Pf::RGB888 => 3,
        Pf::ARGB4444 | Pf::RGB565 | Pf::ARGB1555 | Pf::AL88 => 2,
        _ => 1,
    };

    // Configure the horizontal start and stop position
    LTDC.layer(0).whpcr().write(|w| {
        w.set_whstpos(LTDC.bpcr().read().ahbp() + 1 + WINDOW_X0);
        w.set_whsppos(LTDC.bpcr().read().ahbp() + WINDOW_X1);
    });

    // Configures the vertical start and stop position
    LTDC.layer(0).wvpcr().write(|w| {
        w.set_wvstpos(LTDC.bpcr().read().avbp() + 1 + WINDOW_Y0);
        w.set_wvsppos(LTDC.bpcr().read().avbp() + WINDOW_Y1);
    });

    // Specify the pixel format
    LTDC.layer(0).pfcr().write(|w| w.set_pf(PIXEL_FORMAT));

    // Configures the default color values as zero
    LTDC.layer(0).dccr().modify(|w| {
        w.set_dcblue(BACKCOLOR_BLUE);
        w.set_dcgreen(BACKCOLOR_GREEN);
        w.set_dcred(BACKCOLOR_RED);
        w.set_dcalpha(ALPHA0);
    });

    // Specifies the constant ALPHA value
    LTDC.layer(0).cacr().write(|w| w.set_consta(ALPHA));

    // Specifies the blending factors
    LTDC.layer(0).bfcr().write(|w| {
        w.set_bf1(Bf1::CONSTANT);
        w.set_bf2(Bf2::CONSTANT);
    });

    let (display_buffer_1, display_buffer_2) = unsafe {
        (
            &mut *core::ptr::addr_of_mut!(FB1),
            &mut *core::ptr::addr_of_mut!(FB2),
        )
    };

    info!(
        "Display buffers allocated at {:x}, {:x}",
        display_buffer_1 as *const _, display_buffer_2 as *const _
    );

    LTDC.layer(0)
        .cfbar()
        .write(|w| w.set_cfbadd(display_buffer_1 as *const _ as u32));

    // Configures the color frame buffer pitch in byte
    LTDC.layer(0).cfblr().write(|w| {
        w.set_cfbp(IMAGE_WIDTH * PIXEL_SIZE as u16);
        w.set_cfbll(((WINDOW_X1 - WINDOW_X0) * PIXEL_SIZE as u16) + 3);
    });

    // Configures the frame buffer line number
    LTDC.layer(0)
        .cfblnr()
        .write(|w| w.set_cfblnbr(IMAGE_HEIGHT));

    // Enable LTDC_Layer by setting LEN bit
    LTDC.layer(0).cr().modify(|w| w.set_len(true));

    //LTDC->SRCR = LTDC_SRCR_IMR;
    LTDC.srcr().modify(|w| w.set_imr(Imr::RELOAD));

    // Delay for 1s
    Timer::after_millis(1000).await;

    // Create a display buffer
    let mut display_fb1 = display_target::DisplayBuffer {
        buf: display_buffer_1,
        width: LCD_X_SIZE as i32,
        height: LCD_Y_SIZE as i32,
    };

    let mut display_fb2 = display_target::DisplayBuffer {
        buf: display_buffer_2,
        width: LCD_X_SIZE as i32,
        height: LCD_Y_SIZE as i32,
    };

    let mut display = &mut display_fb1;

    // Disable the layer
    LTDC.layer(0).cr().modify(|w| w.set_len(false));

    // replace the buffer with the new one
    LTDC.layer(0)
        .cfbar()
        .write(|w| w.set_cfbadd(&display.buf[0] as *const _ as u32));

    // Configures the color frame buffer pitch in byte
    LTDC.layer(0).cfblr().write(|w| {
        w.set_cfbp(IMAGE_WIDTH * 4_u16);
        w.set_cfbll(((WINDOW_X1 - WINDOW_X0) * 4_u16) + 3);
    });

    // Configures the frame buffer line number
    LTDC.layer(0)
        .cfblnr()
        .write(|w| w.set_cfblnbr(IMAGE_HEIGHT));

    // Use RGB565 pixel format
    LTDC.layer(0).pfcr().write(|w| w.set_pf(Pf::RGB565));

    // Enable the layer
    LTDC.layer(0).cr().modify(|w| w.set_len(true));

    // Immediately refresh the display
    LTDC.srcr().modify(|w| w.set_imr(Imr::RELOAD));

    pub fn medsize_rgb565_style() -> kolibri_embedded_gui::style::Style<Rgb565> {
        kolibri_embedded_gui::style::Style {
            background_color: Rgb565::new(0x40, 0x80, 0x40), // pretty dark gray
            item_background_color: Rgb565::new(0x20, 0x40, 0x20), // darker gray
            highlight_item_background_color: Rgb565::new(0x10, 0x20, 0x10),
            border_color: Rgb565::WHITE,
            highlight_border_color: Rgb565::WHITE,
            primary_color: Rgb565::CSS_DARK_CYAN,
            secondary_color: Rgb565::YELLOW,
            icon_color: Rgb565::WHITE,
            text_color: Rgb565::WHITE,
            default_widget_height: 16,
            border_width: 0,
            highlight_border_width: 1,
            default_font: mono_font::iso_8859_10::FONT_9X15,
            spacing: kolibri_embedded_gui::style::Spacing {
                item_spacing: Size::new(8, 4),
                button_padding: Size::new(5, 5),
                default_padding: Size::new(1, 1),
                window_border_padding: Size::new(3, 3),
            },
        }
    }

    let mut i: i32 = 0;
    let mut active_buffer = 0;
    loop {
        // Switch buffers
        if active_buffer == 0 {
            display = &mut display_fb1;
            active_buffer = 1;
        } else {
            display = &mut display_fb2;
            active_buffer = 0;
        }

        // create UI (needs to be done each frame)
        let mut ui = Ui::new_fullscreen(display, medsize_rgb565_style());

        // clear UI background (for non-incremental redrawing framebuffered applications)
        ui.clear_background().ok();

        // === ACTUAL UI CODE STARTS HERE ===

        ui.add(Label::new("Basic Example").with_font(ascii::FONT_10X20));

        ui.add(Label::new("Basic Counter (7LOC)"));

        if ui.add_horizontal(Button::new("-")).clicked() {
            i = i.saturating_sub(1);
        }
        //ui.add_horizontal(Label::new(alloc::format!("Clicked {} times", i).as_ref()));
        if ui.add_horizontal(Button::new("+")).clicked() {
            i = i.saturating_add(1);
        }

        ui.new_row();

        let mut checked = true;
        ui.add(Checkbox::new(&mut checked));

        // replace the buffer with the new one
        LTDC.layer(0)
            .cfbar()
            .write(|w| w.set_cfbadd(&display.buf[0] as *const _ as u32));

        // Immediately refresh the display
        LTDC.srcr().modify(|w| w.set_imr(Imr::RELOAD));

        Timer::after_millis(20).await;
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
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

    info!("SDRAM Initialized at {:x}", ram_ptr as usize);

    // set up the LTDC peripheral to send data to the LCD screen
    // setting timing for AM480272H3TMQW-T01H LCD (MB1046 B-01)
    const AM480272H3TMQW_HSYNC: u16 = 41; // Horizontal synchronization
    const AM480272H3TMQW_HBP: u16 = 13; // Horizontal back porch
    const AM480272H3TMQW_HFP: u16 = 32; // Horizontal front porch
    const AM480272H3TMQW_VSYNC: u16 = 10; // Vertical synchronization
    const AM480272H3TMQW_VBP: u16 = 2; // Vertical back porch
    const AM480272H3TMQW_VFP: u16 = 2; // Vertical front porch

    let ltdc_config = LtdcConfiguration {
        active_width: DISPLAY_WIDTH as _,
        active_height: DISPLAY_HEIGHT as _,
        h_back_porch: AM480272H3TMQW_HBP,
        h_front_porch: AM480272H3TMQW_HFP,
        v_back_porch: AM480272H3TMQW_VBP,
        v_front_porch: AM480272H3TMQW_VFP,
        h_sync: AM480272H3TMQW_HSYNC,
        v_sync: AM480272H3TMQW_VSYNC,
        h_sync_polarity: PolarityActive::ActiveLow,
        v_sync_polarity: PolarityActive::ActiveLow,
        data_enable_polarity: PolarityActive::ActiveLow,
        pixel_clock_polarity: PolarityEdge::RisingEdge,
    };

    let mut ltdc_de = Output::new(p.PK7, Level::Low, Speed::High);
    let mut ltdc_disp_ctrl = Output::new(p.PI12, Level::Low, Speed::High);
    let mut ltdc_bl_ctrl = Output::new(p.PK3, Level::Low, Speed::High);

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
    ltdc_de.set_low();
    ltdc_bl_ctrl.set_high();
    ltdc_disp_ctrl.set_high();

    let layer_config = LtdcLayerConfig {
        pixel_format: ltdc::PixelFormat::RGB565,
        layer: LtdcLayer::Layer1,
        window_x0: 0,
        window_x1: DISPLAY_WIDTH as _,
        window_y0: 0,
        window_y1: DISPLAY_HEIGHT as _,
    };

    // enable the bottom layer
    ltdc.init_layer(&layer_config, None);

    // Start the display task
    let spawner = Spawner::for_current_executor().await;

    spawner.spawn(display_task()).unwrap();

    let mut led = Output::new(p.PI1, Level::High, Speed::Low);

    loop {
        led.set_high();
        Timer::after_millis(1000).await;

        led.set_low();
        Timer::after_millis(1000).await;
    }
}
