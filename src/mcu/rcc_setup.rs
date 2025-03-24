use embassy_stm32::time::mhz;
use embassy_stm32::{rcc, Config, Peripherals};

/// Sets up clocks for the stm32f746ng mcu
/// change this if you plan to use a different microcontroller
pub fn stm32f746ng_init() -> Peripherals {
    // setup power and clocks for an 32F746GDISCOVERY run from an external 25 Mhz oscillator
    let mut config = Config::default();

    config.rcc.hse = Some(rcc::Hse {
        freq: mhz(25),
        mode: rcc::HseMode::Oscillator,
    });

    config.rcc.pll = Some(rcc::Pll {
        prediv: rcc::PllPreDiv::DIV25,  // PLL_M Divides the 25MHz HSE to 1MHz
        mul: rcc::PllMul::MUL432,       // PLL_N Multiplies the 1MHz to 432MHz
        divp: Some(rcc::PllPDiv::DIV2), // PLL_P Divides the 432MHz to 216MHz for SYSCLK
        divq: Some(rcc::PllQDiv::DIV9), // PLL_Q Divides the 432MHz to 48MHz for USB, SDMMC and RNG
        divr: None,                     // PLL_R
    });

    config.rcc.pllsai = Some(rcc::Pll {
        prediv: rcc::PllPreDiv::DIV25, // PLL_M Divides the 25MHz HSE to 1MHz
        mul: rcc::PllMul::MUL192,      // PLL_N Multiplies the 1MHz to 192MHz
        divp: None,                    // PLL_P
        divq: None,                    // PLL_Q
        divr: Some(rcc::PllRDiv::DIV4), // PLL_R Divides the 192MHz to 48MHz for LTDC
                                       // LTDC speed according to AN4861 - Table 11
    });

    config.rcc.pll_src = rcc::PllSource::HSE;
    config.rcc.sys = rcc::Sysclk::PLL1_P;
    config.rcc.ahb_pre = rcc::AHBPrescaler::DIV1;
    config.rcc.apb1_pre = rcc::APBPrescaler::DIV4;
    config.rcc.apb2_pre = rcc::APBPrescaler::DIV2;

    embassy_stm32::init(config)
}
