#![no_std]
#![no_main]
extern crate alloc;

use alloc::format;
use core::fmt::Write;
use core::panic::PanicInfo;
use alloc_cortex_m::CortexMHeap;
use cortex_m::asm::delay;
use cortex_m::peripheral::SYST;
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use stm32g0xx_hal::i2c::{I2c};
use stm32g0xx_hal::{cortex_m,timer::Timer, stm32 as device};
use stm32g0xx_hal::prelude::*;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10,mapping::Mapping::Iso8859_1, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_graphics::primitives::{Circle, PrimitiveStyleBuilder, Rectangle, Triangle};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use stm32g0xx_hal::gpio::{OpenDrain, Output, PB6, PB7, PB8};
use stm32g0xx_hal::pac::I2C1;
use stm32g0xx_hal::spi::Mode;
use stm32g0xx_hal::timer::delay::Delay;

// this is the allocator the application will use
#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();
const HEAP_SIZE: usize = 1024; // in bytes

// #[panic_handler]
// fn panic_(_info:&PanicInfo)->!{
//     loop {
//
//     }
// }

#[entry]
fn main() -> ! {
    // 初始化内存分配器
    unsafe { ALLOCATOR.init(cortex_m_rt::heap_start() as usize, HEAP_SIZE) }
    oled_demo();
}

fn oled_demo()->!{
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = device::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    let mut delay = cp.SYST.delay(&mut rcc);

    let mut gpiob = dp.GPIOB.split(&mut rcc);
    //PB6 USART1_TX, TIM1_CH3, TIM16_CH1N, SPI2_MISO, LPTIM1_ETR, I2C1_SCL, EVENTOUT
    //PB7 USART1_RX, SPI2_MOSI, TIM17_CH1N, LPTIM1_IN2, I2C1_SDA, EVENTOUT
    //PB8 SPI2_SCK, TIM16_CH1, I2C1_SCL, EVENTOUT
    //AF6
    let scl = gpiob.pb6.into_open_drain_output_in_state(PinState::High);
    let sda = gpiob.pb7.into_open_drain_output_in_state(PinState::High);
    let i2c =I2c::i2c1(dp.I2C1,sda,scl,400.kHz(),&mut rcc);
    let interface = I2CDisplayInterface::new(i2c);
    delay.delay_ms(1000u32);
    let txt=format!("Hello world!:{}",11);
    drow(interface,&mut delay);
    // drowTxt(txt.as_str(),interface);
    // drowNum(11,interface);
    let mut i=0;
    loop {
        i+=1;
        rprintln!("{}",i);
        delay.delay_ms(1000u32);
    }
}

/*
一个长方形里面绘制一个三角形 正方形 圆形
 */
fn drow(interface: I2CInterface<I2c<I2C1, PB7<Output<OpenDrain>>, PB6<Output<OpenDrain>>>>, delay: &mut Delay<SYST>){
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let yoffset = 8;
    let style = PrimitiveStyleBuilder::new()
        .stroke_width(1)
        .stroke_color(BinaryColor::On)
        .build();
    let mut n=0;
    loop {
        // screen outline
        // default display size is 128x64 if you don't pass a _DisplaySize_
        // enum to the _Builder_ struct
        Rectangle::new(Point::new(0, 0), Size::new(127, 31))
            .into_styled(style)
            .draw(&mut display)
            .unwrap();

        // triangle
        Triangle::new(
            Point::new(16, 16 + yoffset),
            Point::new(16 + 16, 16 + yoffset),
            Point::new(16 + 8, yoffset),
        )
            .into_styled(style)
            .draw(&mut display)
            .unwrap();

        if n%2==0 {
            // square
            Rectangle::new(Point::new(52, yoffset), Size::new_equal(16))
                .into_styled(style)
                .draw(&mut display)
                .unwrap();
            // circle
            Circle::new(Point::new(88, yoffset), 16)
                .into_styled(style)
                .draw(&mut display)
                .unwrap();
        }else{
            // circle
            Circle::new(Point::new(52, yoffset), 16)
                .into_styled(style)
                .draw(&mut display)
                .unwrap();
            // square
            Rectangle::new(Point::new(88, yoffset), Size::new_equal(16))
                .into_styled(style)
                .draw(&mut display)
                .unwrap();
        }
        display.flush().unwrap();
        delay.delay_ms(100u32);
        display.clear();
        n+=1;
    }
}

fn drowTxt(txt:&str, interface: I2CInterface<I2c<I2C1, PB7<Output<OpenDrain>>, PB6<Output<OpenDrain>>>>){
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();
    Text::with_baseline(txt, Point::zero(), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();
    display.flush().unwrap();
}

fn drowNum(number:i32,interface: I2CInterface<I2c<I2C1, PB7<Output<OpenDrain>>, PB6<Output<OpenDrain>>>>){
    let mut display =
        Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0).into_terminal_mode();
    display.init().unwrap();
    display.clear();
    let s1="1";
    let s2="2";
    // let bytes=number.to_be_bytes();
    // display.draw(&bytes);
    // let str = unsafe {
    //     core::str::from_utf8_unchecked(&bytes)
    // };
    // let _ = display.clear();
    // display.write_str(str);
    // for c in 97..123 {
    //     let _ = display.write_str(unsafe { core::str::from_utf8_unchecked(&[c]) });
    // }
    for i in 1..20 {
        display.write_str(unsafe { core::str::from_utf8_unchecked(&[i]) });
        // let c=unsafe{core::char::from_u32_unchecked(11u32)};
        // display.write_char( c);
    }
}
