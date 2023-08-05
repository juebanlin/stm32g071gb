#![no_std]
#![no_main]
#![feature(alloc_error_handler)]
extern crate alloc;

use alloc::format;
use core::alloc::Layout;
use core::fmt::Write;
use core::panic::PanicInfo;

use alloc_cortex_m::CortexMHeap;
use cortex_m_rt::entry;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_hal::Direction;
use rtt_target::{rprintln, rtt_init_print};
use ssd1306::{I2CDisplayInterface, prelude::*, Ssd1306};
use stm32g0xx_hal::{cortex_m, stm32 as device};
use stm32g0xx_hal::analog::dac::GeneratorConfig;
use stm32g0xx_hal::i2c::I2c;
use stm32g0xx_hal::prelude::*;
use stm32g0xx_hal::rcc::Config;

// this is the allocator the application will use
#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();
const HEAP_SIZE: usize = 512;

//内存分配错误处理
#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    loop {}
}

#[panic_handler]
fn panic_(_info:&PanicInfo) -> !{
    loop {

    }
}

#[entry]
fn main() -> ! {
    // 初始化内存分配器
    // Initialize the allocator BEFORE you use it
    {
        use core::mem::MaybeUninit;
        static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { ALLOCATOR.init(HEAP.as_ptr() as usize, HEAP_SIZE) }
    }
    rtt_init_print!();
    rprintln!("rtt init success!");
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
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    // Configure PB4 as a PWM output
    let mut pwm = dp.TIM3.pwm(25.kHz(), &mut rcc);
    let mut pwm_output = pwm.bind_pin(gpiob.pb4);
    pwm_output.enable();
    let pwm_output_max_duty = pwm_output.get_max_duty();

    let mut i=0;
    loop {
        i+=1;
        delay.delay_ms(1000u32);

        let mut per1=i%10+1;
        per1=per1*10;
        if per1<=50 {
            per1=10;
        }else{
            per1=100;
        }
        let per2=per1 as f32/100.0;
        let duty= pwm_output_max_duty *per2 as u32;
        pwm_output.set_duty(duty);

        let txt=format!("p:{},{},d:{}",per1,per2,duty);
        rprintln!("txt:{}",txt);
        //view
        display.clear();
        display.flush().unwrap();
        Text::with_baseline(&txt, Point::zero(), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        display.flush().unwrap();
    }
}