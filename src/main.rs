#![no_std]
#![no_main]
#![feature(alloc_error_handler)]
extern crate alloc;

use alloc::format;
use core::alloc::Layout;
use core::fmt::Write;
use core::panic::PanicInfo;

use alloc_cortex_m::CortexMHeap;
use cortex_m::peripheral::SYST;
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
use stm32g0xx_hal::analog::dac::{Channel1, Enabled, GeneratorConfig};
use stm32g0xx_hal::i2c::I2c;
use stm32g0xx_hal::prelude::*;
use stm32g0xx_hal::rcc::Config;
use stm32g0xx_hal::timer::delay::Delay;

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
    button_demo();
}

fn button_demo() -> !{
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = device::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    let mut delay = cp.SYST.delay(&mut rcc);
    let gpioa = dp.GPIOA.split(&mut rcc);
    //上拉输入,引脚处于高电平 按键接地判定低电平即为按键按下
    let button = gpioa.pa7.into_pull_up_input();
    let mut buttonPress=false;
    let dac0 = dp.DAC.constrain(gpioa.pa4, &mut rcc);
    let mut dac = dac0.calibrate_buffer(&mut delay).enable();
    {
        //初始电压设置
        //BKT050V 15A 没有上升沿 5V-48V很快速 下降沿需要30ms以上,48V-5V速度很慢
        let val= dac.get_value();
        //50V 126
        //1V 2979
        //1V ADD 58
        let max_1v=2979u16;
        let value_1V=58u16;
        let value_01v=5.82f32;
        rprintln!("current:{}",val);
        // dac.set_value(max_1v-((value_01v*10f32*(5f32-1f32)) as u16));//5V
        // dac.set_value(max_1v-((value_01v*10f32*(12f32-1f32)) as u16));//12V
        let mut v12=max_1v-((value_01v*10f32*(12f32-1f32)) as u16);
        let mut t=0;
        loop {
            t=t+1;
            delay.delay(1.millis());
            let c=dac.get_value();
            dac.set_value(c+v12/t);
            if t>=35{
                break;
            }
        }
        dac.set_value(v12);
        // dac.set_value(max_1v-((value_01v*10f32*(24f32-1f32)) as u16));//24V
        // dac.set_value(max_1v-((value_01v*10f32*(36f32-1f32)) as u16));//36V
        // dac.set_value(max_1v-((value_01v*10f32*(48f32-1f32)) as u16));//48V
        let val= dac.get_value();
        rprintln!("current:{}",val);
    }
    loop {
        let mut wait;
        match button.is_high() {
            Ok(true) => {
                wait=300.millis();
                buttonPress=false;
            }
            Ok(false) => {
                wait=100.millis();
                buttonPress=true;
            }
            _ => unreachable!(),
        };
        wait=300.millis();
        delay.delay(wait);
        if buttonPress{
            rprintln!("buttonPress:{}",buttonPress);
            changeV(&mut dac,&mut delay);
        }
    }
}

fn changeV(dac: &mut Channel1<Enabled>, delay: &mut Delay<SYST>){
    //0-2.4V 50V-0
    let max=(4095f32*(2.4f32/3.3f32)) as u16;//2978
    let addValue=max/(50*10);//0.1V需要增加的值  0.1V需要5点 每1V需要50点
    let addValue2=max/(50);//每1V需要50点
    let val= dac.get_value();
    rprintln!("max:{},current:{},addValue:{}",max,val,addValue);
    let mut i=0;
    let time=30;
    // let mut v=0;
    // loop {
    //     i+=1;
    //     delay.delay(1.millis());
    //     let val= dac.get_value();
    //     dac.set_value(val+1);
    //     v+=1;
    //     if i>=time {
    //         if(v<addValue){
    //             let off=addValue-v;
    //             dac.set_value(val+off);
    //         }
    //         break;
    //     }
    // }
    loop {
        i+=1;
        delay.delay(6.millis());
        let val= dac.get_value();
        dac.set_value(val+1);
        if i>=addValue{
            break
        }
    }
    let val= dac.get_value();
    rprintln!("max:{},current:{},addValue:{}",max,val,addValue);
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