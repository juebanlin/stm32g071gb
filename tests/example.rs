#![no_std]
#![no_main]

use core::panic::PanicInfo;
use cortex_m::asm::delay;
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use stm32g0xx_hal::i2c::{BlockingI2c, DutyCycle, Mode};
use stm32g0xx_hal::{cortex_m,timer::Timer, stm32 as device};
use stm32g0xx_hal::prelude::*;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_graphics::mono_font::mapping::Mapping::Iso8859_1;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use stm32g0xx_hal::timer::pwm;

fn ledDemo()->!{
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = device::Peripherals::take().unwrap();
    // //改写复用函数
    // unsafe { dp.GPIOA.afrl.write(|p| p.afsel0().bits(0)); }
    // unsafe { dp.GPIOA.afrl.modify(|_, w| w.afsel5().bits(2)); }
    // unsafe { dp.GPIOA.moder.modify(|_, w| w.moder5().bits(0)); }

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    // 设置时钟总线
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    // 设置通用引脚 (GPIO)
    let mut gpioc = dp.GPIOC.split();
    // LED 对应的 PC13 引脚
    let mut led = gpioc.pc13.into_push_pull_output();
    // 淘宝上有些版本的核心板的 LED 会接在 PB12 引脚上，这样的话用下面两行替换
    // let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    // let mut led = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);
    let mut delay=cp.SYST.delay(&mut clocks);
    // let mut delay2 = dp.TIM1.delay(&clocks);
    hprintln!("开始执行循环");
    loop {
        // 点亮 LED
        led.set_high();
        delay.delay_ms(1000u32);
        // 关闭 LED
        led.set_low();
    }
}

fn pwm()->!{
    use cortex_m::asm;
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = device::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    let gpioa = dp.GPIOA.split(&mut rcc);
    let mut pwm = dp.TIM1.pwm(25.kHz(), &mut rcc);

    let mut pwm_ch1 = pwm.bind_pin(gpioa.pa8);
    let mut pwm_ch2 = pwm.bind_pin(gpioa.pa9);

    let max = pwm_ch1.get_max_duty();
    pwm_ch1.set_duty(max / 2);
    pwm_ch2.set_duty(max / 4);

    pwm_ch1.enable();
    pwm_ch2.enable();
    asm::bkpt();

    pwm_ch1.set_duty(max / 4);
    pwm_ch2.set_duty(max / 8);
    asm::bkpt();

    pwm_ch1.set_duty(max / 8);
    pwm_ch2.set_duty(max / 16);
    asm::bkpt();

    pwm.set_freq(20.kHz());

    loop {}
}

/**
风扇运转时，转子的N,S极切换会产生高低电平变化，转速越快，电平变化也就越快。
从而可以通过变化的频率来侦测风扇的转速。 每分钟的转速=方波频率*（60/磁极对数）
 */
fn rpm() {
    let dp = device::Peripherals::take().unwrap();
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    // Configure the system clock to 72 MHz
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    // Configure PA8 as a PWM output
    let pa8 = gpioa.pa8.into_alternate_push_pull(&mut gpioa.crh);
    let mut pwm = pwm::Pwm::new(dp.TIM1, pa8, clocks, 25.khz());
    let mut pwm = dp.TIM1.pwm(25.kHz(), &mut rcc);

    // Configure PB0 as a PWM input
    let pb0 = gpiob.pb0.into_alternate_push_pull(&mut gpiob.crl);
    let mut timer = Timer::tim3(dp.TIM3, &mut rcc).start_count_down(100.hz());
    let mut pwm_input = pwm::PwmInput::new(timer, pb0, &mut afio.mapr);

    loop {
        let duty = pwm.get_duty() as f32 / pwm.get_max_duty() as f32;
        let freq = pwm_input.get_frequency().unwrap_or(0.0);
        let speed = freq * duty * 60.0 / 2.0;
        println!("Fan speed: {:.2} RPM", speed);
    }
}