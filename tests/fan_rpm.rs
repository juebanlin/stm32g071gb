use std::time::Instant;
use stm32g0xx_hal::{prelude::*, pac::{TIM1, NVIC, Interrupt, RCC}, timer::{self, Event, Timer}, gpio::{self, Input, PullUp}, pac};
use stm32g0xx_hal::exti::Event;
use stm32g0xx_hal::gpio::SignalEdge;
use stm32g0xx_hal::time::MicroSecond;

fn main() {
    // 获取MCU实例
    let mut dp = pac::Peripherals::take().unwrap();
    // 启用CIC时钟
    let mut rcc = dp.RCC.constrain();
    let _ = rcc.cfgr.freeze();

    // 配置GPIO端口
    let gpioa = dp.GPIOB.split();
    let fan_pin = gpioa.pb5.into_pull_up_input();

    let mut timer1 = Timer::tim3(dp.TIM3, &mut rcu);
    let mut counter = 0;
    let mut time_since_last_event = 0;
    let mut last_event = Instant::now();

    while let Some(_) = timer1.wait().ok() {
        let now = Instant::now();
        let elapsed = now.duration_since(last_event);//距离上次的间隔
        last_event = now;
        if fan_pin.is_high().unwrap() {
            counter += 1;
            time_since_last_event = elapsed.as_micros();
        }
        if elapsed.as_secs() > 1 {
            let frequency = (counter as f32) / (time_since_last_event as f32 * 1E-6);
            counter = 0;
            println!("Frequency: {}", frequency);
        }
    }
}
