// 导入必要的库
use cortex_m_rt::entry;
use panic_halt as _;
use stm32g0xx_hal::{
    pac,
    prelude::*,
    delay::Delay,
    gpio::{
        gpioc::{self, PC13},
        ExtiPin, Input, PullUp,
    },
};
// 定义风扇引脚
type FANPIN = PC13<Input<PullUp>>;
// 定义计数器和时钟
static mut COUNTER: u32 = 0;
static mut CLOCK: u32 = 0;
#[entry]
fn main() -> ! {
    // 获取设备的外设实例
    let dp = pac::Peripherals::take().unwrap();
    // 获取设备的Core Peripherals实例
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();
    // 配置时钟
    let rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    // 配置延时
    let mut delay = Delay::new(cp.SYST, clocks);
    // 配置GPIO
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);
    let fan_pin = gpioc.pc13.into_pull_up_input(&mut gpioc.crh);
    // 配置外部中断
    let mut exti = dp.EXTI;
    fan_pin.make_interrupt_source(&mut exti);
    fan_pin.trigger_on_edge(&mut exti, stm32g0xx_hal::gpio::Edge::RISING_FALLING);
    fan_pin.enable_interrupt(&mut exti);
    // 启用中断
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::EXTI4_15);
    }
    // 主循环
    loop {
        // 延时一段时间
        delay.delay_ms(1000_u32);
        // 获取计数器和时钟
        let counter = unsafe { &mut COUNTER };
        let clock = unsafe { &mut CLOCK };
        // 计算风扇转速
        let now = cortex_m::Peripherals::take().unwrap().DWT.cyccnt.read();
        let elapsed = now.wrapping_sub(*clock);
        let rpm = (1000000u32 / elapsed) * (*counter);
        // 打印风扇转速
        hprintln!("Fan RPM: {}", rpm).unwrap();
        // 重置计数器和时钟
        *counter = 0;
        *clock = now;
    }
}
// 中断处理函数
#[cortex_m_rt::interrupt]
fn EXTI4_15() {
    // 获取计数器和时钟
    let counter = unsafe { &mut COUNTER };
    let clock = unsafe { &mut CLOCK };
    // 检查是否为上升沿
    let fan_pin = FANPIN::new();
    if fan_pin.is_rising_edge() {
        // 重置计数器
        *counter = 0;
        // 记录时钟
        *clock = cortex_m::Peripherals::take().unwrap().DWT.cyccnt.read();
    } else {
        // 计数器加一
        *counter += 1;
    }
    // 清除中断标志
    fan_pin.clear_interrupt_pending_bit();
}