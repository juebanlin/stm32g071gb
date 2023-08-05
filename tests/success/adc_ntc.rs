#![no_std]
#![no_main]
#![feature(alloc_error_handler)]
extern crate alloc;

use alloc::format;
use core::alloc::Layout;
use core::fmt::Write;
use core::panic::PanicInfo;
use core::ptr;

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
use stm32g0xx_hal::analog::adc::{OversamplingRatio, Precision, SampleTime, VBat, VTemp};
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
    adc();
}

fn adc()->!{
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = device::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    let mut delay = cp.SYST.delay(&mut rcc);

    let gpioa = dp.GPIOA.split(&mut rcc);
    let mut adc = dp.ADC.constrain(&mut rcc);
    adc.set_sample_time(SampleTime::T_80);
    adc.set_precision(Precision::B_12);
    adc.set_oversampling_ratio(OversamplingRatio::X_16);
    adc.set_oversampling_shift(16);
    adc.oversampling_enable(true);

    delay.delay(20.micros()); // Wait for ADC voltage regulator to stabilize
    adc.calibrate();

    let mut adc_pin = gpioa.pa0.into_analog();
    let mut vbat = VBat::new();
    vbat.enable(&mut adc);
    let mut vtmp=VTemp::new();
    vtmp.enable(&mut adc);
    //     NTC和定值电阻R串联，当环境温度发生变化后，NTC的电阻值发生变化，
    // 导致NTC两端的电压发生变化，单片机通过采集NTC两端的电压就可以反推出当前的温度值
    loop {
        let vref =  adc.read_vref().expect("adc read failed");
        let tmp=adc.read_temperature().expect("tmp read failed");
        let vbat_mv = adc.read_voltage(&mut vbat).expect("adc read failed");
        let pin_mv = adc.read_voltage(&mut adc_pin).expect("adc read failed");
        //10K串联NTC,NTC接GND   VCC--10K--PA0-NTC--GND
        // let rt=pin_mv/100*(vbat_mv/100-pin_mv/100);
        // let ntc_tmp=1/(1/298.15+(rt/10)/3950)-273.15;
        let mut pin_tmp=0;
        {
            // let ts_cal1: u32 = unsafe {
            //     // DS12991 3.14.1
            //     // at 3000 mV Vref+ and 30 degC
            //     ptr::read_volatile(0x1FFF_75A8 as *const u16) as u32
            // };
            let v30 = 2600; // mV
            // 2.5 mV/degC
            pin_tmp= 30 + (v30 as i32 - pin_mv as i32)/3 * 10 / 25;
        }
        rprintln!("vref:{} | tmp:{} | VBat: {}mV | PA0: {}mV | pin_tmp:{}",vref,tmp,vbat_mv * 3, pin_mv,pin_tmp);
        delay.delay(1000.millis());
    }
}
