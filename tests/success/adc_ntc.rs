#![no_std]
#![no_main]
#![feature(alloc_error_handler)]
extern crate alloc;

use alloc::format;
use micromath::F32Ext;
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
    rprintln!("PanicInfo:{}",_info);
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
        let tmpV1=getTmpV1(vbat_mv * 3,pin_mv);
        let tmpV2=getTmpV2(vbat_mv * 3,pin_mv);
        rprintln!("vref:{} | tmp:{} | VBat: {}mV | PA0: {}mV | tmpV1:{} | tmpV2:{}",vref,tmp,vbat_mv * 3, pin_mv,tmpV1,tmpV2);
        delay.delay(1000.millis());
    }
}

fn getTmpV1(vbat_mV:u16,ntc_mV:u16)->u16{
    let VCC=vbat_mV as f32 /1000f32;
    let VOUT=ntc_mV as f32/1000f32;//NTC电阻当前分压值

    let R1=(10*1000)as f32;//10K NTC上方分压定值电阻
    //电阻分压电路图，分压公式为 VOUT=VCC*R_NTC/(R1+R_NTC);
    //变形为 R_NTC=VOUT*R1/(VCC-VOUT);
    let R_NTC=VOUT*R1/(VCC-VOUT);//NTC当前温度下的阻值

    //阻值算温度
    //T1=1 / ( ln(Rt/R_NTC_25)/B + 1/T2 )
    let Rt=R_NTC;
    let R_NTC_25=(50*1000) as f32;//50K 25°时的阻值
    let K=273.15;
    let T2=K+25f32;//25°时的开尔文温度
    let B=3950f32;//NTC b值
    let T1=1f32 / ( (Rt/R_NTC_25).ln()/B + (1f32/T2) );
    let tmp=T1-K;
    rprintln!("VCC:{},VOUT:{},R_NTC:{},tmp:{}",VCC,VOUT,R_NTC,tmp);
    return tmp as u16;
}

fn getTmpV2(vbat_mV:u16,ntc_mV:u16)->u16{
    let VCC=vbat_mV as f32 /1000f32;
    let VOUT=ntc_mV as f32/1000f32;//NTC电阻当前分压值

    let R1=(10*1000)as f32;//10K NTC上方分压定值电阻
    //电阻分压电路图，分压公式为 VOUT=VCC*R_NTC/(R1+R_NTC);变形为 R_NTC=VOUT*R1/(VCC-VOUT);
    let R_NTC=VOUT*R1/(VCC-VOUT);//NTC当前温度下的阻值
    //阻值算温度
    //Steinhart-Hart 方程 1/T=A+Bln(R)+C(ln(R))3
    //https://www.thinksrs.com/downloads/programs/Therm%20Calc/NTCCalibrator/NTCcalculator.htm
    let A=0.8495270643e-3;
    let B=2.180262755e-4;
    let C=1.148651373e-7;
    let K=273.15;
    let R=R_NTC.ln();
    let T1=1f32 /(A+B*R+C*R*R*R);
    let tmp=T1-K;
    rprintln!("VCC:{},VOUT:{},R_NTC:{},tmp:{}",VCC,VOUT,R_NTC,tmp);
    return tmp as u16;
}