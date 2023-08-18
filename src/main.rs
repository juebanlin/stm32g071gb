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
use embedded_graphics::primitives::{Circle, PrimitiveStyleBuilder, Rectangle, Triangle};
use embedded_hal::Direction;
use rtt_target::{rprintln, rtt_init_print};
use ssd1306::{I2CDisplayInterface, prelude::*, Ssd1306};
use stm32g0xx_hal::{cortex_m, stm32 as device};
use stm32g0xx_hal::analog::adc::{OversamplingRatio, Precision, SampleTime, VBat, VTemp};
use stm32g0xx_hal::analog::dac::{Channel1, Enabled, GeneratorConfig};
use stm32g0xx_hal::gpio::{OpenDrain, Output, PB6, PB7};
use stm32g0xx_hal::i2c::I2c;
use stm32g0xx_hal::pac::I2C1;
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
    oled_demo();
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
    loop {

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