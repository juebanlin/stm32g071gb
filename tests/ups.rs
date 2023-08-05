use stm32f1xx_hal::{
    gpio::{Output, PushPull},
    pac,
    prelude::*,
    usb::{Peripheral, UsbBus, UsbBusType},
};
use usb_device::{
    class_prelude::*,
    prelude::*,
    Result,
};
//定义HID Power Device报告
#[derive(Default)]
pub struct HidPowerReport {
    pub input_voltage: u16,
    pub input_current: u16,
    pub output_voltage: u16,
    pub output_current: u16,
    pub battery_voltage: u16,
    pub battery_capacity: u8,
}
//定义HID Power Device命令和通知
#[derive(Clone, Copy)]
pub enum HidPowerCommand {
    EnterSleep,
    WakeUp,
}
#[derive(Clone, Copy)]
pub enum HidPowerNotification {
    Sleep,
    WakeUp,
}
//定义HID Power Device描述符
#[derive(Default)]
pub struct HidPowerDescriptor {
    pub input_voltage_range: u16,
    pub input_current_range: u16,
    pub output_voltage_range: u16,
    pub output_current_range: u16,
    pub battery_voltage_range: u16,
    pub battery_capacity_range: u8,
}
impl UsbClass for HidPowerDevice<UsbBusType> {
    fn get_configuration_descriptors(
        &self,
        writer: &mut DescriptorWriter,
    ) -> Result<()> {
        writer.interface(
            0x01, // interface number
            0x03, // interface class: HID
            0x00, // subclass
            0x00, // protocol
        )?;
        // HID descriptor
        writer.write(
            0x21, // descriptor type: HID
            &[
                0x11, 0x01, // HID version 1.11
                0x00, // target country
                0x01, // number of HID class descriptors
                0x22, // descriptor type: report
                0x3F, 0x00, // total length of report descriptor
            ],
        )?;
        // endpoint descriptor
        writer.endpoint(&self.ep_out)?;
        Ok(())
    }
    fn control_in(&mut self, xfer: ControlIn<B>) {
        match xfer.request() {
            // HID Power Device descriptor
            Request::GetDescriptor {
                descriptor_type: 0x21,
                index: 0x00,
                lang_id: _,
            } => {
                xfer.accept_with(&self.descriptor.as_bytes()).unwrap();
            }
            _ => {
                xfer.reject().unwrap();
            }
        }
    }
    fn control_out(&mut self, xfer: ControlOut<B>) {
        match xfer.request() {
            // HID Power Device command
            Request::Class {
                interface: 0x01,
                request: 0x09,
                value,
                index: _,
            } if value == 0x01 => {
                self.handle_command(HidPowerCommand::EnterSleep);
                xfer.accept().unwrap();
            }
            Request::Class {
                interface: 0x01,
                request: 0x09,
                value,
                index: _,
            } if value == 0x02 => {
                self.handle_command(HidPowerCommand::WakeUp);
                xfer.accept().unwrap();
            }
            _ => {
                xfer.reject().unwrap();
            }
        }
    }
}
//定义HID Power Device UPS设备
pub struct HidPowerDevice<B: UsbBus> {
    device: UsbDevice<'static, B>,
    ep_out: EndpointOut<'static, B>,
    report: HidPowerReport,
    descriptor: HidPowerDescriptor,
}
impl<B: UsbBus> HidPowerDevice<B> {
    pub fn new(usb_bus: &UsbBusAllocator<B>) -> HidPowerDevice<B> {
        let mut ep_out = EndpointOut::new(&usb_bus);
        HidPowerDevice {
            device: UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16C0, 0x27DD))
                .manufacturer("My manufacturer")
                .product("My UPS")
                .serial_number("SN1234")
                .device_class(0xEF)
                .device_sub_class(0x02)
                .device_protocol(0x01)
                .max_packet_size_0(64)
                .build(),
            ep_out: ep_out,
            report: HidPowerReport::default(),
            descriptor: HidPowerDescriptor::default(),
        }
    }
    pub fn handle_command(&mut self, cmd: HidPowerCommand) {
        match cmd {
            HidPowerCommand::EnterSleep => {
                // put device into sleep mode
                self.send_notification(HidPowerNotification::Sleep);
            }
            HidPowerCommand::WakeUp => {
                // wake device up from sleep mode
                self.send_notification(HidPowerNotification::WakeUp);
            }
        }
    }
    pub fn send_notification(&mut self, notification: HidPowerNotification) {
        match notification {
            HidPowerNotification::Sleep => {
                // send sleep notification to host
                self.ep_out.write(&[0x02]).unwrap();
            }
            HidPowerNotification::WakeUp => {
                // send wake-up notification to host
                self.ep_out.write(&[0x03]).unwrap();
            }
        }
    }
    pub fn update_report(&mut self) {
        // update HID Power Device report based on UPS device status
        self.report.input_voltage = 220;
        self.report.input_current = 10;
        self.report.output_voltage = 220;
        self.report.output_current = 5;
        self.report.battery_voltage = 12;
        self.report.battery_capacity = 80;
    }
}
//定义STM32的GPIO引脚
type Pin1 = stm32f1xx_hal::gpio::gpioc::PC13<Output<PushPull>>;
//定义UPS设备
pub struct UpsDevice {
    led: Pin1,
}
impl UpsDevice {
    pub fn new(mut led: Pin1) -> Self {
        led.set_low().unwrap();
        UpsDevice { led }
    }
    pub fn monitor_power(&mut self) {
        //监测电源电压和电流
        //如果电源电压过低或电流过高，就要控制UPS设备关闭输出电源
        if true {
            self.led.set_high().unwrap();
        } else {
            self.led.set_low().unwrap();
        }
    }
}
fn main() {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);
    let usb_dm = gpioc.pc12.into_alternate_push_pull(&mut gpioc.crh);
    let usb_dp = gpioc.pc11.into_alternate_push_pull(&mut gpioc.crh);
    let usb_bus = UsbBus::new(dp.USB, (usb_dm, usb_dp));
    let mut hid_power_device = HidPowerDevice::new(&usb_bus);
    let mut ups_device = UpsDevice::new(gpioc.pc13.into_push_pull_output(&mut gpioc.crh));
    loop {
        hid_power_device.update_report();
        ups_device.monitor_power();
        if !hid_power_device.device.poll(&mut [hid_power_device.ep_out]) {
            continue;
        }
        hid_power_device.ep_out.read(|data| {
            if data[0] == 0x01 {
                // host requested HID Power Device report
                let report = &hid_power_device.report;
                hid_power_device.ep_out.write(&[
                    0x01,
                    report.input_voltage as u8,
                    (report.input_voltage >> 8) as u8,
                    report.input_current as u8,
                    (report.input_current >> 8) as u8,
                    report.output_voltage as u8,
                    (report.output_voltage >> 8) as u8,
                    report.output_current as u8,
                    (report.output_current >> 8) as u8,
                    report.battery_voltage as u8,
                    (report.battery_voltage >> 8) as u8,
                    report.battery_capacity,
                ]).unwrap();
            }
        });
    }
}