//! Makes the pygamer appear as a USB serial port. The color of the
//! neopixel LED can be changed by sending bytes to the serial port.
//!
//! Sending the characters R, G, and O set the LED red, green, and off
//! respectively. For example:
//! $> sudo stty -F /dev/ttyACM0 115200 raw -echo
//! $> sudo bash -c "echo 'R' > /dev/ttyACM0"
//! $> sudo bash -c "echo 'G' > /dev/ttyACM0"
//! $> sudo bash -c "echo 'O' > /dev/ttyACM0"

#![no_std]
#![no_main]

use pygamer as hal;
use pygamer_panic_led as _;

use embedded_hal::digital::v1_compat::OldOutputPin;
use hal::clock::GenericClockController;
use hal::gpio;
use hal::timer::SpinTimer;
use hal::usb::UsbBus;
use smart_leds::{colors, hsv::RGB8, SmartLedsWrite};
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use ws2812_timer_delay as ws2812;

#[rtic::app(device = crate::hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        usb_bus: &'static UsbBusAllocator<UsbBus>,
        usb_serial: usbd_serial::SerialPort<'static, UsbBus>,
        usb_device: UsbDevice<'static, UsbBus>,
        neopixel: NeopixelType,
    }

    #[init]
    fn init(c: init::Context) -> init::LateResources {
        static mut USB_BUS: Option<UsbBusAllocator<UsbBus>> = None;

        let mut peripherals = c.device;
        let mut clocks = GenericClockController::with_internal_32kosc(
            peripherals.GCLK,
            &mut peripherals.MCLK,
            &mut peripherals.OSC32KCTRL,
            &mut peripherals.OSCCTRL,
            &mut peripherals.NVMCTRL,
        );

        let mut pins = hal::Pins::new(peripherals.PORT).split();

        // neopixels
        let timer = SpinTimer::new(4);
        let mut neopixel = pins.neopixel.init(timer, &mut pins.port);

        let _ = neopixel.write((0..5).map(|_| RGB8::default()));

        *USB_BUS = Some(pins.usb.init(
            peripherals.USB,
            &mut clocks,
            &mut peripherals.MCLK,
            &mut pins.port,
        ));

        let usb_serial = SerialPort::new(USB_BUS.as_ref().unwrap());

        let usb_device =
            UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
                .manufacturer("Fake company")
                .product("Serial port")
                .serial_number("TEST")
                .device_class(USB_CLASS_CDC)
                .build();

        init::LateResources {
            usb_bus: USB_BUS.as_ref().unwrap(),
            usb_serial,
            usb_device,
            neopixel,
        }
    }

    #[task(capacity = 1 ,resources = [neopixel])]
    fn display(c: display::Context, color: RGB8) {
        let _ = c.resources.neopixel.write((0..5).map(|_| color));
    }

    #[task(binds = USB_OTHER, resources = [usb_device, usb_serial], spawn = [display])]
    fn usb_other(cx: usb_other::Context) {
        if let Some(color) = usb_poll(cx.resources.usb_device, cx.resources.usb_serial) {
            let _ = cx.spawn.display(color);
        };
    }

    #[task(binds = USB_TRCPT0, resources = [usb_device, usb_serial], spawn = [display])]
    fn usb_trcpt0(cx: usb_trcpt0::Context) {
        if let Some(color) = usb_poll(cx.resources.usb_device, cx.resources.usb_serial) {
            let _ = cx.spawn.display(color);
        };
    }

    #[task(binds = USB_TRCPT1, resources = [usb_device, usb_serial], spawn = [display])]
    fn usb_trcpt1(cx: usb_trcpt1::Context) {
        if let Some(color) = usb_poll(cx.resources.usb_device, cx.resources.usb_serial) {
            let _ = cx.spawn.display(color);
        };
    }

    // RTIC requires that unused interrupts are declared in an extern block when
    // using software tasks; these free interrupts will be used to dispatch the
    // software tasks. Chosen randomly, feel free to replace.
    extern "C" {
        fn SDHC0();
    }
};

fn usb_poll<B: usb_device::bus::UsbBus>(
    usb_dev: &mut UsbDevice<'static, B>,
    serial: &mut SerialPort<'static, B>,
) -> Option<RGB8> {
    if !usb_dev.poll(&mut [serial]) {
        return None;
    }

    // capacity to drain entire buffer
    let mut buf = [0u8; 64];

    // neopixel is blocking so lets just return the color and spawn the display in another priority
    if let Ok(count) = serial.read(&mut buf) {
        // but only care about thethe last character
        let color = match buf[count - 1] as char {
            'R' => colors::RED,
            'G' => colors::GREEN,
            'O' => colors::ORANGE,
            _ => RGB8::default(),
        };

        Some(color)
    } else {
        None
    }
}

type NeopixelType =
    ws2812::Ws2812<hal::timer::SpinTimer, OldOutputPin<gpio::Pa15<gpio::Output<gpio::PushPull>>>>;
