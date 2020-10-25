//! Port of Tensorflow Gesture Demo
//! https://learn.adafruit.com/tensorflow-lite-for-edgebadge-kit-quickstart/gesture-demo
//!
//! With the screen facing you, and the USB port pointing to the ceiling perform
//! one of three gestures. See the linked video for a demonstration of the gestures.
//!
//! Wing: This gesture is a W starting at your top left, going down, up, down up
//! to your top right When that gesture is detected you'lll see the front
//! NeoPixels turn yellow.
//!
//! Ring: This gesture is a O starting at top center, then moving clockwise in a
//! circle to the right, then down, then left and back to when you started in
//! the top center When that gesture is detected you'll see the front NeoPixels
//! turn purple.
//!
//! Slope: This gesture is an L starting at your top right, moving diagonally to
//! your bottom left, then straight across to bottom right. When that gesture is
//! detected you'll see the front NeoPixels turn light blue.
//!
//! Setup:
//! * figure out how to install arm-none-eabi-gcc for your os
//! * `rustup update` to get a recent nightly near august
//!
//! Upload:
//! * `cargo hf2 --example magic_wand --features=tf`
//!
//! You can get feedback from the device via serial over usb using 115200 baud
//!
//! Try Test data to confirm model works at all you can ignore accel data and
//! infer using test data by using one of tf_test_slope or tf_test_ring features
//!
//! Record training Data by using the train feature
//! * `cargo hf2 --example magic_wand --features=tf,tf_train`
//!
//! Note accelerometer seems to get stuck uploading. Until a solution is found,
//! if the device doesn't use panic led, and unplug or toggle power switch.
//!
//! Note leds timing gets wonky sometimes and give odd result, ignore that.

#![no_std]
#![no_main]

use pygamer as hal;
use pygamer_panic_led as _;

use cortex_m::peripheral::DWT;
use embedded_hal::{digital::v1_compat::OldOutputPin, timer::CountDown};
use hal::gpio;
use hal::prelude::*;
use hal::sercom::{I2CMaster2, Sercom2Pad0, Sercom2Pad1};
use hal::time::KiloHertz;
use hal::usb::UsbBus;
use hal::{clock::GenericClockController, timer::TimerCounter};
use lis3dh::{accelerometer::RawAccelerometer, Lis3dh, SlaveAddr};
use smart_leds::{brightness, colors, hsv::RGB8, SmartLedsWrite};
// use ufmt::uwriteln;
use core::fmt::Write;
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use ws2812_timer_delay as ws2812;
// use core::io::Write as IoWrite;
use tfmicro::{MicroInterpreter, Model, MutableOpResolver};

#[cfg(not(feature = "tf_train"))]
const N: usize = 128;

#[cfg(feature = "tf_train")]
const N: usize = 64;

const TENSOR_ARENA_SIZE: usize = 60 * 1024;

#[rtic::app(device = crate::hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        usb_bus: &'static UsbBusAllocator<UsbBus>,
        usb_serial: SerialPort<'static, UsbBus>,
        usb_device: UsbDevice<'static, UsbBus>,

        neopixel: NeopixelType,
        lis3dh: Lis3dhType,

        #[init([0; TENSOR_ARENA_SIZE])]
        tensor_arena: [u8; TENSOR_ARENA_SIZE],
    }

    #[init(resources = [tensor_arena])]
    fn init(mut c: init::Context) -> init::LateResources {
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

        let gclk0 = clocks.gclk0();
        let timer_clock = clocks.tc2_tc3(&gclk0).unwrap();
        let mut timer = TimerCounter::tc3_(&timer_clock, peripherals.TC3, &mut peripherals.MCLK);
        timer.start(3_000_000u32.hz());

        let neopixel = pins.neopixel.init(timer, &mut pins.port);

        // Initialize (enable) the monotonic timer (CYCCNT)
        c.core.DCB.enable_trace();
        // required on Cortex-M7 devices that software lock the DWT (e.g. STM32F7)
        DWT::unlock();
        c.core.DWT.enable_cycle_counter();

        let i2c = pins.i2c.init(
            &mut clocks,
            KiloHertz(400),
            peripherals.SERCOM2,
            &mut peripherals.MCLK,
            &mut pins.port,
        );

        let mut lis3dh = Lis3dh::new(i2c, SlaveAddr::Alternate).unwrap();
        lis3dh.set_mode(lis3dh::Mode::HighResolution).unwrap();
        lis3dh.set_range(lis3dh::Range::G4).unwrap();
        lis3dh.set_datarate(lis3dh::DataRate::Hz_25).unwrap();

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
            lis3dh,
        }
    }

    //neopixels and inference are blocking so we need to run them at a low
    //priority. Were not doing an power saving in idle, so well use that instead
    //of scheduling a task
    #[idle(resources = [neopixel, lis3dh, usb_serial, tensor_arena])]
    fn main(c: main::Context) -> ! {
        let neopixel = c.resources.neopixel;
        let lis3dh = c.resources.lis3dh;
        let mut usb_serial = c.resources.usb_serial;

        //  (x,y,z)
        let mut data = [0.0; N * 3];

        let model = include_bytes!("./models/magic_wand.tflite");

        #[cfg(feature = "tf_test_ring")]
        let test = include_bytes!("../models/ring_micro_f9643d42_nohash_4.data")
            .chunks_exact(4)
            .map(|c| f32::from_be_bytes([c[0], c[1], c[2], c[3]]))
            .collect::<heapless::Vec<_, heapless::consts::U384>>();

        #[cfg(feature = "tf_test_slope")]
        let test = include_bytes!("../models/slope_micro_f2e59fea_nohash_1.data")
            .chunks_exact(4)
            .map(|c| f32::from_be_bytes([c[0], c[1], c[2], c[3]]))
            .collect::<heapless::Vec<_, heapless::consts::U384>>();

        // Map the model into a usable data structure. This doesn't involve
        // any copying or parsing, it's a very lightweight operation.
        let model = Model::from_buffer(&model[..]).unwrap();

        // Pull in all needed operation implementations
        let micro_op_resolver = MutableOpResolver::empty()
            .depthwise_conv_2d()
            .max_pool_2d()
            .conv_2d()
            .fully_connected()
            .softmax();

        let arena: &mut [u8; 61440] = c.resources.tensor_arena;

        // Build an interpreter to run the model with
        //hard faults in init?
        let mut interpreter =
            MicroInterpreter::new(&model, micro_op_resolver, &mut arena[..]).unwrap();

        // Check properties of the input sensor
        assert_eq!([1, 128, 3, 1], interpreter.input_info(0).dims);

        loop {
            #[cfg(not(feature = "tf_train"))]
            let _ = usb_serial.lock(|serial| writeln!(DirtyWriter(serial), "Magic Starts!\r"));
            neopixel
                .write(brightness(
                    [
                        colors::GREEN,
                        colors::GREEN,
                        colors::GREEN,
                        colors::GREEN,
                        colors::GREEN,
                    ]
                    .iter()
                    .cloned(),
                    1,
                ))
                .unwrap();

            (0..N).for_each(|n| {
                while !lis3dh.is_data_ready().unwrap() {}
                let dat = lis3dh.accel_raw().unwrap();

                // test data is normalized to 1mg per digit
                // shift to justify, .002 scale, *1000 for to mg
                let x = (dat[0] >> 4) as f32 * 20.0;
                let y = (dat[1] >> 4) as f32 * 20.0;
                let z = (dat[2] >> 4) as f32 * 20.0;

                // invert and move around for our board orientation
                data[n * 3] = -z;
                data[n * 3 + 1] = -x;
                data[n * 3 + 2] = y;
            });

            #[cfg(feature = "tf_train")]
            let color = {
                let _ = usb_serial.lock(|serial| {
                    writeln!(DirtyWriter(serial), "-,-,-\r").ok();
                    writeln!(DirtyWriter(serial), "{:04.1?}\r", data).ok();
                });

                RGB8::default()
            };

            #[cfg(not(feature = "tf_train"))]
            let color = {
                let _ = usb_serial.lock(|serial| {
                    writeln!(DirtyWriter(serial), "{:04.1?}\r", data).ok();
                });

                #[cfg(not(any(feature = "tf_test_ring", feature = "tf_test_slope")))]
                interpreter.input(0, &data).ok();

                #[cfg(any(feature = "tf_test_ring", feature = "tf_test_slope"))]
                interpreter.input(0, &test).ok();

                interpreter.invoke().ok();

                let output_tensor = interpreter.output(0);
                assert_eq!([1, 4], output_tensor.info().dims);

                let res = output_tensor.as_data::<f32>();

                let _ = usb_serial.lock(|serial| {
                    writeln!(DirtyWriter(serial), "{:04.1?}\r", res).ok();
                });

                // 0 WingScore
                // 1 RingScore
                // 2 SlopeScore
                // 3 NegativeScore
                let color = if res[0] > 0.5 {
                    colors::YELLOW
                } else if res[1] > 0.5 {
                    colors::PURPLE
                } else if res[2] > 0.5 {
                    colors::BLUE
                } else {
                    RGB8::default()
                };

                color
            };

            let _ = neopixel.write(brightness(
                [color, color, color, color, color].iter().cloned(),
                1,
            ));

            //120mhz, 120_000_000 cycles is a second * 3 = 360000000
            //but thats.. too long? 120_000_000 is about 3 seconds
            cortex_m::asm::delay(120000000);
        }
    }

    #[task(binds = USB_OTHER, resources = [usb_device, usb_serial])]
    fn usb_other(cx: usb_other::Context) {
        usb_poll(cx.resources.usb_device, cx.resources.usb_serial);
    }

    #[task(binds = USB_TRCPT0, resources = [usb_device, usb_serial])]
    fn usb_trcpt0(cx: usb_trcpt0::Context) {
        usb_poll(cx.resources.usb_device, cx.resources.usb_serial);
    }

    #[task(binds = USB_TRCPT1, resources = [usb_device, usb_serial])]
    fn usb_trcpt1(cx: usb_trcpt1::Context) {
        usb_poll(cx.resources.usb_device, cx.resources.usb_serial);
    }

    // RTIC requires that unused interrupts are declared in an extern block when
    // using software tasks; these free interrupts will be used to dispatch the
    // software tasks. Chosen randomly, feel free to replace.
    extern "C" {
        fn SDHC0();
    }
};

// throw away incoming
fn usb_poll<B: usb_device::bus::UsbBus>(
    usb_dev: &mut UsbDevice<'static, B>,
    serial: &mut SerialPort<'static, B>,
) {
    if !usb_dev.poll(&mut [serial]) {
        return;
    }
    let mut buf = [0; 10];
    match serial.read(&mut buf) {
        Ok(_) => {}
        Err(UsbError::WouldBlock) => {}
        e => panic!("USB read error: {:?}", e),
    }
}

type NeopixelType = ws2812::Ws2812<
    hal::timer::TimerCounter3,
    OldOutputPin<gpio::Pa15<gpio::Output<gpio::PushPull>>>,
>;

type Lis3dhType =
    Lis3dh<I2CMaster2<Sercom2Pad0<gpio::Pa12<gpio::PfC>>, Sercom2Pad1<gpio::Pa13<gpio::PfC>>>>;

pub struct DirtyWriter<'a, B: 'static + usb_device::bus::UsbBus>(&'a mut SerialPort<'static, B>);

// core::fmt machinery is laughably large, but ufmt doesnt support f32..
// impl<'a, B: usb_device::bus::UsbBus> ufmt::uWrite for DirtyWriter<'a, B> {
//     type Error = usb_device::UsbError;
//     fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
//         match self.0.write(&s.as_bytes()) {
//             Ok(_) => Ok(()),
//             Err(UsbError::WouldBlock) => Ok(()),
//             Err(e) => Err(e),
//         }
//     }
// }

impl<'a, B: usb_device::bus::UsbBus> core::fmt::Write for DirtyWriter<'a, B> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        match self.0.write(&s.as_bytes()) {
            Ok(_) => Ok(()),
            Err(UsbError::WouldBlock) => Ok(()),
            Err(_) => Err(core::fmt::Error),
        }
    }
}
