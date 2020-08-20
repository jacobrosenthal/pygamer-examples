//! LIS3DH accelerometer example. Move the neopixel led by tilting left and right.
//!
//! Note accelerometer seems to get stuck sometimes and you'll see a panic led. Until a solution is found,
//! toggle the power power switch.

#![no_std]
#![no_main]

use pygamer as hal;
use pygamer_panic_led as _;

use cortex_m::peripheral::DWT;
use embedded_hal::digital::v1_compat::OldOutputPin;
use hal::clock::GenericClockController;
use hal::gpio;
use hal::sercom::{I2CMaster2, Sercom2Pad0, Sercom2Pad1};
use hal::time::KiloHertz;
use hal::timer::SpinTimer;
use lis3dh::{accelerometer::Accelerometer, Lis3dh, SlaveAddr};
use smart_leds::hsv::{hsv2rgb, Hsv, RGB8};
use smart_leds::SmartLedsWrite;
use ws2812_timer_delay as ws2812;

#[rtic::app(device = crate::hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        neopixel: NeopixelType,
        lis3dh: Lis3dhType,

        #[init(TiltState::new(2.0, 2))]
        state: TiltState,
    }

    #[init]
    fn init(mut c: init::Context) -> init::LateResources {
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
        lis3dh.set_range(lis3dh::Range::G2).unwrap();

        // we update neopixels based on this so cant be too fast
        lis3dh.set_datarate(lis3dh::DataRate::Hz_10).unwrap();

        init::LateResources { neopixel, lis3dh }
    }

    //neopixels are blocking so we need to run them at a low priority. Were not doing an power saving in idle,z
    #[idle(resources = [neopixel, lis3dh, state])]
    fn main(c: main::Context) -> ! {
        loop {
            while !c.resources.lis3dh.is_data_ready().unwrap() {}
            let dat = c.resources.lis3dh.accel_norm().unwrap();

            let (pos, j) = c.resources.state.update(dat.x);

            // iterate through neopixels and paint the one led
            let _ = c.resources.neopixel.write((0..5).map(|i| {
                if i == pos {
                    hsv2rgb(Hsv {
                        hue: j,
                        sat: 255,
                        val: 32,
                    })
                } else {
                    RGB8::default()
                }
            }));
        }
    }

    // RTIC requires that unused interrupts are declared in an extern block when
    // using software tasks; these free interrupts will be used to dispatch the
    // software tasks.
    extern "C" {
        fn SDHC0();
    }
};

pub struct TiltState {
    /// define a band for which -deadzone to +deadzone is considered level
    deadzone: f32,
    /// how many detections before it finally moves. Note this value bhas a relation to your datarate
    friction: u8,
    pos: usize,
    j: u8,
    count: i8,
}

impl TiltState {
    // start at the middle pixel
    const fn new(deadzone: f32, friction: u8) -> TiltState {
        TiltState {
            count: 0,
            friction,
            deadzone,
            pos: 2,
            j: 0,
        }
    }

    fn update(&mut self, value: f32) -> (usize, u8) {
        // first you have to be tilted enough (gt / lt) to be counted
        self.count = if value > self.deadzone {
            self.count + 1
        } else if value < -self.deadzone {
            self.count - 1
        } else {
            self.count
        };

        // then you must overcome friction
        if self.count.abs() as u8 > self.friction {
            //todo use clamp which is nightly
            if self.count.is_negative() {
                //make sure we can move
                if self.pos > 0 {
                    self.pos -= 1;
                    //if we moved, reset friction
                    self.count = 0;
                }
            } else {
                //make sure we can move
                if self.pos < 4 {
                    self.pos += 1;
                    //if we moved, reset friction
                    self.count = 0;
                }
            }
        }

        //incremement the hue easing
        self.j = self.j.wrapping_add(1);

        (self.pos, self.j)
    }
}

type NeopixelType =
    ws2812::Ws2812<hal::timer::SpinTimer, OldOutputPin<gpio::Pa15<gpio::Output<gpio::PushPull>>>>;

type Lis3dhType =
    Lis3dh<I2CMaster2<Sercom2Pad0<gpio::Pa12<gpio::PfC>>, Sercom2Pad1<gpio::Pa13<gpio::PfC>>>>;
