//! LIS3DH accelerometer example. Move the neopixel led by tilting left and right.
//!
//! Note accelerometer seems to get stuck sometimes and you'll see a panic led. Until a solution is found,
//! toggle the power power switch.

#![no_std]
#![no_main]

use pygamer as hal;
use pygamer_panic_led as _;

use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::pac::{CorePeripherals, Peripherals};
use hal::prelude::*;
use hal::time::KiloHertz;
use hal::timer::SpinTimer;
use smart_leds::hsv::{hsv2rgb, Hsv, RGB8};
use smart_leds::SmartLedsWrite;

use lis3dh::{accelerometer::Accelerometer, Lis3dh, SlaveAddr};

#[hal::entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let _core_peripherals = CorePeripherals::take().unwrap();

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

    let mut delay = Delay::new(_core_peripherals.SYST, &mut clocks);
    delay.delay_ms(255u8);

    // i2c
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

    let mut state = TiltState::new(2.0, 2);

    loop {
        while !lis3dh.is_data_ready().unwrap() {}
        let dat = lis3dh.accel_norm().unwrap();

        let (pos, j) = state.update(dat.x);

        // iterate through neopixels and paint the one led
        let _ = neopixel.write((0..5).map(|i| {
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
