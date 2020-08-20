# Adafruit PyGamer Advanced Examples

These are examples that didnt fit into the [other examples](https://crates.io/crates/pygamer) for the [Adafruit PyGamer board](https://www.adafruit.com/product/4242).

## Prerequisites

* Add the cross compile toolchain `rustup target add thumbv7em-none-eabihf`
* Install [hf2-cli](https://crates.io/crates/hf2-cli) the hf2 bootloader flasher tool however your platform requires.

## Uploading an example

* cd into an example directory
* Put your device in bootloader mode usually by tapping the reset button twice.
* Build and upload in one step

```bash
$ cargo run --example neopixel_tilt
    Finished dev [optimized + debuginfo] target(s) in 0.19s
    Searching for a connected device with known vid/pid pair.
    Trying  Ok(Some("Adafruit Industries")) Ok(Some("PyGamer"))
    Flashing "/home/jacob/Downloads/pygamer-quickstart/target/thumbv7em-none-eabihf/debug/neopixel_tilt"
    Finished in 0.079s
$
```

Or often you can just click run in your ide

Complex examples might requires features
`cargo +nightly run --example magic_wand --features=tensorflow`

## What happend

In your ./cargo/config hf2 is set as your 'runner' so when the build is done, it calls hf2 with your elf file. If hf2 finds a usb device connected and in bootloader mode, it uploads to it. You could do this manually by calling something like `cargo build --example neopixel_tilt` and `hf2 elf target/thumbv7em-none-eabihf/debug/neopixel_tilt`

## Note

The dev profile is set to use release optimizations since we have no way to debug by default, so we leave off using `--release` you might see elsewhere.
