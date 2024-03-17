#![no_std]
#![no_main]
#![allow(async_fn_in_trait)]

//! This example uses the RP Pico W board Wifi chip (cyw43).
//! Scans Wifi for ssid names.
use core::str;

use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_net::Stack;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[embassy_executor::task]
async fn wifi_task(runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<cyw43::NetDriver<'static>>) -> ! {
    stack.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    defmt::info!("Initializing...");

    let peripherals = embassy_rp::init(Default::default());

    // For Wi-Fi according to example here: https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/wifi_scan.rs
    let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");

    let pwr = Output::new(peripherals.PIN_23, Level::Low);
    let cs = Output::new(peripherals.PIN_25, Level::High);
    let mut pio = Pio::new(peripherals.PIO0, Irqs);
    let spi = PioSpi::new(&mut pio.common, pio.sm0, pio.irq0, cs, peripherals.PIN_24, peripherals.PIN_29, peripherals.DMA_CH0);

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (_net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    unwrap!(spawner.spawn(wifi_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    // let mut scanner = control.scan(Default::default()).await;
    // while let Some(bss) = scanner.next().await {
    //     if let Ok(ssid_str) = str::from_utf8(&bss.ssid) {
    //         info!("scanned {} == {:x}", ssid_str, bss.bssid);
    //     }
    // }

    let result = control.join_wpa2(wifi_name, wifi_password).await;



    let mut led_pin = Output::new(peripherals.PIN_16, Level::Low);
    let vib_pin = Input::new(peripherals.PIN_17, Pull::Down);

    defmt::info!("Initialized.");

    let cycles_per_blocks = 200;
    let mut block = 1;
    let mut cycles = 0;
    let mut vibration_detected_in_last_block = false;

    loop {
        if vib_pin.is_high() {
            vibration_detected_in_last_block = true;
        }

        if cycles % cycles_per_blocks == 0 {
            if vibration_detected_in_last_block {
                led_pin.set_high();
                info!("{}. On!", block);
            } else {
                led_pin.set_low();
                info!("{}. Off!", block);
            }

            vibration_detected_in_last_block = false;
            cycles = 0;
            block += 1;
        }

        Timer::after(Duration::from_millis(1)).await;

        cycles += 1;
    }
}