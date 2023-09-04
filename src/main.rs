#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![allow(unused_must_use)]
#![allow(clippy::empty_loop)]
#![feature(int_roundings)]
#![allow(unsafe_code)]

mod io;
mod vfd;

use crate::io::Inputs;
use crate::vfd::{VfdModbus, RS485};
#[cfg(feature = "agas")]
use a4988::A4988;
use core::cell::RefCell;
#[cfg(feature = "agas, mtilt")]
use core::cmp::Ordering;
use cws_firmware as _;
use embedded_hal::spi::{Mode, Phase, Polarity};
use lcd1602::LcdI2c;
use max6675::Max6675;
use muxhc4067::Mux74hc4067;
use stm32f1xx_hal::gpio::PinState;
#[cfg(feature = "mtilt")]
use stm32f1xx_hal::gpio::{Output, PA8, PA9};
use stm32f1xx_hal::serial::StopBits;
use stm32f1xx_hal::{
    device::{Interrupt, Peripherals},
    i2c::{BlockingI2c, DutyCycle, Mode as I2cMode},
    pac::interrupt,
    prelude::*,
    serial::{Config, Serial},
    spi::Spi,
};

pub const MODE: Mode = Mode {
    polarity: Polarity::IdleHigh,
    phase: Phase::CaptureOnSecondTransition,
};

static RS485: cortex_m::interrupt::Mutex<RefCell<Option<RS485>>> =
    cortex_m::interrupt::Mutex::new(RefCell::new(None));

#[interrupt]
unsafe fn USART2() {
    cortex_m::interrupt::free(|cs| {
        if let Some(rs485) = RS485.borrow(cs).borrow_mut().as_mut() {
            rs485.rs485_listener(cs);
        }
    })
}

#[cortex_m_rt::entry]
fn main() -> ! {
    if let (Some(dp), Some(cp)) = (
        Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {
        // local re-writable variables

        let mut curr_ladle_speed: i8 = 0;
        #[cfg(feature = "agas")]
        let mut curr_gas_flow: Option<u8> = Some(0);
        #[cfg(feature = "mtilt")]
        let mut curr_tilt_pos: Option<u8> = Some(0);

        // Take ownership over the raw flash and rcc devices and convert them into the corresponding
        // HAL structs
        let mut flash = dp.FLASH.constrain();
        let rcc = dp.RCC.constrain();
        let mut afio = dp.AFIO.constrain();

        // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
        let clocks = rcc
            .cfgr
            // .use_hse(16.MHz())
            .freeze(&mut flash.acr);

        // Create a delay abstraction based on SysTick
        let mut delay = cp.SYST.delay(&clocks);

        // Prepare the GPIO peripheral
        let mut gpioa = dp.GPIOA.split();
        let mut gpiob = dp.GPIOB.split();

        let (_, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

        // -----------------------------------------------------------------------------------------
        // LCD Module
        // -----------------------------------------------------------------------------------------
        defmt::info!("Initializing I2C comm for LCD module");
        let (scl, sda) = (
            gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl),
            gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl),
        );
        let i2c = BlockingI2c::i2c1(
            dp.I2C1,
            (scl, sda),
            &mut afio.mapr,
            I2cMode::Fast {
                frequency: 400.kHz(),
                duty_cycle: DutyCycle::Ratio16to9,
            },
            clocks,
            1000,
            10,
            1000,
            1000,
        );

        let mut lcd1602 = match LcdI2c::new(0x27, i2c, 16, 2, &mut delay) {
            Ok(lcd) => lcd,
            Err(e) => {
                panic!("Error initializing LCD {:?}", e)
            }
        };
        lcd1602.clear_buffer();
        lcd1602.print_str("COOKWOK SUPREME", 0, 0, 16);
        lcd1602.print_str("INITIALIZING", 1, 2, 14);
        lcd1602.write_to_display(&mut delay);

        // -----------------------------------------------------------------------------------------
        // GAS Stepper Driver Module
        // -----------------------------------------------------------------------------------------
        // Stepper A4988 pins
        #[cfg(feature = "agas")]
        defmt::info!("Initializing A4988 drivers for stepper valve");
        #[cfg(feature = "agas")]
        let (enable, reset, step, direction, sleep) = (
            gpiob.pb15.into_push_pull_output(&mut gpiob.crh),
            gpiob.pb14.into_push_pull_output(&mut gpiob.crh),
            gpiob.pb13.into_push_pull_output(&mut gpiob.crh),
            gpiob.pb12.into_push_pull_output(&mut gpiob.crh),
            gpiob.pb9.into_push_pull_output(&mut gpiob.crh),
        );
        // Configure Stepper motor driver
        #[cfg(feature = "agas")]
        let mut a4988 = A4988::new(enable, reset, step, direction, sleep);

        // -----------------------------------------------------------------------------------------
        // Temp sensor SPI init
        // -----------------------------------------------------------------------------------------
        defmt::info!("Initializing SPI for MAX6675 k-type thermocouple");
        // Configure pins for SPI
        let (sck, miso, mosi) = (
            gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl),
            gpioa.pa6,
            gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl),
        );
        // Configure SPI with 100kHz rate
        let spi = Spi::spi1(
            dp.SPI1,
            (sck, miso, mosi),
            &mut afio.mapr,
            MODE,
            1.MHz(),
            clocks,
        );
        let cs = gpiob.pb8.into_push_pull_output(&mut gpiob.crh);
        let max6675 = Max6675::new(spi, cs);

        // -----------------------------------------------------------------------------------------
        // MUX init
        // -----------------------------------------------------------------------------------------
        defmt::info!("Initializing MUX extender");
        let (a1, a2, a3, a4) = (
            gpiob.pb10.into_push_pull_output(&mut gpiob.crh),
            gpiob.pb1.into_push_pull_output(&mut gpiob.crl),
            gpiob.pb0.into_push_pull_output(&mut gpiob.crl),
            gpiob.pb2.into_push_pull_output(&mut gpiob.crl),
        );
        let mux_signal = gpiob.pb11.into_pull_down_input(&mut gpiob.crh);
        // Configure MUX extender
        let mux74hc4067 = Mux74hc4067::new((a1, a2, a3, a4), mux_signal);

        // -----------------------------------------------------------------------------------------
        // Output pins
        // -----------------------------------------------------------------------------------------
        // Tilt output pins
        #[cfg(feature = "mtilt")]
        defmt::info!("Configuring Output pins");
        #[cfg(feature = "mtilt")]
        let mut tilt_pins: Option<(PA8<Output>, PA9<Output>)> = if cfg!(feature = "mtilt") {
            Some((
                gpioa.pa8.into_push_pull_output(&mut gpioa.crh), //D04
                gpioa.pa9.into_push_pull_output(&mut gpioa.crh), //D05
            ))
        };
        // Keypad LED PINS
        defmt::info!("Configuring LED indicator pins");
        let led_pins = (
            pb3.into_push_pull_output(&mut gpiob.crl),
            pb4.into_push_pull_output(&mut gpiob.crl),
            gpiob.pb5.into_push_pull_output(&mut gpiob.crl),
        );

        defmt::info!("Configuring IGN trigger pin");
        // Ignition trigger output pin
        let mut ignition_trigger_pin = gpioa.pa11.into_push_pull_output(&mut gpioa.crh); //D07

        // -----------------------------------------------------------------------------------------
        // VFD RS485 init
        // -----------------------------------------------------------------------------------------
        defmt::info!("Initializing MODBUS RS485");
        // USART interface for VFD RS485 communication
        let serial = (
            gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl),
            gpioa.pa3,
        );
        let uart = Serial::new(
            dp.USART2,
            serial,
            &mut afio.mapr,
            Config::default()
                .parity_none()
                .baudrate(9600.bps())
                .stopbits(StopBits::STOP2),
            &clocks,
        );

        let rts = gpioa
            .pa4
            .into_push_pull_output_with_state(&mut gpioa.crl, PinState::Low);
        let (tx, rx) = uart.split();

        let tim3_counter = dp.TIM3.counter_ms(&clocks);
        // store globals
        cortex_m::interrupt::free(|cs| {
            let rs485 = RS485::new(tx, rx, rts, tim3_counter);
            RS485.borrow(cs).replace(Some(rs485));
        });
        // unmask the USART2 interrupt
        unsafe {
            cortex_m::peripheral::NVIC::unmask(Interrupt::USART2);
        }

        defmt::info!("Initializing VFD Drive");
        // init vfd drive
        let mut vfd: VfdModbus = VfdModbus::new();
        cortex_m::interrupt::free(|cs| {
            let mut rs485_ref = RS485.borrow(cs).borrow_mut();
            let rs485 = rs485_ref.as_mut().unwrap();

            if let Err(e) = vfd.update_drive_state(cs, rs485, &mut delay) {
                defmt::error!("Error init VFD: {:?}", e);
            };
        });

        // -----------------------------------------------------------------------------------------
        // IO module init
        // -----------------------------------------------------------------------------------------
        defmt::info!("Initializing IO module");
        let mut io = Inputs::new(mux74hc4067, max6675, led_pins);

        // -----------------------------------------------------------------------------------------
        // Main program loop
        // -----------------------------------------------------------------------------------------
        defmt::info!("Running main loop now");
        loop {
            // Reading sensor values
            io.read_sensor_values(&mut delay).unwrap();

            // Reading user input
            let keypad_values = io.read_keypad_values(&mut delay).unwrap();

            cortex_m::interrupt::free(|cs| {
                let mut rs485_ref = RS485.borrow(cs).borrow_mut();
                let rs485 = rs485_ref.as_mut().unwrap();

                if let Err(e) = vfd.set_ladle_speed(
                    cs,
                    rs485,
                    &mut delay,
                    keypad_values.ladle_speed,
                    curr_ladle_speed,
                ) {
                    defmt::error!("Error setting ladle speed: {:?}", e);
                };

                curr_ladle_speed = keypad_values.ladle_speed;
            });

            // Maybe ignite
            {
                // Pull ignition pin low on every cycle
                ignition_trigger_pin.set_low();
                delay.delay_ms(5_u8);

                if keypad_values.ignition_state ^ keypad_values.prev_ignition_state {
                    lcd1602.print_str("IGNITING", 1, 0, 10);

                    ignition_trigger_pin.set_high();
                    #[cfg(feature = "agas")]
                    {
                        stepper.forward()?;
                        stepper.step(INIT_GAS_FLOW_FOR_IGNITION as u16, delay)?;
                    }
                    delay.delay_ms(2000_u16);
                }
            }

            #[cfg(feature = "mtilt")]
            // Set tilt position
            {
                let curr_pos = curr_tilt_pos.unwrap();
                if let Some((tc_down_pin, tc_up_pin)) = &mut tilt_pins {
                    tc_down_pin.set_low();
                    tc_up_pin.set_low();

                    match keypad_values.tilt_pos.cmp(&curr_pos) {
                        Ordering::Greater => {
                            // defmt::info!(
                            //     "Tilt: {}, {}",
                            //     keypad_values.tilt_pos,
                            //     curr_tilt_pos
                            // );
                            if !sensor_readings.in_tilt_down_limit_position {
                                tc_up_pin.set_high();
                            } else {
                                tc_up_pin.set_low();
                            }
                        }
                        Ordering::Less => {
                            if !sensor_readings.in_tilt_up_limit_position {
                                tc_down_pin.set_high();
                            } else {
                                tc_down_pin.set_low();
                            }
                        }
                        Ordering::Equal => {}
                    }

                    curr_tilt_pos = Some(keypad_values.tilt_pos);
                    delay.delay_ms(100_u16);
                }
            }

            #[cfg(feature = "agas")]
            // Set gas flow
            {
                let gas_flow = curr_gas_flow.unwrap();
                let required_flow_pct = keypad_values.gas_flow.abs_diff(gas_flow);
                defmt::info!("Gas: {}, {}", required_flow_pct, gas_flow);

                if keypad_values.ignition_state {
                    match required_flow_pct.cmp(&gas_flow) {
                        Ordering::Greater => {
                            a4988.forward();
                            a4988.step(required_flow_pct as u16, delay);
                        }
                        Ordering::Less => {
                            a4988.reverse();
                            a4988.step(required_flow_pct as u16, delay);
                        }
                        _ => {
                            a4988.sleep();
                        }
                    }

                    delay.delay_ms(100_u16);
                }
                curr_gas_flow = Some(required_flow_pct);
            }

            // if let Err(e) = io._print_mux_channels() {
            //     defmt::error!("MUX channels: {:?}", e)
            // };

            // Updating display
            if let Err(e) = io.update_display(&mut lcd1602, &mut delay) {
                defmt::error!("Error updating display: {:?}", e);
            };
        }
    }

    // -----------------------------------------------------------------------------------------
    // Sleep loop
    // -----------------------------------------------------------------------------------------
    loop {
        defmt::info!("{}", "Going to sleep");
        // Go to sleep
        cortex_m::asm::wfi();
    }
}
