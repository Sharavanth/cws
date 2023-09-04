pub mod types;

use self::types::{Lcd, MaxTempReader, Mux};
use core::{
    convert::Infallible,
    ops::{Add, Sub},
};
use defmt::Format;
use embedded_hal::{blocking::delay::DelayMs, digital::v2::OutputPin};
use lcd1602::DisplayError;
use muxhc4067::Mux74hc4067Error;
#[cfg(feature = "agas")]
use pid::Pid;

// Output channels
/*
OUT1 --> CH0
OUT2 --> CH8
OUT3 --> CH1
OUT4 --> CH9
OUT5 --> CH2
OUT6 --> CH10
OUT7 --> CH3
OUT8 --> CH11
*/

#[repr(u8)]
#[derive(PartialEq, Format)]
enum Channels {
    LadlePlus = 5,
    LadleMinus = 14,
    Ignition = 15,
    #[cfg(feature = "mtilt")]
    TiltPlus = 13,
    #[cfg(feature = "mtilt")]
    TiltMinus = 7,
    #[cfg(feature = "agas")]
    AutoGas = 6,
    #[cfg(feature = "agas")]
    GasPlus = 4,
    #[cfg(feature = "agas")]
    GasMinus = 12,

    // SENSOR AND ALARMS
    InHomePos = 0,
    InLadleStopPos = 8,
    InTiltDownLimit = 1,
    InTiltUpLimit = 9,
    EmergencyAlarm = 2,
    FlameoutAlarm = 10,
    VFDMtrAlarm = 3,
    TiltMtrAlarm = 11,
}

#[derive(PartialEq, Format, Clone, Copy)]
enum LedPin {
    Red,
    Green,
    Blue,
}

#[derive(PartialEq, Format, Clone, Copy)]
enum Alarms {
    None,
    FlameOut,
    Emergency,
    VFDTrip,
    TiltMotorTrip,
}

#[derive(Format, Copy, Clone)]
pub struct SensorValues {
    alarm: Alarms,
    temperature: f32,
    in_home_position: bool,
    in_ladle_stop_position: bool,
    pub in_tilt_down_limit_position: bool,
    pub in_tilt_up_limit_position: bool,
}

#[derive(Format, Copy, Clone)]
pub struct KeypadValues {
    temp_set_point: f32,
    pub gas_flow: u8,
    pub tilt_pos: u8,
    in_auto_mode: bool,
    pub ladle_speed: i8,
    pub ignition_state: bool,
    pub prev_ignition_state: bool,
}

pub struct Inputs<L1, L2, L3> {
    mux: Mux,
    temp_reader: MaxTempReader,
    sensor_values: SensorValues,
    keypad_values: KeypadValues,
    led_pins: (L1, L2, L3),
    #[cfg(feature = "agas")]
    pid: Option<Pid<f32>>,
}

impl<L1, L2, L3> Inputs<L1, L2, L3>
where
    L1: OutputPin<Error = Infallible>,
    L2: OutputPin<Error = Infallible>,
    L3: OutputPin<Error = Infallible>,
{
    pub fn new(mux: Mux, temp_reader: MaxTempReader, led_pins: (L1, L2, L3)) -> Self {
        let sensor_values = SensorValues {
            temperature: 0.0,
            alarm: Alarms::None,
            in_home_position: false,
            in_ladle_stop_position: false,
            in_tilt_down_limit_position: false,
            in_tilt_up_limit_position: false,
        };
        let keypad_values = KeypadValues {
            temp_set_point: 0.0,
            in_auto_mode: false,
            ladle_speed: 0,
            gas_flow: 0,
            tilt_pos: 0,
            ignition_state: false,
            prev_ignition_state: false,
        };
        Inputs {
            mux,
            led_pins,
            temp_reader,
            sensor_values,
            keypad_values,
            #[cfg(feature = "agas")]
            pid: Pid::new(0.2, 0.01, 1.0, 100.0, 100.0, 100.0, 100.0, 30.0),
        }
    }

    pub fn read_sensor_values(
        &mut self,
        delay: &mut impl DelayMs<u32>,
    ) -> Result<SensorValues, Mux74hc4067Error<Infallible>> {
        self.sensor_values.temperature = self.temp_reader.read().unwrap_or(0.0_f32);
        self.sensor_values.in_home_position = self.read_mux_channel(Channels::InHomePos, delay)?;

        self.sensor_values.in_ladle_stop_position =
            self.read_mux_channel(Channels::InLadleStopPos, delay)?;

        if cfg!(feature = "mtilt") {
            self.sensor_values.in_tilt_down_limit_position =
                self.read_mux_channel(Channels::InTiltDownLimit, delay)?;
            self.sensor_values.in_tilt_up_limit_position =
                self.read_mux_channel(Channels::InTiltUpLimit, delay)?;
        }

        if self.read_mux_channel(Channels::FlameoutAlarm, delay)? {
            // set ignition state to off
            self.keypad_values.ignition_state = false;
            self.sensor_values.alarm = Alarms::FlameOut
        } else if self.read_mux_channel(Channels::EmergencyAlarm, delay)? {
            // set ignition state to off
            self.keypad_values.ignition_state = false;
            self.sensor_values.alarm = Alarms::Emergency
        } else if self.read_mux_channel(Channels::VFDMtrAlarm, delay)? {
            self.keypad_values.ignition_state = false;
            self.sensor_values.alarm = Alarms::VFDTrip
        } else if self.read_mux_channel(Channels::TiltMtrAlarm, delay)? {
            self.keypad_values.ignition_state = false;
            self.sensor_values.alarm = Alarms::TiltMotorTrip
        } else {
            self.sensor_values.alarm = Alarms::None
        }
        Ok(self.sensor_values)
    }

    pub fn read_keypad_values(
        &mut self,
        delay: &mut impl DelayMs<u32>,
    ) -> Result<KeypadValues, Mux74hc4067Error<Infallible>> {
        #[cfg(feature = "agas")]
        {
            let auto_gas_pin = self.read_mux_channel(Channels::AutoGas, delay)?;
            // read Gas Auto mode
            if self.keypad_values.ignition_state && auto_gas_pin {
                delay.delay_ms(150u32);
                self.keypad_values.in_auto_mode ^= auto_gas_pin;
            }
        }

        // read ignition pin
        let ign_pin = self.read_mux_channel(Channels::Ignition, delay)?;
        self.keypad_values.prev_ignition_state = self.keypad_values.ignition_state;
        if ign_pin {
            delay.delay_ms(150u32);
            self.keypad_values.ignition_state = match (self.keypad_values.ignition_state, ign_pin) {
                (false, false) => false,
                (false, true) => true,
                (true, false) => true,
                (true, true) => true,
            };
        }

        //DEBUG: Not condition to be removed here
        if !self.sensor_values.in_home_position {
            let mut ladle_plus: i8 = 0;
            let mut ladle_minus: i8 = 0;

            let ladle_plus_pin = self.read_mux_channel(Channels::LadlePlus, delay)?;
            let ladle_minus_pin = self.read_mux_channel(Channels::LadleMinus, delay)?;
            // read Ladle Speed changes
            if ladle_plus_pin {
                delay.delay_ms(120_u32);
                ladle_plus = if ladle_plus_pin { 1_i8 } else { 0_i8 };
            }
            if ladle_minus_pin {
                delay.delay_ms(120_u32);
                ladle_minus = if ladle_minus_pin { 1_i8 } else { 0_i8 };
            }

            self.keypad_values.ladle_speed = self
                .keypad_values
                .ladle_speed
                .add(ladle_plus)
                .sub(ladle_minus)
                .min(30)
                .max(-30);

            #[cfg(feature = "agas")]
            {
                let gas_or_temp_plus_pin = self.read_mux_channel(Channels::GasPlus, delay)?;
                let gas_or_temp_minus_pin = self.read_mux_channel(Channels::GasMinus, delay)?;
                if !self.keypad_values.ignition_state {
                    // Mc is not in ignited, so close gas valve
                    self.keypad_values.gas_flow = 0;
                } else if self.keypad_values.in_auto_mode {
                    // AUTO MODE
                    let mut temp_plus: f32 = 0.0;
                    let mut temp_minus = 0.0;

                    if gas_or_temp_plus_pin {
                        delay.delay_ms(120_u32);
                        temp_plus = if gas_or_temp_plus_pin {
                            1.0_f32
                        } else {
                            0.0_f32
                        };
                    }
                    if gas_or_temp_minus_pin {
                        delay.delay_ms(120_u32);
                        temp_minus = if gas_or_temp_minus_pin {
                            1.0_f32
                        } else {
                            0.0_f32
                        };
                    }

                    self.keypad_values.temp_set_point = self
                        .keypad_values
                        .temp_set_point
                        .add(temp_plus)
                        .sub(temp_minus)
                        .mul(10.0)
                        .min(250.0)
                        .max(20.0);

                    self.update_setpoint(self.keypad_values.temp_set_point);
                    let output = self.pid.next_control_output(self.sensor_values.temperature);

                    self.keypad_values.gas_flow = self
                        .keypad_values
                        .gas_flow
                        .add(output.output as u8)
                        .min(100)
                        .max(10);

                    defmt::info!("{}, {}, {}", output.p, output.i, output.d);
                } else {
                    // MANUAL MODE
                    let mut gas_plus: u8 = 0;
                    let mut gas_minus: u8 = 0;

                    if gas_or_temp_plus_pin {
                        delay.delay_ms(120_u32);
                        gas_plus = if gas_or_temp_plus_pin { 1_u8 } else { 0_u8 };
                    }
                    if gas_or_temp_minus_pin {
                        delay.delay_ms(120_u32);
                        gas_minus = if gas_or_temp_minus_pin { 1_u8 } else { 0_u8 };
                    }

                    self.keypad_values.gas_flow = self
                        .keypad_values
                        .gas_flow
                        .add(gas_plus)
                        .sub(gas_minus)
                        .mul(10)
                        .min(100)
                        .max(10);
                }
            }
        }

        #[cfg(feature = "mtilt")]
        {
            let tilt_plus_pin = self.read_mux_channel(Channels::TiltPlus, delay)?;
            let tilt_minus_pin = self.read_mux_channel(Channels::TiltMinus, delay)?;

            if !self.sensor_values.in_home_position {
                // read Tilt position
                let mut tilt_plus: u8 = 0;
                let mut tilt_minus: u8 = 0;

                if tilt_plus_pin {
                    delay.delay_ms(120_u32);
                    tilt_plus = if tilt_plus_pin { 1_u8 } else { 0_u8 };
                }
                if tilt_minus_pin {
                    delay.delay_ms(120_u32);
                    tilt_minus = if tilt_minus_pin { 1_u8 } else { 0_u8 };
                }

                self.keypad_values.tilt_pos = self
                    .keypad_values
                    .tilt_pos
                    .saturating_add(tilt_plus)
                    .saturating_sub(tilt_minus)
                    .min(10)
                    .max(1);
            }
        }

        Ok(self.keypad_values)
    }

    pub fn update_display(
        &mut self,
        lcd: &mut Lcd,
        delay: &mut impl DelayMs<u32>,
    ) -> Result<(), DisplayError> {
        let alarm = self.sensor_values.alarm;
        let temperature = self.sensor_values.temperature;
        let ladle_speed = self.keypad_values.ladle_speed;
        let tilt_pos = self.keypad_values.tilt_pos;
        let gas_flow = self.keypad_values.gas_flow;
        let temp_set_point = self.keypad_values.temp_set_point;
        let ignition_state = self.keypad_values.ignition_state;
        let in_auto_mode = self.keypad_values.in_auto_mode;

        lcd.clear_buffer();

        lcd.print_str("LADLE", 0, 10, 6);
        lcd.print_i8(ladle_speed, 1, 10, 3);
        lcd.print_str("rpm", 1, 13, 3);
        self.set_led(LedPin::Blue);

        match alarm {
            Alarms::FlameOut => {
                lcd.print_str("BRNR RDY", 0, 0, 10);
                lcd.print_str("!FLAMEOUT", 1, 0, 10);
                self.set_led(LedPin::Red);
                defmt::warn!("Flame Out");
            }
            Alarms::Emergency => {
                lcd.print_str("  !! EMERGENCY", 0, 0, 16);
                lcd.print_str("  STOP PRESSED", 0, 1, 16);
                self.set_led(LedPin::Red);
                defmt::warn!("Emergency pressed");
            }
            Alarms::VFDTrip => {
                lcd.print_str("     !! VFD", 0, 0, 16);
                lcd.print_str("  MOTOR TRIP", 0, 1, 16);
                self.set_led(LedPin::Red);
            }
            Alarms::TiltMotorTrip => {
                lcd.print_str("    !! TILT", 0, 0, 16);
                lcd.print_str("  MOTOR TRIP", 0, 1, 16);
                self.set_led(LedPin::Red);
            }
            Alarms::None => {
                //DEBUG: Not condition to be added here
                if self.sensor_values.in_home_position {
                    lcd.print_str("MC NOT IN HOME", 0, 0, 16);
                    lcd.print_str("TILT POS:", 1, 0, 10);
                    lcd.print_u8(tilt_pos, 1, 11, 6);
                    self.set_led(LedPin::Red);
                } else if !ignition_state {
                    // defmt::info!("!IGN");
                    lcd.print_str("BRNR RDY", 0, 0, 10);
                    lcd.print_str("PRESS IGN", 1, 0, 10);
                    self.set_led(LedPin::Blue);
                // } else if ignition_state == IgnitionState::Igniting {
                //     lcd.print_str("IGN...", 0, 0);
                //     lcd.print_u8(gas_flow, 1, 0);
                //     lcd.print_str("%", 1, 5);
                //     self.set_led(LedPin::Green);
                } else if in_auto_mode {
                    lcd.print_str("AUTO", 0, 0, 5);
                    lcd.print_ufloat(temp_set_point, 0, 5, 3);
                    lcd.print_str("C", 0, 8, 2);

                    lcd.print_u8(gas_flow, 1, 0, 3);
                    lcd.print_str("%", 1, 3, 2);
                    lcd.print_ufloat(temperature, 1, 5, 3);
                    lcd.print_str("C", 1, 8, 2);

                    self.set_led(LedPin::Green);
                } else {
                    lcd.print_str("MANUAL", 0, 0, 10);

                    #[cfg(feature = "agas")]
                    {
                        lcd.print_u8(gas_flow, 1, 0, 4);
                        lcd.print_str("%", 1, 4, 6);
                    }
                    #[cfg(not(feature = "agas"))]
                    {
                        lcd.print_str("BURNING", 1, 0, 10);
                    }
                    self.set_led(LedPin::Green);
                }
            }
        }

        lcd.write_to_display(delay);
        Ok(())
    }

    pub fn _print_mux_channels(&mut self) -> Result<(), Mux74hc4067Error<Infallible>> {
        let mut result = [false; 16];

        self.mux.print_all_channels(&mut result)?;
        defmt::info!("{:?}", result);
        Ok(())
    }
    // ###########################################################################################
    fn read_mux_channel(
        &mut self,
        channel: Channels,
        delay: &mut impl DelayMs<u32>,
    ) -> Result<bool, Mux74hc4067Error<Infallible>> {
        self.mux.read_channel(channel as u8, delay)
    }
    fn set_led(&mut self, pin: LedPin) -> Result<(), Infallible> {
        let (r, g, b) = &mut self.led_pins;

        r.set_low()?;
        g.set_low()?;
        b.set_low()?;

        match pin {
            LedPin::Red => r.set_high()?,
            LedPin::Green => g.set_high()?,
            LedPin::Blue => b.set_high()?,
        }
        Ok(())
    }
    #[cfg(feature = "agas")]
    fn update_setpoint(&mut self, setpoint: f32) {
        if let Some(mut pid) = self.pid {
            pid.setpoint = setpoint;
            self.pid = Some(pid);
        }
    }
}
