pub mod rs485;

use bit_field::BitField;
use core::ops::{Add, Div, Mul, Sub};
use defmt::Format;
use embedded_hal::blocking::delay::DelayMs;
use modbus::{ModbusRTU, ModbusRTUError};
pub use rs485::RS485;
use rs485::{RS485Error, RS485State};

#[derive(Clone, Format)]
pub enum VfdError {
    DriveNotReady,
    DriveAlarm,
    Rtu(ModbusRTUError),
    RS485(RS485Error),
}

impl From<ModbusRTUError> for VfdError {
    fn from(e: ModbusRTUError) -> Self {
        VfdError::Rtu(e)
    }
}
impl From<RS485Error> for VfdError {
    fn from(e: RS485Error) -> Self {
        VfdError::RS485(e)
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum DriveState {
    Ready = 1,
    Error = 2,
    NotRead = 0,
}
#[derive(Clone, Copy)]
pub enum Direction {
    Forward,
    Reverse,
    Unset,
    NotRead,
}
#[derive(Clone, Copy)]
pub enum RunState {
    Run,
    Stop,
    JogRun,
}

const RESPONSE_BUF_SIZE: usize = 8 * 4;
pub struct VfdModbus {
    alarm: bool,
    output: bool,
    modbus: ModbusRTU,
    direction: Direction,
    drive_state: DriveState,
}

impl VfdModbus {
    pub fn new() -> Self {
        Self {
            alarm: false,
            output: false,
            direction: Direction::Unset,
            drive_state: DriveState::NotRead,
            modbus: ModbusRTU::new(0x01),
        }
    }

    pub fn set_ladle_speed<D>(
        &mut self,
        _cs: &cortex_m::interrupt::CriticalSection,
        rs485: &mut RS485,
        delay: &mut D,
        set_speed: i8,
        curr_speed: i8,
    ) -> Result<(), VfdError>
    where
        D: DelayMs<u16>,
    {
        if self.drive_state == DriveState::Error {
            return Err(VfdError::DriveNotReady);
        }
        if self.alarm {
            return Err(VfdError::DriveAlarm);
        }

        match set_speed.cmp(&curr_speed) {
            core::cmp::Ordering::Greater => {
                self.set_run_state(_cs, rs485, delay, RunState::Run, Direction::Forward)?;
                self.set_freq(_cs, rs485, delay, set_speed.unsigned_abs() as u16)?;
                Ok(())
            }
            core::cmp::Ordering::Less => {
                self.set_run_state(_cs, rs485, delay, RunState::Run, Direction::Reverse)?;
                self.set_freq(_cs, rs485, delay, set_speed.unsigned_abs() as u16)?;
                Ok(())
            }
            core::cmp::Ordering::Equal => {
                self.set_run_state(_cs, rs485, delay, RunState::Stop, Direction::Unset)?;
                self.set_freq(_cs, rs485, delay, 0)?;
                Ok(())
            }
        }
    }

    pub fn update_drive_state(
        &mut self,
        _cs: &cortex_m::interrupt::CriticalSection,
        rs485: &mut RS485,
        delay: &mut impl DelayMs<u16>,
    ) -> Result<(), VfdError> {
        let mut rsp = [0; RESPONSE_BUF_SIZE];
        const DATA_LEN: usize = 1;
        let (req, computed_rsp_len) = self.modbus.build_request(0x2226, 0x03, DATA_LEN as u16)?;

        rs485.poll(&req, &mut rsp, computed_rsp_len, delay, _cs)?;

        let mut result = [0_u16; DATA_LEN];
        self.modbus.check_confirmation(&req, &rsp)?;
        self.modbus.check_integrity(&rsp)?;
        self.modbus.decode(&rsp, &mut result);

        let drive_status_frame = result[0];
        self.direction = match drive_status_frame.get_bits(0..=1) {
            0b01 => Direction::Forward,
            0b10 => Direction::Reverse,
            0b00 => Direction::Unset,
            _ => Direction::NotRead,
        };
        self.drive_state = match drive_status_frame.get_bits(2..=3) {
            0b01 => DriveState::Ready,
            0b10 => DriveState::Error,
            _ => DriveState::NotRead,
        };
        self.output = drive_status_frame.get_bit(4);
        self.alarm = drive_status_frame.get_bit(5);
        Ok(())
    }

    fn set_freq(
        &mut self,
        _cs: &cortex_m::interrupt::CriticalSection,
        rs485: &mut RS485,
        delay: &mut impl DelayMs<u16>,
        speed: u16,
    ) -> Result<Option<()>, VfdError> {
        let mut rsp = [0; RESPONSE_BUF_SIZE];
        let (req, computed_rsp_len) = self.modbus.build_request(0x2001, 0x06, speed)?;

        rs485.poll(&req, &mut rsp, computed_rsp_len, delay, _cs)?;
        self.modbus.check_confirmation(&req, &rsp)?;
        self.modbus.check_integrity(&rsp)?;
        Ok(Some(()))
    }
    fn set_run_state(
        &mut self,
        _cs: &cortex_m::interrupt::CriticalSection,
        rs485: &mut RS485,
        delay: &mut impl DelayMs<u16>,
        state: RunState,
        direction: Direction,
    ) -> Result<Option<()>, VfdError> {
        let mut content = 0_u16;
        content.set_bits(
            0..=1,
            match state {
                RunState::Run => 0b10,
                RunState::Stop => 0b01,
                RunState::JogRun => 0b11,
            },
        );
        content.set_bits(
            4..=5,
            match direction {
                Direction::Forward => 0b01,
                Direction::Reverse => 0b10,
                _ => 0b00,
            },
        );

        let mut rsp = [0; RESPONSE_BUF_SIZE];
        let (req, computed_rsp_len) = self.modbus.build_request(0x2000, 0x06, content)?;

        rs485.poll(&req, &mut rsp, computed_rsp_len, delay, _cs)?;
        self.modbus.check_confirmation(&req, &rsp)?;
        self.modbus.check_integrity(&rsp)?;
        Ok(Some(()))
    }
    fn get_dc_bus_voltage(
        &mut self,
        _cs: &cortex_m::interrupt::CriticalSection,
        rs485: &mut RS485,
        delay: &mut impl DelayMs<u16>,
    ) -> Result<Option<u8>, VfdError> {
        let mut rsp = [0; RESPONSE_BUF_SIZE];
        let (req, computed_rsp_len) = self.modbus.build_request(0x2203, 0x03, 2)?;
        rs485.poll(&req, &mut rsp, computed_rsp_len, delay, _cs)?;
        defmt::info!("POLL_DC_BUS_VOLTAGE {:#02x}, {:#02x}", req, rsp);

        let mut result = [0_u16; 2];
        self.modbus.check_confirmation(&req, &rsp)?;
        self.modbus.check_integrity(&rsp)?;
        self.modbus
            .decode(&rsp[0..(computed_rsp_len as usize)], &mut result);
        let bus_voltage = (result[0] as f32)
            .mul(100_f32)
            .add(result[1] as f32)
            .div(100_f32);
        defmt::info!("POLL_DC_BUS_VOLTAGE {:#02x}, {}", result, bus_voltage);
        Ok(Some(bus_voltage as u8))
    }
    fn get_set_frequency(
        &mut self,
        _cs: &cortex_m::interrupt::CriticalSection,
        rs485: &mut RS485,
        delay: &mut impl DelayMs<u16>,
    ) -> Result<Option<u8>, VfdError> {
        let mut rsp = [0; RESPONSE_BUF_SIZE];
        let (req, computed_rsp_len) = self.modbus.build_request(0x2102, 0x03, 1)?;
        rs485.poll(&req, &mut rsp, computed_rsp_len, delay, _cs)?;
        let mut result = [0_u16; 1];
        self.modbus.check_confirmation(&req, &rsp)?;
        self.modbus.check_integrity(&rsp)?;
        self.modbus.decode(&rsp, &mut result);

        let set_freq = result[0].div_ceil(100);
        Ok(Some(set_freq as u8))
    }
    fn get_output_frequency(
        &mut self,
        _cs: &cortex_m::interrupt::CriticalSection,
        rs485: &mut RS485,
        delay: &mut impl DelayMs<u16>,
    ) -> Result<Option<i8>, VfdError> {
        let mut rsp = [0; RESPONSE_BUF_SIZE];
        let (req, computed_rsp_len) = self.modbus.build_request(0x2103, 0x03, 1)?;
        rs485.poll(&req, &mut rsp, computed_rsp_len, delay, _cs)?;
        let mut result = [0_u16; 1];
        self.modbus.check_confirmation(&req, &rsp)?;
        self.modbus.check_integrity(&rsp)?;
        self.modbus.decode(&rsp, &mut result);

        let output_freq = result[0].div_floor(100) as u8;
        let output_freq = match self.direction {
            Direction::Forward => (output_freq.max(0)) as i8,
            Direction::Reverse => (output_freq.max(0).sub(0)) as i8,
            Direction::Unset => 0,
            Direction::NotRead => 0,
        };
        Ok(Some(output_freq))
    }
}
