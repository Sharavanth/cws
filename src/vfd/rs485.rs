#![allow(dead_code)]
use core::{
    convert::Infallible,
    ops::{Add, Mul},
};
use defmt::Format;
use embedded_hal::blocking::delay::DelayMs;
use fugit::ExtU32;
use nb::block;
use stm32f1xx_hal::{
    device::{TIM3, USART2},
    gpio::{Output, PA4},
    serial::{Rx, Tx},
    timer::Counter,
};

#[derive(Debug, Clone, Format, Copy)]
pub enum RS485State {
    Idle,
    Transmitting,
    Receiving,
}
#[derive(Debug, Clone, Format)]
pub enum RS485Error {
    Write,
    NoData,
    Timeout,
}

const BUFFER_LEN: usize = 8 * 4;
const TRANSMISSION_DELAY: u16 = 50;
pub struct RS485 {
    tx: Tx<USART2>,
    rx: Rx<USART2>,
    rts: PA4<Output>,
    counter: Counter<TIM3, 1000>,
    // rx buffer
    buf_idx: usize,
    buf: [u8; BUFFER_LEN],
    // state
    _counter: u16,
    _predelay: u16,
    _postdelay: u16,
    _state: RS485State,
    _transmision_begun: bool,
}

impl RS485 {
    pub fn new(
        tx: Tx<USART2>,
        rx: Rx<USART2>,
        rts: PA4<Output>,
        timer: Counter<TIM3, 1000>,
    ) -> Self {
        let mut rs485 = RS485 {
            tx,
            rx,
            rts,
            counter: timer,
            // rx buffer
            buf_idx: 0,
            buf: [0; BUFFER_LEN],
            // state
            _predelay: 0,
            _postdelay: 0,
            _counter: 0,
            _transmision_begun: true,
            _state: RS485State::Idle,
        };
        rs485.set_delays(TRANSMISSION_DELAY, 0);
        rs485
    }

    pub fn poll(&mut self, _cs: &cortex_m::interrupt::CriticalSection) -> RS485State {
        self._state
    }

    pub fn send(
        &mut self,
        _cs: &cortex_m::interrupt::CriticalSection,
        req: &[u8],
        computed_rsp_len: u8,
        delay: &mut impl DelayMs<u16>,
    ) -> Result<(), RS485Error> {
        self.begin_transmission(delay)?;
        self.write_all(req)?;
        self.end_transmission(delay)?;

        let timeout = (computed_rsp_len as u32)
            .mul(2)
            .add(4_u32.mul(4))
            .add(10)
            .min(2000);
        self.counter.cancel();
        self.counter.start(timeout.millis());
        self._counter += 1;
        self._state = RS485State::Receiving;
        defmt::trace!("Receiving, CNT {}, TO {} ms", self._counter, timeout);
        Ok(())
    }
    pub fn read(
        &mut self,
        _cs: &cortex_m::interrupt::CriticalSection,
        rsp: &mut [u8],
    ) -> Result<(), RS485Error> {
        if self.counter.wait().is_ok() {
            self._state = RS485State::Idle;
            defmt::trace!("TIMED OUT");
            self.counter.cancel();
            if !self._transmision_begun && self.buf_idx < 1 {
                Err(RS485Error::Timeout)
            } else {
                let idx = self.buf_idx;
                self.buf_idx = 0;
                rsp[0..idx].copy_from_slice(&self.buf[0..idx]);
                Ok(())
            }
        } else if !self._transmision_begun && self.buf_idx < 1 {
            self._state = RS485State::Receiving;
            Err(RS485Error::NoData)
        } else {
            defmt::trace!("TRYING TO READ SINCE RX COMPLETE, {:#02x}", self.buf);
            self._state = RS485State::Idle;
            self.counter.cancel();
            let idx = self.buf_idx;
            self.buf_idx = 0;
            rsp[0..idx].copy_from_slice(&self.buf[0..idx]);
            Ok(())
        }
    }
    pub fn tx_listener(&mut self, _cs: &cortex_m::interrupt::CriticalSection) {
        let tx = &mut self.tx;
        if tx.is_tx_complete() {
            self.receive();
        }
    }
    pub fn rx_listener(&mut self, _cs: &cortex_m::interrupt::CriticalSection) {
        let rx = &mut self.rx;

        if rx.is_rx_not_empty() {
            if let Ok(byte) = block!(rx.read()) {
                self.buf[self.buf_idx] = byte;
                self.buf_idx += 1;
                defmt::trace!("Byte {:#02x}", byte);
            }
        } else {
            rx.listen();
        }
    }

    fn set_delays(&mut self, pre: u16, post: u16) {
        self._predelay = pre;
        self._postdelay = post;
    }
    fn begin_transmission(&mut self, delay: &mut impl DelayMs<u16>) -> Result<(), RS485Error> {
        if self._predelay > 0 {
            delay.delay_ms(self._predelay);
        }
        self.no_receive();
        self._transmision_begun = true;
        Ok(())
    }
    fn end_transmission(&mut self, delay: &mut impl DelayMs<u16>) -> Result<(), RS485Error> {
        if self._postdelay > 0 {
            delay.delay_ms(self._postdelay);
        }

        self._transmision_begun = false;
        Ok(())
    }
    fn write_all(&mut self, words: &[u8]) -> Result<(), RS485Error> {
        if !self._transmision_begun {
            return Err(RS485Error::Write);
        }

        self.flush();
        self.tx.bwrite_all(words);
        Ok(())
    }
    fn write(&mut self, word: u8) -> nb::Result<(), Infallible> {
        if !self._transmision_begun {
            return Err(nb::Error::WouldBlock);
        }

        self.flush();
        self.tx.write(word);
        Ok(())
    }
    fn receive(&mut self) {
        defmt::trace!("Receiving. TC unset. TX set");
        if self.rts.is_set_high() {
            self.rts.set_low();
        }
        self.tx.unlisten();
        // start listening to rx events till idle
        self.rx.listen();
    }
    fn no_receive(&mut self) {
        defmt::trace!("Not Receiving. TC set. TX unset");
        if self.rts.is_set_low() {
            self.rts.set_high();
        }
        self.tx.listen();
        self.rx.unlisten();
    }
    fn flush(&mut self) -> nb::Result<(), Infallible> {
        self.tx.flush()
    }
}
