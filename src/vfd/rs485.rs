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

#[derive(Debug, Format, PartialEq)]
enum RS485State {
    Idle,
    Receiving,
}
#[derive(Debug, Clone, Format)]
pub enum RS485Error {
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
    pre_delay: u16,
    post_delay: u16,
    state: RS485State,
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
            pre_delay: 0,
            post_delay: 0,
            state: RS485State::Idle,
        };
        rs485.set_delays(TRANSMISSION_DELAY, 0);
        rs485
    }

    pub fn poll(
        &mut self,
        req: &[u8],
        rsp: &mut [u8],
        computed_rsp_len: u8,
        delay: &mut impl DelayMs<u16>,
        _cs: &cortex_m::interrupt::CriticalSection,
    ) -> Result<(), RS485Error> {
        let timeout = (computed_rsp_len as u32)
            .mul(2)
            .add(4_u32.mul(4))
            .add(10)
            .max(500);

        self.begin_transmission(delay)?;
        self.write_all(req)?;
        self.end_transmission(delay)?;

        self.counter.cancel();
        self.counter.start(timeout.millis());
        self.state = RS485State::Receiving;

        self.try_read(rsp)
    }

    fn try_read(&mut self, rsp: &mut [u8]) -> Result<(), RS485Error> {
        if self.state == RS485State::Receiving {
            if self.counter.wait().is_ok() {
                self.state = RS485State::Idle;
                defmt::trace!("TIMED OUT");
                self.counter.cancel();
                Err(RS485Error::Timeout)
            } else if self.buf_idx >= 1 {
                defmt::trace!("TRYING TO READ SINCE RX COMPLETE, {:#02x}", self.buf);
                self.state = RS485State::Idle;
                self.counter.cancel();
                let idx = self.buf_idx;
                self.buf_idx = 0;
                rsp[0..idx].copy_from_slice(&self.buf[0..idx]);
                Ok(())
            } else {
                self.try_read(rsp)
            }
        } else {
            Ok(())
        }
    }

    pub fn rs485_listener(&mut self, _cs: &cortex_m::interrupt::CriticalSection) {
        if self.rx.is_rx_not_empty() {
            if let Ok(byte) = block!(self.rx.read()) {
                self.buf[self.buf_idx] = byte;
                self.buf_idx += 1;
            }
        } else if self.tx.is_tx_complete() {
            self.listen_for_rxne();
        } else {
            self.rx.listen();
        }
    }

    fn set_delays(&mut self, pre: u16, post: u16) {
        self.pre_delay = pre;
        self.post_delay = post;
    }
    fn begin_transmission(&mut self, delay: &mut impl DelayMs<u16>) -> Result<(), RS485Error> {
        if self.pre_delay > 0 {
            delay.delay_ms(self.pre_delay);
        }
        self.listen_for_tc();
        Ok(())
    }
    fn end_transmission(&mut self, delay: &mut impl DelayMs<u16>) -> Result<(), RS485Error> {
        if self.post_delay > 0 {
            delay.delay_ms(self.post_delay);
        }
        Ok(())
    }
    fn write_all(&mut self, words: &[u8]) -> Result<(), RS485Error> {
        self.flush();
        self.tx.bwrite_all(words);
        Ok(())
    }
    fn listen_for_rxne(&mut self) {
        if self.rts.is_set_high() {
            self.rts.set_low();
        }
        self.tx.unlisten();
        // start listening to rx events till idle
        self.rx.listen();
    }
    fn listen_for_tc(&mut self) {
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
