use stm32f0xx_hal::gpio::{Output, Pin, PushPull};
use stm32f0xx_hal::prelude::*;
use stm32f0xx_hal::spi::{MosiPin, Phase, Polarity, SckPin};
use stm32f0xx_hal::stm32;
use stm32f0xx_hal::stm32::SPI1;

use embedded_hal::spi::MODE_0;

pub enum Channel {
    A,
    B,
}

pub struct MCP4922<MOSI, SCK> {
    _sck: SCK,
    _mosi: MOSI,
    cs: Pin<Output<PushPull>>,
    spi: SPI1,
    raw_data_a: u16,
    raw_data_b: u16,
}

impl<MOSI, SCK> MCP4922<MOSI, SCK>
where
    MOSI: MosiPin<SPI1>,
    SCK: SckPin<SPI1>,
{
    pub fn new(spi: SPI1, mosi: MOSI, sck: SCK, cs: Pin<Output<PushPull>>) -> Self {
        unsafe {
            let rcc = &(*stm32::RCC::ptr());
            rcc.apb2enr.modify(|_, w| w.spi1en().enabled());
            rcc.apb2rstr.modify(|_, w| w.spi1rst().reset());
            rcc.apb2rstr.modify(|_, w| w.spi1rst().clear_bit());
        }

        let mode = MODE_0;
        spi.cr1.write(|w| {
            w.cpha()
                .first_edge()
                .cpol()
                .idle_low()
                .mstr()
                .master()
                .br()
                .div8()
                .lsbfirst()
                .msbfirst()
                .ssm()
                .enabled()
                .ssi()
                .set_bit()
                .rxonly()
                .clear_bit()
                .bidimode()
                .clear_bit()
                .spe()
                .enabled()
        });

        spi.cr2
            .write(|w| w.frxth().half().ds().sixteen_bit().ssoe().disabled());

        Self {
            _sck: sck,
            _mosi: mosi,
            cs,
            spi,
            raw_data_a: 0,
            raw_data_b: 0,
        }
    }

    pub fn set_voltage(&mut self, channel: Channel, voltage: f32) {
        match channel {
            Channel::A => self.raw_data_a = Self::voltage_to_raw(voltage),
            Channel::B => self.raw_data_b = Self::voltage_to_raw(voltage),
        }
    }

    fn voltage_to_raw(voltage: f32) -> u16 {
        let clamped_voltage: f32;
        if voltage < 0.0 {
            clamped_voltage = 0.0;
        } else if voltage > 3.3 {
            clamped_voltage = 3.3;
        } else {
            clamped_voltage = voltage;
        }

        ((clamped_voltage * 0xfff as f32) / 3.3) as u16
    }

    pub fn update(&mut self) {
        let raw_a = (0b0011 << 12) | self.raw_data_a;
        let raw_b = (0b1011 << 12) | self.raw_data_b;

        self.cs.set_low().unwrap();
        while !self.spi.sr.read().txe().bit() {}
        self.spi.dr.write(|w| unsafe { w.bits(raw_a as u32) });
        while self.spi.sr.read().bsy().bit_is_set() {}
        self.cs.set_high().unwrap();

        self.cs.set_low().unwrap();
        while !self.spi.sr.read().txe().bit() {}
        self.spi.dr.write(|w| unsafe { w.bits(raw_b as u32) });
        while self.spi.sr.read().bsy().bit_is_set() {}
        self.cs.set_high().unwrap();
    }
}
