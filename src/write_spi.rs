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
                .bit(mode.phase == Phase::CaptureOnSecondTransition)
                .cpol()
                .bit(mode.polarity == Polarity::IdleHigh)
                .mstr()
                .master()
                .br()
                .div256()
                .lsbfirst()
                .clear_bit()
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
        }
    }

    pub fn set_voltage(&mut self, channel: Channel, voltage: f32) {
        let clamped_voltage: f32;
        if voltage < 0.0 {
            clamped_voltage = 0.0;
        } else if voltage > 3.3 {
            clamped_voltage = 3.3;
        } else {
            clamped_voltage = voltage;
        }

        let duty: u16 = ((clamped_voltage * (2_u16.pow(12) as f32 - 1.0)) / 3.3) as u16;
        let header: u16;
        match channel {
            Channel::A => header = 0b0011,
            Channel::B => header = 0b1011,
        }

        self.cs.set_low().unwrap();
        let data = header << 12 | duty;
        while self.spi.sr.read().ftlvl().bits() > 0 {}
        self.spi.dr.write(|w| unsafe { w.bits(data as u32) });
        while self.spi.sr.read().txe().bit_is_clear() || self.spi.sr.read().bsy().bit_is_set() {}
        self.cs.set_high().unwrap();
    }
}
