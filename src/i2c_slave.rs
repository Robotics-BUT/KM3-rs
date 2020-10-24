use stm32f0xx_hal::pac::I2C1;

pub struct I2CSlave {
    i2c: I2C1,
}

// direction as specified in the datasheet
#[derive(PartialEq)]
pub enum Direction {
    Write, // slave is receiver
    Read,  // slave is transmitter
}

impl From<Direction> for bool {
    fn from(dir: Direction) -> Self {
        match dir {
            Direction::Write => false,
            Direction::Read => true,
        }
    }
}

impl From<bool> for Direction {
    fn from(raw: bool) -> Self {
        if raw {
            Direction::Read
        } else {
            Direction::Write
        }
    }
}

pub enum Status {
    AddressMatch(Direction),
    Busy,
    Timeout,
    Overrun,
    ArbitrationLost,
    BusError,
    TransferCompleteReload,
    Stop,
    NACKReceived,
    RxNotEmpty,
    TxDataMustBeWritten,
    TxEmpty,
}

impl I2CSlave {
    pub fn new(i2c: I2C1) -> Self {
        let rcc = unsafe { &(*stm32f0xx_hal::pac::RCC::ptr()) };
        rcc.apb1enr.modify(|_, w| w.i2c1en().enabled());
        rcc.apb1rstr.modify(|_, w| w.i2c1rst().reset());

        i2c.cr1.write(|w| {
            w.nostretch()
                .disabled() // enable clock stretching
                .anfoff()
                .enabled() // enable analog filter
                .dnf()
                .no_filter() // disable digital filter
                .errie()
                .enabled() // error interrupt enabled
                .tcie()
                .enabled() // transmission complete interrupt enabled
                .stopie()
                .enabled() // stop interrupt enabled
                .nackie()
                .enabled() // nack interrupt enabled
                .addrie()
                .enabled() // address match interrupt enabled
                .rxie() // rx interrupt enabled
                .enabled()
                .txie() // tx interrupt enabled
                .enabled()
                .wupen()
                .enabled() // wake up when address match
        });

        // decide about using sbc with nbytes
        // set up timing for nostretch mode

        i2c.oar1
            .write(|w| w.oa1en().enabled().oa1().bits(0x55).oa1mode().bit7());

        i2c.cr1.modify(
            |_, w| w.pe().enabled(), // enable peripheral
        );

        I2CSlave { i2c }
    }

    pub fn is_status(&self, status: Status, clear: bool) -> bool {
        let isr = self.i2c.isr.read();

        match status {
            Status::AddressMatch(dir) => {
                if isr.addr().bit_is_set() {
                    if dir != isr.dir().bit().into() {
                        return false;
                    }
                    if clear {
                        self.i2c.icr.write(|w| w.addrcf().clear());
                    }
                    return true;
                } else {
                    false
                }
            }
            Status::Busy => {
                return isr.busy().bit_is_set();
            }
            Status::Timeout => {
                if isr.timeout().bit_is_set() {
                    if clear {
                        self.i2c.icr.write(|w| w.timoutcf().clear());
                    }
                    return true;
                }
                return false;
            }
            Status::Overrun => {
                if isr.ovr().bit_is_set() {
                    if clear {
                        self.i2c.icr.write(|w| w.ovrcf().clear());
                    }
                    return true;
                }
                return false;
            }
            Status::ArbitrationLost => {
                if isr.arlo().bit_is_set() {
                    if clear {
                        self.i2c.icr.write(|w| w.arlocf().clear());
                    }
                    return true;
                }
                return false;
            }
            Status::BusError => {
                if isr.berr().bit_is_set() {
                    if clear {
                        self.i2c.icr.write(|w| w.berrcf().clear());
                    }
                    return true;
                }
                return false;
            }
            Status::TransferCompleteReload => {
                if isr.tcr().bit_is_set() {
                    if clear {
                        defmt::error!("Cannot be cleared.");
                    }
                    return true;
                }
                return false;
            }
            Status::Stop => {
                if isr.stopf().bit_is_set() {
                    if clear {
                        self.i2c.icr.write(|w| w.stopcf().clear());
                    }
                    return true;
                }
                return false;
            }
            Status::NACKReceived => {
                if isr.nackf().bit_is_set() {
                    if clear {
                        self.i2c.icr.write(|w| w.nackcf().clear());
                    }
                    return true;
                }
                return false;
            }
            Status::RxNotEmpty => {
                if isr.rxne().bit_is_set() {
                    if clear {
                        defmt::error!("Cannot be cleared.");
                    }
                    return true;
                }
                return false;
            }
            Status::TxDataMustBeWritten => {
                if isr.txis().bit_is_set() {
                    if clear {
                        defmt::error!("Cannot be cleared.");
                    }
                    return true;
                }
                return false;
            }
            Status::TxEmpty => {
                if isr.txe().bit_is_set() {
                    if clear {
                        defmt::error!("Cannot be cleared.");
                    }
                    return true;
                }
                return false;
            }
        }
    }
}
