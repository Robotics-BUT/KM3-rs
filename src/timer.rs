use embedded_hal::timer::{CountDown, Periodic};
use stm32f0xx_hal::rcc::{Clocks, Rcc};
use stm32f0xx_hal::time::Hertz;
use void::Void;

pub trait Updatable {
    type Time;

    fn update<T: Into<Self::Time>>(&mut self, timeout: T);
}

/// Interrupt events
pub enum Event {
    /// Timer timed out / count down ended
    TimeOut,
}

pub struct Timer<TIM> {
    tim: TIM,
    clocks: Clocks,
}

macro_rules! my_timers {
    ($($TIM:ident: ($tim:ident, $timXen:ident, $timXrst:ident, $apbenr:ident, $apbrstr:ident),)+) => {
        $(
            use stm32f0xx_hal::pac::$TIM;
            impl Timer<$TIM> {
                // XXX(why not name this `new`?) bummer: constructors need to have different names
                // even if the `$TIM` are non overlapping (compare to the `free` function below
                // which just works)
                /// Configures a TIM peripheral as a periodic count down timer
                pub fn $tim<T>(tim: $TIM, timeout: T, rcc: &mut Rcc) -> Self
                where
                    T: Into<Hertz>,
                {
                    // enable and reset peripheral to a clean slate state
                    let raw_rcc = unsafe { &(*stm32f0xx_hal::pac::RCC::ptr()) };
                    raw_rcc.$apbenr.modify(|_, w| w.$timXen().set_bit());
                    raw_rcc.$apbrstr.modify(|_, w| w.$timXrst().set_bit());
                    raw_rcc.$apbrstr.modify(|_, w| w.$timXrst().clear_bit());

                    let mut timer = Timer {
                        clocks: rcc.clocks,
                        tim,
                    };
                    timer.start(timeout);

                    timer
                }

                /// Starts listening for an `event`
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt
                            self.tim.dier.write(|w| w.uie().set_bit());
                        }
                    }
                }

                /// Stops listening for an `event`
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt
                            self.tim.dier.write(|w| w.uie().clear_bit());
                        }
                    }
                }

                /// Releases the TIM peripheral
                pub fn release(self) -> $TIM {
                    let rcc = unsafe { &(*stm32f0xx_hal::pac::RCC::ptr()) };
                    // Pause counter
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());
                    // Disable timer
                    rcc.$apbenr.modify(|_, w| w.$timXen().clear_bit());
                    self.tim
                }
            }

            impl CountDown for Timer<$TIM> {
                type Time = Hertz;

                /// Start the timer with a `timeout`
                fn start<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
                {
                    // pause
                    // self.tim.cr1.modify(|_, w| w.cen().clear_bit());
                    // // restart counter
                    // self.tim.cnt.reset();

                    let frequency = timeout.into().0;

                    if frequency == 0 {
                        return;
                    }

                    // If pclk is prescaled from hclk, the frequency fed into the timers is doubled
                    let tclk = if self.clocks.hclk().0 == self.clocks.pclk().0 {
                        self.clocks.pclk().0
                    } else {
                        self.clocks.pclk().0 * 2
                    };
                    let ticks = tclk / frequency;

                    let psc = cast::u16((ticks - 1) / (1 << 16)).unwrap();
                    self.tim.psc.write(|w| w.psc().bits(psc));

                    let arr = cast::u16(ticks / cast::u32(psc + 1)).unwrap();
                    self.tim.arr.write(|w| unsafe { w.bits(cast::u32(arr)) });

                    // start counter
                    self.tim.cr1.modify(|_, w| w.cen().set_bit());
                }

                /// Return `Ok` if the timer has wrapped
                /// Automatically clears the flag and restarts the time
                fn wait(&mut self) -> nb::Result<(), Void> {
                    if self.tim.sr.read().uif().bit_is_clear() {
                        Err(nb::Error::WouldBlock)
                    } else {
                        self.tim.sr.modify(|_, w| w.uif().clear_bit());
                        Ok(())
                    }
                }
            }

            impl Updatable for Timer<$TIM> {
                type Time = Hertz;

                fn update<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
                {
                    let frequency = timeout.into().0;

                    if frequency == 0 {
                        self.tim.cr1.modify(|_, w| w.cen().clear_bit());
                        return;
                    } else {
                        self.tim.cr1.modify(|_, w| w.cen().set_bit());
                    }

                    // If pclk is prescaled from hclk, the frequency fed into the timers is doubled
                    let tclk = if self.clocks.hclk().0 == self.clocks.pclk().0 {
                        self.clocks.pclk().0
                    } else {
                        self.clocks.pclk().0 * 2
                    };
                    let ticks = tclk / frequency;

                    let psc = cast::u16((ticks - 1) / (1 << 16)).unwrap();
                    self.tim.psc.write(|w| w.psc().bits(psc));

                    let arr = cast::u16(ticks / cast::u32(psc + 1)).unwrap();
                    self.tim.arr.write(|w| unsafe { w.bits(cast::u32(arr)) });
                }
            }

            impl Periodic for Timer<$TIM> {}
        )+
    }
}

my_timers! {
    TIM1: (tim1, tim1en, tim1rst, apb2enr, apb2rstr),
    TIM3: (tim3, tim3en, tim3rst, apb1enr, apb1rstr),
    TIM14: (tim14, tim14en, tim14rst, apb1enr, apb1rstr),
    TIM16: (tim16, tim16en, tim16rst, apb2enr, apb2rstr),
    TIM17: (tim17, tim17en, tim17rst, apb2enr, apb2rstr),
    TIM2: (tim2, tim2en, tim2rst, apb1enr, apb1rstr),
}
