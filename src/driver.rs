use crate::write_spi::{Channel, MCP4922};
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use stm32f0xx_hal::gpio::gpioa::{PA5, PA7};
use stm32f0xx_hal::gpio::{Alternate, Input, Output, Pin, PullUp, PushPull, AF0};
use stm32f0xx_hal::prelude::*;
use stm32f0xx_hal::rcc::Clocks;
use stm32f0xx_hal::stm32;
use stm32f0xx_hal::time::Hertz;
const SENSE_R: f32 = 0.1;
const FAILSAFE_COUNTER_TOP: u8 = 4;

struct Part {
    step: Pin<Output<PushPull>>,
    dir: Pin<Output<PushPull>>,
    current_speed: f32,
    target_speed: f32,
    count: i32,
}

pub struct Driver {
    timer1: stm32::TIM14,
    timer2: stm32::TIM16,
    update_timer: stm32::TIM2,
    failsafe_timer: stm32::TIM3,
    failsafe_counter: u8,
    dac: MCP4922<PA7<Alternate<AF0>>, PA5<Alternate<AF0>>>,
    fault: Pin<Input<PullUp>>,
    enable: Pin<Output<PushPull>>,
    first_motor: Part,
    second_motor: Part,
    step_value: f32,
    clocks: Clocks,
}

impl Driver {
    pub fn new(
        timer14: stm32::TIM14,
        timer16: stm32::TIM16,
        update_timer: stm32::TIM2,
        failsafe_timer: stm32::TIM3,
        dac: MCP4922<PA7<Alternate<AF0>>, PA5<Alternate<AF0>>>,
        step1: Pin<Output<PushPull>>,
        dir1: Pin<Output<PushPull>>,
        step2: Pin<Output<PushPull>>,
        dir2: Pin<Output<PushPull>>,
        mut enable: Pin<Output<PushPull>>,
        fault: Pin<Input<PullUp>>,
        clocks: Clocks,
    ) -> Self {
        let raw_rcc = unsafe { &(*stm32f0xx_hal::pac::RCC::ptr()) };
        raw_rcc
            .apb1enr
            .modify(|_, w| w.tim2en().enabled().tim3en().enabled().tim14en().enabled());
        raw_rcc.apb2enr.modify(|_, w| w.tim16en().set_bit());
        raw_rcc.apb1rstr.modify(|_, w| {
            w.tim2rst()
                .set_bit()
                .tim3rst()
                .set_bit()
                .tim14rst()
                .set_bit()
        });
        raw_rcc.apb2rstr.modify(|_, w| w.tim16rst().set_bit());
        raw_rcc.apb1rstr.modify(|_, w| {
            w.tim2rst()
                .clear_bit()
                .tim3rst()
                .clear_bit()
                .tim14rst()
                .clear_bit()
        });
        raw_rcc.apb2rstr.modify(|_, w| w.tim16rst().clear_bit());

        timer14.dier.write(|w| w.uie().set_bit());
        timer16.dier.write(|w| w.uie().set_bit());
        update_timer.dier.write(|w| w.uie().set_bit());
        failsafe_timer.dier.write(|w| w.uie().set_bit());

        enable.set_low().unwrap();

        let mut s = Self {
            timer1: timer14,
            timer2: timer16,
            update_timer,
            failsafe_timer,
            failsafe_counter: 0,
            dac,
            fault,
            enable,
            first_motor: Part {
                step: step1,
                dir: dir1,
                current_speed: 0.0,
                target_speed: 0.0,
                count: 0,
            },
            second_motor: Part {
                step: step2,
                dir: dir2,
                current_speed: 0.0,
                target_speed: 0.0,
                count: 0,
            },
            step_value: 1.0,
            clocks,
        };

        s.update_update_timer(&50.hz());
        s.update_failsafe_timer(&4.hz());

        s
    }

    /// Sets speeds of the motors, speeds are set in RPM
    /// # Arguments
    /// * `motor1_speed` - target speed in RPM
    /// * `motor2_speed` - target speed in RPM
    pub fn set_speeds(&mut self, motor1_speed: f32, motor2_speed: f32) {
        self.first_motor.target_speed = motor1_speed;
        self.second_motor.target_speed = motor2_speed;
        self.failsafe_counter = FAILSAFE_COUNTER_TOP;
    }

    pub fn get_raw_odometry(&mut self) -> (i32, i32) {
        let odometry = (self.first_motor.count, self.second_motor.count);
        self.first_motor.count = 0;
        self.second_motor.count = 0;
        odometry
    }

    /// Updates the current motor speeds using ramps
    fn update(&mut self) {
        let (current1, freq1) = self.first_motor.update(self.step_value);
        let (current2, freq2) = self.second_motor.update(self.step_value);
        self.update_timer1(&freq1);
        self.update_timer2(&freq2);

        if freq1.0 == 0 && freq2.0 == 0 {
            self.enable.set_high().unwrap();
        } else {
            self.enable.set_low().unwrap();
        }
        let voltage1 = current1 * 5.0 * SENSE_R;
        let voltage2 = current2 * 5.0 * SENSE_R;
        self.dac.set_voltage(Channel::A, voltage1);
        self.dac.set_voltage(Channel::B, voltage2);
        self.dac.update();
    }

    fn run_failsafe_check(&mut self) {
        if self.failsafe_counter == 0 {
            self.set_speeds(0.0, 0.0);
            defmt::warn!("Failsafe timer engaged.");
        } else {
            self.failsafe_counter -= 1;
        }
    }

    pub fn tim2_tick(&mut self) {
        self.update_timer.sr.modify(|_, w| w.uif().clear_bit());
        self.update();
    }

    pub fn tim3_tick(&mut self) {
        self.failsafe_timer.sr.modify(|_, w| w.uif().clear_bit());
        self.run_failsafe_check();
    }

    pub fn tim14_tick(&mut self) {
        self.timer1.sr.modify(|_, w| w.uif().clear_bit());
        self.first_motor.on_tick();
    }

    pub fn tim16_tick(&mut self) {
        self.timer2.sr.modify(|_, w| w.uif().clear_bit());
        self.second_motor.on_tick();
    }

    fn freq_to_timer_settings<T>(timeout: &T, clocks: Clocks) -> (u16, u16)
    where
        T: Into<Hertz> + Copy,
    {
        // If pclk is prescaled from hclk, the frequency fed into the timers is doubled
        let tclk = if clocks.hclk().0 == clocks.pclk().0 {
            clocks.pclk().0
        } else {
            clocks.pclk().0 * 2
        };
        let f: u32 = (*timeout).into().0;
        let ticks = tclk / f;

        let psc = cast::u16((ticks - 1) / (1 << 16)).unwrap();
        let arr = cast::u16(ticks / cast::u32(psc + 1)).unwrap();

        (psc, arr)
    }

    fn update_timer1<T>(&mut self, timeout: &T)
    where
        T: Into<Hertz> + Copy,
    {
        let frequency = (*timeout).into().0;

        if frequency == 0 {
            self.timer1.cr1.modify(|_, w| w.cen().clear_bit());
            return;
        } else {
            self.timer1.cr1.modify(|_, w| w.cen().set_bit());
        }

        let (psc, arr) = Self::freq_to_timer_settings(timeout, self.clocks);

        self.timer1.psc.write(|w| w.psc().bits(psc));
        self.timer1.arr.write(|w| unsafe { w.bits(cast::u32(arr)) });
    }

    fn update_timer2<T>(&mut self, timeout: &T)
    where
        T: Into<Hertz> + Copy,
    {
        let frequency = (*timeout).into().0;

        if frequency == 0 {
            self.timer2.cr1.modify(|_, w| w.cen().clear_bit());
            return;
        } else {
            self.timer2.cr1.modify(|_, w| w.cen().set_bit());
        }

        let (psc, arr) = Self::freq_to_timer_settings(timeout, self.clocks);
        self.timer2.psc.write(|w| w.psc().bits(psc));
        self.timer2.arr.write(|w| unsafe { w.bits(cast::u32(arr)) });
    }
}

macro_rules! update_timer {
    ($t:ident, $handler: ident) => {
        impl Driver {
            fn $handler<T>(&mut self, timeout: &T)
            where
                T: Into<Hertz> + Copy,
            {
                let frequency = (*timeout).into().0;

                if frequency == 0 {
                    self.$t.cr1.modify(|_, w| w.cen().clear_bit());
                    return;
                } else {
                    self.$t.cr1.modify(|_, w| w.cen().set_bit());
                }

                let (psc, arr) = Self::freq_to_timer_settings(timeout, self.clocks);
                self.$t.psc.write(|w| w.psc().bits(psc));
                self.$t.arr.write(|w| unsafe { w.bits(cast::u32(arr)) });
            }
        }
    };
}

update_timer!(update_timer, update_update_timer);
update_timer!(failsafe_timer, update_failsafe_timer);

impl Part {
    pub fn on_tick(&mut self) {
        self.step.toggle().unwrap();
        self.count += if self.dir.is_set_high().unwrap() {
            1
        } else {
            -1
        };
    }

    pub fn update(&mut self, step: f32) -> (f32, Hertz) {
        let diff = self.target_speed - self.current_speed;
        if fabs(diff) < step {
            self.current_speed = self.target_speed;
        } else {
            self.current_speed += fsign(diff) * step;
        }

        if self.current_speed < 0.0f32 {
            self.dir.set_low().unwrap();
        } else {
            self.dir.set_high().unwrap();
        }

        let current = if self.current_speed == 0.0 && self.target_speed == 0.0 {
            0.5 // steppers are stopped
        } else if self.current_speed == self.target_speed {
            0.5 // steppers are running at a constant speed
        } else {
            0.7 // steppers are accelerating or decelerating
        };

        // double the frequency to generatee rising edges at the desired frequency
        let raw_speed = (fabs(self.current_speed) / 60.0f32 * 200f32 * 2.0 * 32f32) as u32;
        (current, raw_speed.hz())
    }
}

#[no_mangle]
pub fn fabs(a: f32) -> f32 {
    if a < 0.0 {
        -a
    } else {
        a
    }
}

#[no_mangle]
pub fn fsign(a: f32) -> f32 {
    if a == 0.0 {
        0.0
    } else if a > 0.0 {
        1.0
    } else {
        -1.0
    }
}
