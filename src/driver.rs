use crate::timer::Updatable;
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use embedded_hal::timer::{CountDown, Periodic};
use stm32f0xx_hal::gpio::{Output, Pin, PushPull};
use stm32f0xx_hal::prelude::*;
use stm32f0xx_hal::time::Hertz;
use void::Void;

pub struct Driver<TIM> {
    timer: TIM,
    step: Pin<Output<PushPull>>,
    dir: Pin<Output<PushPull>>,
    current_speed: f32,
    target_speed: f32,
    step_value: f32,
}

impl<TIM: Periodic + CountDown + Updatable> Driver<TIM> {
    pub fn new(timer: TIM, step: Pin<Output<PushPull>>, dir: Pin<Output<PushPull>>) -> Self {
        Self {
            timer,
            step,
            dir,
            current_speed: 0.0,
            target_speed: 0.0,
            step_value: 1.5,
        }
    }

    pub fn tick(&mut self) -> nb::Result<(), Void> {
        self.timer.wait()?;
        self.step.toggle().unwrap();
        Ok(())
    }

    /// Sets speed of the motor, speed is set in RPM
    /// # Arguments
    /// * `speed` - target speed in RPM
    pub fn set_speed(&mut self, speed: f32) {
        self.target_speed = speed;
    }

    pub fn update(&mut self)
    where
        <TIM as CountDown>::Time: From<Hertz>,
        <TIM as Updatable>::Time: From<Hertz>,
    {
        let diff = self.target_speed - self.current_speed;
        if fabs(diff) < self.step_value {
            self.current_speed = self.target_speed;
        } else {
            self.current_speed += fsign(diff) * self.step_value;
        }

        if self.current_speed < 0.0f32 {
            self.dir.set_low().unwrap();
        } else {
            self.dir.set_high().unwrap();
        }

        // double the frequency to generatee rising edges at the desired frequency
        let raw_speed = (self.current_speed / 60.0f32 * 200f32 * 2.0 * 32f32) as u32;
        self.timer.update(raw_speed.hz());
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
