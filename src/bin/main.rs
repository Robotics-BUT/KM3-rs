#![no_main]
#![no_std]

use byteorder::{ByteOrder, LittleEndian};
use core::convert::TryFrom;
use km3_rs as _; // global logger + panicking-behavior + memory layout
use km3_rs::driver::Driver;
use km3_rs::i2c_slave::{I2CSlave, State};
use km3_rs::mcp4922::{Channel, MCP4922};
use stm32f0xx_hal::delay::Delay;
use stm32f0xx_hal::gpio::gpioa::{PA10, PA9};
use stm32f0xx_hal::gpio::{Alternate, Output, Pin, PushPull, AF4};
use stm32f0xx_hal::prelude::*;

type I2C = I2CSlave<PA10<Alternate<AF4>>, PA9<Alternate<AF4>>>;

const I2C_ADDRESS: u8 = 0x55;

enum Register {
    TargetSpeed,
    Odometry,
    RampFrequency,
    RampStep,
    FaultStatus,
}

impl TryFrom<u8> for Register {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x10 => Ok(Self::TargetSpeed),
            0x20 => Ok(Self::Odometry),
            0x30 => Ok(Self::RampFrequency),
            0x40 => Ok(Self::RampStep),
            0x50 => Ok(Self::FaultStatus),
            _ => Err(()),
        }
    }
}

#[rtic::app(device = stm32f0xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        led: Pin<Output<PushPull>>,
        delay: Delay,
        driver: Driver,
        i2c_slave: I2C,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        let core: cortex_m::Peripherals = cx.core;
        let mut device: stm32f0xx_hal::stm32::Peripherals = cx.device;

        let raw_rcc = device.RCC;

        let mut rcc = raw_rcc
            .configure()
            .sysclk(48.mhz())
            .pclk(48.mhz())
            .hclk(48.mhz())
            .freeze(&mut device.FLASH);

        let gpioa = device.GPIOA.split(&mut rcc);
        let (motor1_dir, fault, led, motor2_step, motor2_dir, sck, cs, mosi, scl, sda) =
            cortex_m::interrupt::free(|cs| {
                (
                    gpioa.pa0.into_push_pull_output(cs).downgrade(),
                    gpioa.pa1.into_pull_up_input(cs).downgrade(),
                    gpioa.pa2.into_push_pull_output(cs).downgrade(),
                    gpioa.pa3.into_push_pull_output(cs).downgrade(),
                    gpioa.pa4.into_push_pull_output(cs).downgrade(),
                    gpioa.pa5.into_alternate_af0(cs),
                    gpioa.pa6.into_push_pull_output(cs).downgrade(),
                    gpioa.pa7.into_alternate_af0(cs),
                    gpioa.pa9.into_alternate_af4(cs),
                    gpioa.pa10.into_alternate_af4(cs),
                )
            });

        let gpiob = device.GPIOB.split(&mut rcc);
        let ldac = cortex_m::interrupt::free(|cs| gpiob.pb1.into_push_pull_output(cs).downgrade());

        let gpiof = device.GPIOF.split(&mut rcc);
        let (mut enable, motor1_step) = cortex_m::interrupt::free(|cs| {
            (
                gpiof.pf1.into_push_pull_output(cs).downgrade(),
                gpiof.pf0.into_push_pull_output(cs).downgrade(),
            )
        });

        enable.set_low().unwrap();

        let mut dac = MCP4922::new(device.SPI1, mosi, sck, cs, ldac);
        dac.set_voltage(Channel::A, 0.4f32);
        dac.set_voltage(Channel::B, 0.4f32);

        let driver = Driver::new(
            device.TIM14,
            device.TIM16,
            device.TIM2,
            device.TIM3,
            dac,
            motor1_step,
            motor1_dir,
            motor2_step,
            motor2_dir,
            enable,
            fault,
            rcc.clocks,
        );

        let delay = Delay::new(core.SYST, &rcc);

        let i2c = device.I2C1;
        let i2c_slave = I2CSlave::new(i2c, I2C_ADDRESS, sda, scl);

        defmt::debug!("Init done.");
        init::LateResources {
            led,
            delay,
            driver,
            i2c_slave,
        }
    }

    #[idle(resources = [delay, led])]
    fn idle(cx: idle::Context) -> ! {
        let delay: &mut Delay = cx.resources.delay;
        let led: &mut Pin<Output<PushPull>> = cx.resources.led;
        loop {
            led.set_low().unwrap();
            delay.delay_ms(100u8);
            led.set_high().unwrap();
            delay.delay_ms(100u8);
        }
    }

    #[task(binds = TIM2, resources = [driver])]
    fn update_timer_handler(cx: update_timer_handler::Context) {
        cx.resources.driver.tim2_tick();
    }

    #[task(binds = TIM3, resources = [driver])]
    fn failsafe_timer_tick(cx: failsafe_timer_tick::Context) {
        cx.resources.driver.tim3_tick();
    }

    #[task(binds = TIM14, resources = [driver])]
    fn motor1_timer_handler(cx: motor1_timer_handler::Context) {
        cx.resources.driver.tim14_tick();
    }

    #[task(binds = TIM16, resources = [driver])]
    fn motor2_timer_handler(cx: motor2_timer_handler::Context) {
        cx.resources.driver.tim16_tick();
    }

    #[task(binds = I2C1, resources = [i2c_slave, driver])]
    fn i2c1_handler(cx: i2c1_handler::Context) {
        let i2c: &mut I2C = cx.resources.i2c_slave;
        let driver: &mut Driver = cx.resources.driver;
        i2c.interrupt();
        match i2c.get_state() {
            None => {}
            Some(State::DataReceived(register)) => {
                let data = i2c.get_received_data();
                let register = Register::try_from(register);
                if register.is_err() {
                    return;
                }
                let register = register.unwrap();

                match register {
                    Register::TargetSpeed => {
                        if data.len() != 8 {
                            defmt::error!("Not enough data for target speed: {:usize}", data.len());
                            return;
                        }
                        let speed1 = LittleEndian::read_f32(&data[..4]);
                        let speed2 = LittleEndian::read_f32(&data[4..8]);
                        driver.set_speeds(speed1, speed2);
                    }
                    Register::RampFrequency => {
                        if data.len() != 2 {
                            defmt::error!(
                                "Not enough data for ramp frequency: {:usize}",
                                data.len()
                            );
                            return;
                        }
                        let freq = LittleEndian::read_u16(&data[..2]);
                        driver.set_ramp_frequency(freq);
                    }
                    Register::RampStep => {
                        if data.len() != 4 {
                            defmt::error!("Not enough data for ramp step: {:usize}", data.len());
                            return;
                        }
                        let step = LittleEndian::read_f32(&data[..4]);
                        driver.set_ramp_step(step);
                    }
                    _ => {}
                }
            }
            Some(State::DataRequested(register)) => {
                let register = Register::try_from(register);
                if register.is_err() {
                    return;
                }
                let register = register.unwrap();
                match register {
                    Register::TargetSpeed => {
                        let mut data = [0u8; 8];
                        let speeds = driver.get_speeds();
                        LittleEndian::write_f32(&mut data[..4], speeds.0);
                        LittleEndian::write_f32(&mut data[4..], speeds.1);
                        i2c.set_transmit_buffer(&data);
                    }
                    Register::Odometry => {
                        let mut data = [0u8; 8];
                        let odometry = driver.get_raw_odometry();
                        LittleEndian::write_i32(&mut data[..4], odometry.0);
                        LittleEndian::write_i32(&mut data[4..], odometry.1);
                        i2c.set_transmit_buffer(&data);
                    }
                    Register::RampFrequency => {
                        let mut data = [0u8; 2];
                        LittleEndian::write_u16(&mut data, driver.get_ramp_frequency());
                        i2c.set_transmit_buffer(&data);
                    }
                    Register::RampStep => {
                        let mut data = [0u8; 4];
                        LittleEndian::write_f32(&mut data[..4], driver.get_ramp_step());
                        i2c.set_transmit_buffer(&data);
                    }
                    Register::FaultStatus => {
                        i2c.set_transmit_buffer(&[driver.get_fault_status()]);
                    }
                }
            }
        }
    }
};
