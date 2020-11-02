#![no_main]
#![no_std]

use byteorder::{ByteOrder, LittleEndian};
use km3_rs as _; // global logger + panicking-behavior + memory layout
use km3_rs::driver::Driver;
use km3_rs::i2c_slave::{I2CSlave, State};
use km3_rs::timer::{Event, Timer};
use km3_rs::write_spi::{Channel, MCP4922};
use stm32f0xx_hal::delay::Delay;
use stm32f0xx_hal::gpio::gpioa::{PA5, PA7};
use stm32f0xx_hal::gpio::{Alternate, Output, Pin, PushPull, AF0};
use stm32f0xx_hal::{prelude::*, stm32};

type DAC = MCP4922<PA7<Alternate<AF0>>, PA5<Alternate<AF0>>>;
const FAILSAFE_COUNTER_TOP: u8 = 4;

#[rtic::app(device = stm32f0xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        led: Pin<Output<PushPull>>,
        dac: DAC,
        delay: Delay,
        motor1: Driver<Timer<stm32::TIM14>>,
        motor2: Driver<Timer<stm32::TIM16>>,
        update_timer: Timer<stm32::TIM2>,
        failsafe_timer: Timer<stm32::TIM3>,
        #[init(0)]
        failsafe_counter: u8,
        i2c_slave: I2CSlave,
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
        let (mut motor1_dir, _fault, led, motor2_step, mut motor2_dir, sck, cs, mosi, scl, sda) =
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
        let mut ldac =
            cortex_m::interrupt::free(|cs| gpiob.pb1.into_push_pull_output(cs).downgrade());

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

        let mut motor1_timer = Timer::tim14(device.TIM14, 0.hz(), &mut rcc);
        motor1_timer.listen(Event::TimeOut);
        let motor1 = Driver::new(motor1_timer, motor1_step, motor1_dir);

        let mut motor2_timer = Timer::tim16(device.TIM16, 0.hz(), &mut rcc);
        motor2_timer.listen(Event::TimeOut);
        let motor2 = Driver::new(motor2_timer, motor2_step, motor2_dir);

        let mut update_timer = Timer::tim2(device.TIM2, 50.hz(), &mut rcc);
        update_timer.listen(Event::TimeOut);

        let mut failsafe_timer = Timer::tim3(device.TIM3, 2.hz(), &mut rcc);
        failsafe_timer.listen(Event::TimeOut);

        let delay = Delay::new(core.SYST, &rcc);

        let i2c = device.I2C1;
        let i2c_slave = I2CSlave::new(i2c);

        defmt::debug!("Init done.");
        init::LateResources {
            led,
            delay,
            dac,
            motor1,
            motor2,
            update_timer,
            failsafe_timer,
            i2c_slave,
        }
    }

    #[idle(resources = [delay, led, dac])]
    fn idle(cx: idle::Context) -> ! {
        let delay: &mut Delay = cx.resources.delay;
        let led: &mut Pin<Output<PushPull>> = cx.resources.led;
        loop {
            led.set_low().unwrap();
            delay.delay_ms(100u8);
            led.set_high().unwrap();
            delay.delay_ms(100u8);
            cx.resources.dac.update();
        }
    }

    #[task(binds = TIM2, resources = [motor1, motor2, update_timer])]
    fn update_timer_handler(cx: update_timer_handler::Context) {
        cx.resources.update_timer.wait();
        cx.resources.motor1.update();
        cx.resources.motor2.update();
    }

    #[task(binds = TIM3, resources = [motor1, motor2, failsafe_timer, failsafe_counter])]
    fn failsafe_timer_tick(cx: failsafe_timer_tick::Context) {
        let counter: &mut u8 = cx.resources.failsafe_counter;
        if cx.resources.failsafe_timer.wait().is_err() {
            defmt::error!("Failed to wait for the failsafe timer.");
        }

        if *counter == 0 {
            cx.resources.motor1.set_speed(0.0);
            cx.resources.motor2.set_speed(0.0);
            defmt::warn!("Failsafe timer engaged.");
        } else {
            *counter -= 1;
        }
    }

    #[task(binds = TIM14, resources = [motor1])]
    fn motor1_timer_handler(cx: motor1_timer_handler::Context) {
        if cx.resources.motor1.tick().is_err() {
            defmt::error!("Failed to wait for the motor1 timer.");
        }
    }

    #[task(binds = TIM16, resources = [motor2])]
    fn motor2_timer_handler(cx: motor2_timer_handler::Context) {
        if cx.resources.motor2.tick().is_err() {
            defmt::error!("Failed to wait for the motor2 timer.");
        }
    }

    #[task(binds = I2C1, resources = [i2c_slave, motor1, motor2, failsafe_counter])]
    fn i2c1_handler(cx: i2c1_handler::Context) {
        let i2c: &mut I2CSlave = cx.resources.i2c_slave;
        let motor1: &mut Driver<Timer<stm32::TIM14>> = cx.resources.motor1;
        let motor2: &mut Driver<Timer<stm32::TIM16>> = cx.resources.motor2;
        i2c.interrupt();
        match i2c.get_state() {
            None => {}
            Some(State::DataReceived(register)) => {
                let data = i2c.get_received_data();
                *cx.resources.failsafe_counter = FAILSAFE_COUNTER_TOP;
                match register {
                    0x10 => {
                        if data.len() != 8 {
                            defmt::error!(
                                "Not enough data for register: {:u8}, {:usize}",
                                register,
                                data.len()
                            );
                            return;
                        }
                        let speed1 = LittleEndian::read_f32(&data[..4]);
                        let speed2 = LittleEndian::read_f32(&data[4..8]);
                        motor1.set_speed(speed1);
                        motor2.set_speed(speed2);
                    }
                    _ => {}
                }
                defmt::error!("Finished Receiving {:u8}.", register);
            }
            Some(State::DataRequested(register)) => {
                if register < 0x10 {
                    i2c.set_transmit_buffer(&[register]);
                } else {
                    i2c.set_transmit_buffer(&[register, register + 1]);
                }
                defmt::error!("Data requested {:u8}.", register);
            }
        }
    }
};
