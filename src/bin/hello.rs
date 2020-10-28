#![no_main]
#![no_std]

use km3_rs as _; // global logger + panicking-behavior + memory layout
use km3_rs::i2c_slave::{Direction, I2CSlave, Status};
use km3_rs::timer::{Event, Timer};
use km3_rs::write_spi::{Channel, MCP4922};
use stm32f0xx_hal::delay::Delay;
use stm32f0xx_hal::gpio::gpioa::{PA5, PA7};
use stm32f0xx_hal::gpio::{Alternate, Output, Pin, PushPull, AF0};
use stm32f0xx_hal::{prelude::*, stm32};

type DAC = MCP4922<PA7<Alternate<AF0>>, PA5<Alternate<AF0>>>;
type Motor1Timer = Timer<stm32::TIM14>;
type Motor2Timer = Timer<stm32::TIM16>;

#[derive(PartialEq)]
pub enum TransferState {
    Idle,
    Addressed,
    RegisterSet,
    Receiving,
    Transmitting,
}

#[rtic::app(device = stm32f0xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        led: Pin<Output<PushPull>>,
        dac: DAC,
        delay: Delay,
        motor1_timer: Motor1Timer,
        motor2_timer: Motor2Timer,
        motor1_step: Pin<Output<PushPull>>,
        motor2_step: Pin<Output<PushPull>>,
        motor1_dir: Pin<Output<PushPull>>,
        motor2_dir: Pin<Output<PushPull>>,
        i2c_slave: I2CSlave,
        #[init(TransferState::Idle)]
        transfer_state: TransferState,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        defmt::info!("Hello, world!");
        let core: cortex_m::Peripherals = cx.core;
        let mut device: stm32f0xx_hal::stm32::Peripherals = cx.device;

        let raw_rcc = device.RCC;

        raw_rcc.apb1enr.modify(|_, w| w.i2c1en().enabled());
        raw_rcc.apb1rstr.modify(|_, w| w.i2c1rst().reset());

        // todo perform manipulation of RCC register values for custom peripherals
        let mut rcc = raw_rcc
            .configure()
            .sysclk(48.mhz())
            .pclk(48.mhz())
            .hclk(48.mhz())
            .freeze(&mut device.FLASH);

        let gpioa = device.GPIOA.split(&mut rcc);
        let (motor1_dir, _fault, led, motor2_step, motor2_dir, sck, cs, mosi, scl, sda) =
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
        let (enable, motor1_step) = cortex_m::interrupt::free(|cs| {
            (
                gpiof.pf1.into_push_pull_output(cs).downgrade(),
                gpiof.pf0.into_push_pull_output(cs).downgrade(),
            )
        });

        ldac.set_low().unwrap();

        // let dma = device.DMA1.split(&mut rcc);
        // let spi_channel = dma.3;

        let mut dac = MCP4922::new(device.SPI1, mosi, sck, cs);
        dac.set_voltage(Channel::A, 2.0f32);
        dac.set_voltage(Channel::B, 2.0f32);

        let mut motor1_timer = Timer::tim14(device.TIM14, 0.hz(), &mut rcc);
        motor1_timer.listen(Event::TimeOut);

        let mut motor2_timer = Timer::tim16(device.TIM16, 10.hz(), &mut rcc);
        motor2_timer.listen(Event::TimeOut);

        let delay = Delay::new(core.SYST, &rcc);

        let i2c = device.I2C1;
        let i2c_slave = I2CSlave::new(i2c);

        i2c_slave.dump();

        defmt::debug!("Init done.");
        init::LateResources {
            led,
            delay,
            dac,
            motor1_timer,
            motor2_timer,
            motor1_step,
            motor2_step,
            motor1_dir,
            motor2_dir,
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

    #[task(binds = TIM14, resources = [motor1_timer])]
    fn motor1_timer_handler(cx: motor1_timer_handler::Context) {
        let motor1_timer: &mut Motor1Timer = cx.resources.motor1_timer;
        if motor1_timer.wait().is_err() {
            defmt::error!("Failed to wait for the motor1 timer.");
        }
    }

    #[task(binds = TIM16, resources = [motor2_timer])]
    fn motor2_timer_handler(cx: motor2_timer_handler::Context) {
        let motor2_timer: &mut Motor2Timer = cx.resources.motor2_timer;
        if motor2_timer.wait().is_err() {
            defmt::error!("Failed to wait for the motor2 timer.");
        }
    }

    #[task(binds = I2C1, resources = [i2c_slave, transfer_state])]
    fn i2c1_handler(cx: i2c1_handler::Context) {
        let i2c: &mut I2CSlave = cx.resources.i2c_slave;
        let state: &mut TransferState = cx.resources.transfer_state;
        if *state == TransferState::Idle {
            i2c.enable_txie(false);
        }
        if i2c.is_status(Status::AddressMatch(Direction::Write), true) {
            *state = TransferState::Addressed;
            defmt::debug!("Addressed.");
        } else if i2c.is_status(Status::TxDataMustBeWritten, false) {
            // this may be true more times than actual data length, ignore then
            if *state == TransferState::Transmitting {
                // state is not changed
                i2c.write(0x66);
                defmt::debug!("Transmitting: 0x66");
            }
        } else if i2c.is_status(Status::AddressMatch(Direction::Read), true) {
            if *state == TransferState::RegisterSet {
                defmt::debug!("Transmitting");
                i2c.enable_txie(true);
                *state = TransferState::Transmitting;
            } else {
                defmt::debug!("Master wants to read.");
            }
        } else if i2c.is_status(Status::RxNotEmpty, false) {
            if *state == TransferState::Addressed {
                *state = TransferState::RegisterSet;
                let register = i2c.read();
                defmt::debug!("RegisterSet {:u8}", register);
            } else if *state == TransferState::RegisterSet {
                *state = TransferState::Receiving;
                defmt::debug!("Receiving");
            } else if *state == TransferState::Receiving {
                // do not change state, just read
                defmt::debug!("RECV: {:u8}", i2c.read());
            } else {
                defmt::debug!("master wrote: {:u8}", i2c.read());
            }
        } else if i2c.is_status(Status::Stop, true) {
            defmt::debug!("stopko");
            *state = TransferState::Idle;
        } else if i2c.is_status(Status::BusError, true) {
            defmt::debug!("buserr");
            *state = TransferState::Idle;
        } else if i2c.is_status(Status::Overrun, true) {
            defmt::debug!("overrun");
            *state = TransferState::Idle;
        } else if i2c.is_status(Status::ArbitrationLost, true) {
            defmt::debug!("arlo");
            *state = TransferState::Idle;
        } else if i2c.is_status(Status::NACKReceived, true) {
            defmt::debug!("nack");
            *state = TransferState::Idle;
        } else if i2c.is_status(Status::Timeout, true) {
            defmt::debug!("timeout");
            *state = TransferState::Idle;
        } else {
            defmt::error!("hello");
        }
    }
};
