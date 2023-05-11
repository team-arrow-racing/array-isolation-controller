//! Array Isolation Controller
//!
//! Gralvanically isolates the solar array from the rest of the high-voltage
//! system.
//!
//! # Task priority assignment
//!
//! It would be more idomatic to have these assigned in a enum or some constants
//! but RTIC doesn't yet support variables (static or otherwise) in task
//! definitions.
//!
//! | Priority | Use |
//! | --- | --- |
//! | 0 | `idle` task and background tasks. |
//! | 1 (default) | General and asychronous tasks. |
//! | 2 | Synchronous comms tasks. |
//! | 3 | System critical tasks. |

#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

use bxcan::{filter::Mask32, Id, Interrupts};
use cortex_m::delay::Delay;
use dwt_systick_monotonic::{fugit, DwtSystick};
use solar_car::{
    com,
    com::array::{PGN_FEED_WATCHDOG, PGN_ISOLATE, PGN_START_PRECHARGE},
    device, j1939,
    j1939::pgn::Pgn,
};
use stm32l4xx_hal::{
    adc::ADC,
    can::Can,
    gpio::{Alternate, ErasedPin, Output, PushPull, PA11, PA12},
    pac::CAN1,
    prelude::*,
    watchdog::IndependentWatchdog,
};

mod isolator;
use crate::isolator::Isolator;

mod thermistor;
use crate::thermistor::Thermistor;

const DEVICE: device::Device = device::Device::ArrayIsolationController;
const SYSCLK: u32 = 80_000_000;

#[rtic::app(device = stm32l4xx_hal::pac, dispatchers = [SPI1, SPI2, SPI3, QUADSPI])]
mod app {
    use super::*;

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = DwtSystick<SYSCLK>;
    pub type Duration = fugit::TimerDuration<u64, SYSCLK>;
    pub type Instant = fugit::TimerInstant<u64, SYSCLK>;

    #[shared]
    struct Shared {
        adc: ADC,
        can: bxcan::Can<
            Can<
                CAN1,
                (PA12<Alternate<PushPull, 9>>, PA11<Alternate<PushPull, 9>>),
            >,
        >,
        isolator: Isolator,
        isolator_wd_fed: Option<Instant>,
    }

    #[local]
    struct Local {
        watchdog: IndependentWatchdog,
        led_status: ErasedPin<Output<PushPull>>,
        thermistor: Thermistor,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::trace!("task: init");

        // peripherals
        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();
        let mut pwr = cx.device.PWR.constrain(&mut rcc.apb1r1);
        let mut gpioa = cx.device.GPIOA.split(&mut rcc.ahb2);
        let mut _gpiob = cx.device.GPIOB.split(&mut rcc.ahb2);
        let mut gpioc = cx.device.GPIOC.split(&mut rcc.ahb2);

        // configure system clock
        let clocks = rcc.cfgr.sysclk(80.MHz()).freeze(&mut flash.acr, &mut pwr);

        // configure adc
        let adc = {
            let mut delay = Delay::new(
                unsafe { stm32l4xx_hal::pac::CorePeripherals::steal().SYST },
                clocks.sysclk().to_Hz(),
            );

            ADC::new(
                cx.device.ADC1,
                cx.device.ADC_COMMON,
                &mut rcc.ahb2,
                &mut rcc.ccipr,
                &mut delay,
            )
        };

        // configure thermistor
        let thermistor = {
            let pin = gpioa.pa0.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
            Thermistor::new(pin, 10_000.0, 3435.0)
        };

        // configure monotonic time
        let mono = DwtSystick::new(
            &mut cx.core.DCB,
            cx.core.DWT,
            cx.core.SYST,
            clocks.sysclk().to_Hz(),
        );

        // configure status led
        let led_status = gpioc
            .pc13
            .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper)
            .erase();

        // configure can bus
        let can = {
            let rx = gpioa.pa11.into_alternate(
                &mut gpioa.moder,
                &mut gpioa.otyper,
                &mut gpioa.afrh,
            );
            let tx = gpioa.pa12.into_alternate(
                &mut gpioa.moder,
                &mut gpioa.otyper,
                &mut gpioa.afrh,
            );

            let peripheral =
                Can::new(&mut rcc.apb1r1, cx.device.CAN1, (tx, rx));

            let mut can = bxcan::Can::builder(peripheral)
                .set_bit_timing(0x001c_0009) // 500kbit/s
                .enable();

            can.modify_filters().enable_bank(0, Mask32::accept_all());

            can.enable_interrupts(
                Interrupts::TRANSMIT_MAILBOX_EMPTY
                    | Interrupts::FIFO0_MESSAGE_PENDING,
            );

            nb::block!(can.enable_non_blocking()).unwrap();

            can
        };

        // configure contactors
        let isolator = {
            let negative = gpioa.pa3.into_alternate_push_pull(
                &mut gpioa.moder,
                &mut gpioa.otyper,
                &mut gpioa.afrl,
            );

            let postive = gpioa.pa2.into_alternate_push_pull(
                &mut gpioa.moder,
                &mut gpioa.otyper,
                &mut gpioa.afrl,
            );

            let pwm = cx.device.TIM2.pwm(
                (postive, negative),
                40000.Hz(),
                clocks,
                &mut rcc.apb1r1,
            );

            Isolator::new(isolator::Contactors {
                precharge: gpioa
                    .pa5
                    .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper)
                    .erase(),
                negative: pwm.1,
                positive: pwm.0,
            })
        };

        // configure watchdog
        let mut watchdog = IndependentWatchdog::new(cx.device.IWDG);
        watchdog.stop_on_debug(&cx.device.DBGMCU, true);
        watchdog.start(100.millis());

        // start tasks
        run::spawn().unwrap();
        isolator_watchdog::spawn().unwrap();
        heartbeat::spawn().unwrap();

        (
            Shared {
                adc,
                can,
                isolator,
                isolator_wd_fed: None,
            },
            Local {
                watchdog,
                led_status,
                thermistor,
            },
            init::Monotonics(mono),
        )
    }

    #[task(shared = [isolator], local = [watchdog])]
    fn run(mut cx: run::Context) {
        // defmt::trace!("task: run");

        cx.shared.isolator.lock(|isolator| {
            isolator.run();
        });

        cx.local.watchdog.feed();

        run::spawn_after(10.millis().into()).unwrap();
    }

    #[task(local = [led_status], shared = [can])]
    fn heartbeat(mut cx: heartbeat::Context) {
        defmt::trace!("task: heartbeat");

        cx.local.led_status.toggle();

        if cx.local.led_status.is_set_low() {
            cx.shared.can.lock(|can| {
                let _ = can.transmit(&com::heartbeat::message(DEVICE));
            });
        }

        // repeat every second
        heartbeat::spawn_after(500.millis().into()).unwrap();
    }

    #[task(priority = 3, local = [thermistor], shared = [adc, isolator, isolator_wd_fed])]
    fn isolator_watchdog(mut cx: isolator_watchdog::Context) {
        defmt::trace!("task: isolator watchdog");

        let time = monotonics::now();

        cx.shared.isolator.lock(|isolator| {
            // check temporal watchdog
            cx.shared.isolator_wd_fed.lock(|last_fed| {
                if let Some(fed) = last_fed {
                    if time.checked_duration_since(*fed).unwrap()
                        > Duration::millis(500)
                    {
                        isolator.isolate();
                    }
                } else {
                    isolator.isolate();
                }
            });

            // check thermal watchdog
            cx.shared.adc.lock(|adc| {
                let temperature = cx.local.thermistor.read(adc);

                match temperature {
                    Ok(t) => {
                        // temperature over 50 degrees
                        if t > 50.0 {
                            isolator.isolate();
                        }
                    }
                    Err(_) => isolator.isolate(),
                }
            });
        });

        isolator_watchdog::spawn_after(100.millis().into()).unwrap();
    }

    /// RX 0 interrupt pending handler.
    #[task(priority = 2, binds = CAN1_RX0)]
    fn can_rx0_pending(_: can_rx0_pending::Context) {
        defmt::trace!("task: can rx0 pending");
        can_frame_handler::spawn().unwrap();
    }

    /// RX 1 interrupt pending handler.
    #[task(priority = 2, binds = CAN1_RX1)]
    fn can_rx1_pending(_: can_rx1_pending::Context) {
        defmt::trace!("task: can rx1 pending");
        can_frame_handler::spawn().unwrap();
    }

    /// Process can frames.
    #[task(priority = 1, shared = [can, isolator, isolator_wd_fed])]
    fn can_frame_handler(mut cx: can_frame_handler::Context) {
        defmt::trace!("task: can receive");

        cx.shared.can.lock(|can| {
            // receive messages until there is none left
            loop {
                match can.receive() {
                    Ok(frame) => {
                        match frame.id() {
                            Id::Standard(id) => {
                                // EV Driver controls switch position
                                if id.as_raw() == 0x505 {
                                    // start precharge
                                    cx.shared
                                        .isolator
                                        .lock(|iso| iso.start_precharge())
                                }
                            } // not for us
                            Id::Extended(id) => {
                                // convert to a J1939 id
                                let id: j1939::ExtendedId = id.into();

                                // is this message for us?
                                match id.pgn {
                                    Pgn::Destination(pgn) => match pgn {
                                        PGN_START_PRECHARGE => {
                                            cx.shared.isolator_wd_fed.lock(
                                                |time| {
                                                    *time =
                                                        Some(monotonics::now());
                                                },
                                            );

                                            cx.shared.isolator.lock(|iso| {
                                                iso.start_precharge()
                                            })
                                        }
                                        PGN_ISOLATE => cx
                                            .shared
                                            .isolator
                                            .lock(|iso| iso.isolate()),
                                        PGN_FEED_WATCHDOG => {
                                            cx.shared.isolator_wd_fed.lock(
                                                |time| {
                                                    *time =
                                                        Some(monotonics::now());
                                                },
                                            );
                                        }
                                        _ => {}
                                    },
                                    _ => {} // ignore broadcast messages
                                }
                            }
                        }
                    }
                    Err(nb::Error::Other(_)) => {} // go to next frame
                    Err(nb::Error::WouldBlock) => break, // done
                }
            }
        })
    }
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

// show millisecond timestamp in debug log
defmt::timestamp!("time={=u64}ms", {
    app::monotonics::MonoTimer::now()
        .duration_since_epoch()
        .to_millis()
});
