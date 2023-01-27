#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

use stm32_hal2::{
    self,
    gpio::{Pin, PinMode, Port},
};

use stm32l4xx_hal::{prelude::*, watchdog::IndependentWatchdog};

use systick_monotonic::{
    fugit::{MillisDurationU32, MillisDurationU64},
    Systick,
};

type Duration = MillisDurationU64;

mod isolator;
use crate::isolator::Isolator;

#[rtic::app(device = stm32l4xx_hal::pac, dispatchers = [SPI1, SPI2, SPI3])]
mod app {
    use super::*;

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;

    #[shared]
    struct Shared {
        isolator: Isolator,
    }

    #[local]
    struct Local {
        watchdog: IndependentWatchdog,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("init");

        // peripherals
        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();
        let mut pwr = cx.device.PWR.constrain(&mut rcc.apb1r1);

        // configure system clock
        let clocks = rcc.cfgr.sysclk(80.MHz()).freeze(&mut flash.acr, &mut pwr);

        // configure monotonic time
        let mono = Systick::new(cx.core.SYST, clocks.sysclk().to_Hz());

        // configure contactors
        let isolator = Isolator::new(isolator::Contactors {
            precharge: Pin::new(Port::A, 5, PinMode::Output),
            negative: Pin::new(Port::B, 1, PinMode::Output),
            positive: Pin::new(Port::B, 0, PinMode::Output),
        });

        // configure watchdog
        let mut watchdog = IndependentWatchdog::new(cx.device.IWDG);
        watchdog.stop_on_debug(&cx.device.DBGMCU, true);
        watchdog.start(MillisDurationU32::millis(100));

        // schedule some state transitions whilst we don't have CAN bus working
        start::spawn_after(Duration::millis(1000)).unwrap();
        end::spawn_after(Duration::millis(5000)).unwrap();

        // start main loop
        run::spawn_after(Duration::millis(1)).unwrap();

        // start heartbeat
        heartbeat::spawn_after(Duration::millis(1)).unwrap();

        (
            Shared { isolator },
            Local { watchdog },
            init::Monotonics(mono),
        )
    }

    #[task(shared = [isolator])]
    fn start(mut cx: start::Context) {
        defmt::info!("starting precharge.");

        cx.shared.isolator.lock(|isolator| {
            isolator.start_precharge(monotonics::now());
        });
    }

    #[task(shared = [isolator])]
    fn end(mut cx: end::Context) {
        defmt::info!("isolating.");

        cx.shared.isolator.lock(|isolator| {
            isolator.isolate();
        });
    }

    #[task(shared = [isolator], local = [watchdog], priority = 3)]
    fn run(mut cx: run::Context) {
        cx.shared.isolator.lock(|isolator| {
            isolator.run(monotonics::now());
        });

        cx.local.watchdog.feed();

        run::spawn_after(Duration::millis(10)).unwrap();
    }

    #[task(priority = 2)]
    fn heartbeat(_cx: heartbeat::Context) {
        defmt::debug!("heartbeat!");
        // repeat every second
        heartbeat::spawn_after(Duration::millis(1000)).unwrap();
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        // if the idle task is entered (there is no scheduled tasks) we will
        // idle until the watchdog timer resets the device.
        loop {
            cortex_m::asm::nop(); // prevent loop from being optomised away.
        }
    }

    // show millisecond timestamp in debug log
    defmt::timestamp!("{=u64}ms", { monotonics::now().ticks() });
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
