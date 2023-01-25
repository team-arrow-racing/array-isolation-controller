#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

use stm32_hal2::{
    self,
    clocks::Clocks,
    gpio::{Pin, PinMode, Port},
};

use systick_monotonic::{fugit::MillisDurationU64, Systick};

type Duration = MillisDurationU64;

mod isolator;
use crate::isolator::Isolator;

#[rtic::app(device = stm32l4xx_hal::pac, dispatchers = [SPI1])]
mod app {
    use super::*;

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;

    #[shared]
    struct Shared {
        isolator: Isolator,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("init");

        let clock_cfg = Clocks::default();
        clock_cfg.setup().unwrap();

        let mono = Systick::new(cx.core.SYST, clock_cfg.systick());

        let isolator = Isolator::new(
            Pin::new(Port::A, 5, PinMode::Output),
            Pin::new(Port::B, 1, PinMode::Output),
            Pin::new(Port::B, 0, PinMode::Output),
        );

        run::spawn().unwrap();
        start::spawn_after(Duration::millis(1000)).unwrap();
        end::spawn_after(Duration::millis(5000)).unwrap();

        (Shared { isolator }, Local {}, init::Monotonics(mono))
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }

    #[task(shared = [isolator])]
    fn start(mut cx: start::Context) {
        defmt::info!("starting precharge.");

        cx.shared.isolator.lock(|isolator| {
            isolator.start_precharge();
        });
    }

    #[task(shared = [isolator])]
    fn end(mut cx: end::Context) {
        defmt::info!("isolating.");

        cx.shared.isolator.lock(|isolator| {
            isolator.isolate();
        });
    }

    #[task(shared = [isolator])]
    fn run(mut cx: run::Context) {
        cx.shared.isolator.lock(|isolator| {
            isolator.run(monotonics::now());
        });

        run::spawn_after(Duration::millis(50)).unwrap();
    }

    defmt::timestamp!("{=u64}ms", {
        monotonics::now().ticks()
    });
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
