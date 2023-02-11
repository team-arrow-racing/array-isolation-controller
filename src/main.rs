#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

use stm32l4xx_hal::{
    can::Can,
    gpio::{ErasedPin, Output, Alternate, PushPull, PA11, PA12},
    pac::CAN1,
    prelude::*,
    watchdog::IndependentWatchdog,
};

use dwt_systick_monotonic::{fugit, DwtSystick};

use bxcan::{filter::Mask32, Interrupts};

mod isolator;
use crate::isolator::Isolator;

use solar_car::{com, device};

const DEVICE: device::Device = device::Device::ArrayIsolationController;
const SYSCLK: u32 = 80_000_000;

#[rtic::app(device = stm32l4xx_hal::pac, dispatchers = [SPI1])]
mod app {
    use super::*;

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = DwtSystick<SYSCLK>;
    pub type Duration = fugit::TimerDuration<u64, SYSCLK>;
    pub type Instant = fugit::TimerInstant<u64, SYSCLK>;

    #[shared]
    struct Shared {
        can: bxcan::Can<
            Can<
                CAN1,
                (PA12<Alternate<PushPull, 9>>, PA11<Alternate<PushPull, 9>>),
            >,
        >,
        isolator: Isolator,
    }

    #[local]
    struct Local {
        watchdog: IndependentWatchdog,
        status_led: ErasedPin<Output<PushPull>>,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::trace!("task: init");

        // peripherals
        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();
        let mut pwr = cx.device.PWR.constrain(&mut rcc.apb1r1);
        let mut gpioa = cx.device.GPIOA.split(&mut rcc.ahb2);
        let mut gpiob = cx.device.GPIOB.split(&mut rcc.ahb2);
        let mut gpioc = cx.device.GPIOC.split(&mut rcc.ahb2);

        // configure system clock
        let clocks = rcc.cfgr.sysclk(80.MHz()).freeze(&mut flash.acr, &mut pwr);

        // configure monotonic time
        let mono = DwtSystick::new(
            &mut cx.core.DCB,
            cx.core.DWT,
            cx.core.SYST,
            clocks.sysclk().to_Hz(),
        );

        // configure status led
        let status_led = gpioc
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
                .set_loopback(true)
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
        let isolator = Isolator::new(isolator::Contactors {
            precharge: gpioa
                .pa5
                .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper)
                .erase(),
            negative: gpiob
                .pb1
                .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper)
                .erase(),
            positive: gpiob
                .pb0
                .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper)
                .erase(),
        });

        // configure watchdog
        let watchdog = {
            let mut wd = IndependentWatchdog::new(cx.device.IWDG);
            wd.stop_on_debug(&cx.device.DBGMCU, true);
            wd.start(fugit::MillisDurationU32::millis(100));

            wd
        };

        // schedule some state transitions whilst we don't have CAN bus working
        start::spawn_after(Duration::millis(1000)).unwrap();
        end::spawn_after(Duration::millis(5000)).unwrap();

        // start main loop
        run::spawn_after(Duration::millis(1)).unwrap();

        // start heartbeat
        heartbeat::spawn_after(Duration::millis(500)).unwrap();

        (
            Shared { can, isolator },
            Local { watchdog, status_led },
            init::Monotonics(mono),
        )
    }

    #[task(shared = [isolator])]
    fn start(mut cx: start::Context) {
        defmt::trace!("task: start");

        cx.shared.isolator.lock(|isolator| {
            isolator.start_precharge();
        });
    }

    #[task(shared = [isolator])]
    fn end(mut cx: end::Context) {
        defmt::trace!("task: end");

        cx.shared.isolator.lock(|isolator| {
            isolator.isolate();
        });
    }

    #[task(shared = [isolator], local = [watchdog])]
    fn run(mut cx: run::Context) {
        defmt::trace!("task: run");

        cx.shared.isolator.lock(|isolator| {
            isolator.run();
        });

        cx.local.watchdog.feed();

        run::spawn_after(Duration::millis(10)).unwrap();
    }

    #[task(local = [status_led], shared = [can])]
    fn heartbeat(mut cx: heartbeat::Context) {
        defmt::trace!("task: heartbeat");

        cx.local.status_led.toggle();

        if cx.local.status_led.is_set_low() {
            cx.shared.can.lock(|can| {
                can.transmit(&com::heartbeat::message(DEVICE)).unwrap();
            });
        }

        // repeat every second
        heartbeat::spawn_after(Duration::millis(500)).unwrap();
    }

    #[task(shared = [can], binds = CAN1_RX0)]
    fn can_receive(mut cx: can_receive::Context) {
        defmt::trace!("task: can receive");

        cx.shared.can.lock(|can| {
            can.receive().unwrap();
        });
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::trace!("task: idle");

        loop {
            cortex_m::asm::nop();
        }
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
