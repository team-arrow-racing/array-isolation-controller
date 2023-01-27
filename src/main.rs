#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

use stm32_hal2::{
    self,
    gpio::{Pin, PinMode, Port},
};

use stm32l4xx_hal::{
    can::Can,
    gpio::{Alternate, PushPull, PA11, PA12},
    pac::CAN1,
    prelude::*,
    watchdog::IndependentWatchdog,
};

use systick_monotonic::{
    fugit::{MillisDurationU32, MillisDurationU64},
    Systick,
};

use bxcan::{
    filter::Mask32,
    Interrupts, {Frame, StandardId},
};

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
        can: bxcan::Can<Can<CAN1, (PA12<Alternate<PushPull, 9>>, PA11<Alternate<PushPull, 9>>)>>,
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
        let mut gpioa = cx.device.GPIOA.split(&mut rcc.ahb2);
        let mut gpiob = cx.device.GPIOB.split(&mut rcc.ahb2);

        // configure system clock
        let clocks = rcc.cfgr.sysclk(80.MHz()).freeze(&mut flash.acr, &mut pwr);

        // configure monotonic time
        let mono = Systick::new(cx.core.SYST, clocks.sysclk().to_Hz());

        // configure can bus
        let can = {
            let rx =
                gpioa
                    .pa11
                    .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
            let tx =
                gpioa
                    .pa12
                    .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);

            let can = Can::new(&mut rcc.apb1r1, cx.device.CAN1, (tx, rx));

            bxcan::Can::builder(can)
        }
        .set_bit_timing(0x001c_0031)
        .set_loopback(true);

        let mut can = can.enable();

        can.modify_filters().enable_bank(0, Mask32::accept_all());

        can.enable_interrupts(
            Interrupts::TRANSMIT_MAILBOX_EMPTY | Interrupts::FIFO0_MESSAGE_PENDING,
        );
        nb::block!(can.enable_non_blocking()).unwrap();

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
            Shared { can, isolator },
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

    #[task(shared = [isolator], local = [watchdog])]
    fn run(mut cx: run::Context) {
        cx.shared.isolator.lock(|isolator| {
            isolator.run(monotonics::now());
        });

        cx.local.watchdog.feed();

        run::spawn_after(Duration::millis(10)).unwrap();
    }

    #[task(shared = [can])]
    fn heartbeat(mut cx: heartbeat::Context) {
        defmt::debug!("heartbeat!");

        cx.shared.can.lock(|can| {
            // Send a frame
            let mut test: [u8; 8] = [0; 8];

            test[0] = 0;
            test[1] = 1;
            test[2] = 2;
            test[3] = 3;
            test[4] = 4;
            test[5] = 5;
            test[6] = 6;
            test[7] = 7;
            let test_frame = Frame::new_data(StandardId::new(0x500).unwrap(), test);
            can.transmit(&test_frame).unwrap();

            // Wait for TX to finish
            while !can.is_transmitter_idle() {}

            defmt::trace!("sent message");
        });

        // repeat every second
        heartbeat::spawn_after(Duration::millis(1000)).unwrap();
    }

    #[task(shared = [can], binds = CAN1_RX0)]
    fn can_receive(mut cx: can_receive::Context) {
        defmt::debug!("received can message");

        cx.shared.can.lock(|can| {
            can.receive();
        });
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
