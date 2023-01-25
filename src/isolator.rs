use stm32_hal2::gpio::Pin;
use systick_monotonic::fugit::{MillisDurationU64, TimerInstantU64};

type Instant = TimerInstantU64<1000>;
type Duration = MillisDurationU64;

#[derive(Clone, Copy, Debug)]
pub enum IsolatorState {
    Isolated,
    Precharging { state: PrechargeState },
    Engaged,
}

#[derive(Clone, Copy, Debug)]
pub enum PrechargeState {
    Negative { start: Instant },
    Charging { start: Instant },
}

pub struct Contactors {
    pub precharge: Pin,
    pub negative: Pin,
    pub positive: Pin,
}

pub struct Isolator {
    state: IsolatorState,
    contactors: Contactors,
}

impl Isolator {
    pub fn new(contactors: Contactors) -> Self {
        Isolator {
            state: IsolatorState::Isolated,
            contactors,
        }
    }

    pub fn status(&self) -> IsolatorState {
        self.state
    }

    pub fn start_precharge(&mut self) {
        self.state = match self.state {
            IsolatorState::Isolated => {
                defmt::trace!("contactors common negative.");
                IsolatorState::Precharging {
                    state: PrechargeState::Negative {
                        start: Instant::from_ticks(100),
                    },
                }
            },
            _ => panic!("invalid state transition: isolator was not in the isolated state when precharge was requested"),
        }
    }

    pub fn isolate(&mut self) {
        self.state = IsolatorState::Isolated;
        defmt::trace!("contactors isolated.");
    }

    pub fn run(&mut self, time: Instant) {
        match self.state {
            IsolatorState::Isolated => {
                self.contactors.precharge.set_low();
                self.contactors.negative.set_low();
                self.contactors.positive.set_low();
            }
            IsolatorState::Precharging { state } => match state {
                PrechargeState::Negative { .. } => {
                    self.contactors.precharge.set_low();
                    self.contactors.negative.set_high();
                    self.contactors.positive.set_low();
                }
                PrechargeState::Charging { .. } => {
                    self.contactors.precharge.set_high();
                    self.contactors.negative.set_high();
                    self.contactors.positive.set_low();
                }
            },
            IsolatorState::Engaged => {
                self.contactors.precharge.set_low();
                self.contactors.negative.set_high();
                self.contactors.positive.set_high();
            }
        }

        match self.state {
            IsolatorState::Isolated => {}
            IsolatorState::Precharging { state } => match state {
                PrechargeState::Negative { start } => {
                    let duration = Duration::from_ticks(1000);
                    let elapsed = time.checked_duration_since(start).unwrap();

                    if elapsed > duration {
                        self.state = IsolatorState::Precharging {
                            state: PrechargeState::Charging { start: time },
                        };
                        defmt::trace!("contactors charging load.");
                    }
                }
                PrechargeState::Charging { start } => {
                    let duration = Duration::from_ticks(1000);
                    let elapsed = time.checked_duration_since(start).unwrap();

                    if elapsed > duration {
                        self.state = IsolatorState::Engaged;
                        defmt::trace!("contactors engaged.");
                    }
                }
            },
            IsolatorState::Engaged => {}
        }
    }
}
