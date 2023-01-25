use stm32_hal2::gpio::Pin;
use systick_monotonic::fugit::{Instant, MillisDurationU64};

#[derive(Clone, Copy, Debug)]
pub enum IsolatorState {
    Isolated,
    Precharging { state: PrechargeState },
    Engaged,
}

#[derive(Clone, Copy, Debug)]
pub enum PrechargeState {
    Negative { start: Instant<u64, 1, 1000> },
    Charging { start: Instant<u64, 1, 1000> },
}

pub struct Isolator {
    state: IsolatorState,
    precharge: Pin,
    negative: Pin,
    positive: Pin,
}

impl Isolator {
    pub fn new(precharge: Pin, negative: Pin, positive: Pin) -> Self {
        Isolator {
            state: IsolatorState::Isolated,
            precharge: precharge,
            negative: negative,
            positive: positive,
        }
    }

    pub fn status(&self) -> IsolatorState {
        self.state
    }

    pub fn start_precharge(&mut self) {
        self.state = match self.state {
            IsolatorState::Isolated => IsolatorState::Precharging {
                state: PrechargeState::Negative {
                    start: Instant::<u64, 1, 1000>::from_ticks(100),
                },
            },
            _ => panic!("Invalid state transition."),
        }
    }

    pub fn isolate(&mut self) {
        self.state = IsolatorState::Isolated;
    }

    pub fn run(&mut self, time: Instant<u64, 1, 1000>) {
        match self.state {
            IsolatorState::Isolated => {
                self.precharge.set_low();
                self.negative.set_low();
                self.positive.set_low();
                defmt::trace!("contactors isolated.");
            }
            IsolatorState::Precharging { state } => match state {
                PrechargeState::Negative { .. } => {
                    self.precharge.set_low();
                    self.negative.set_high();
                    self.positive.set_low();
                    defmt::trace!("contactors common negative.");
                }
                PrechargeState::Charging { .. } => {
                    self.precharge.set_high();
                    self.negative.set_high();
                    self.positive.set_low();
                    defmt::trace!("contactors charging load.");
                }
            },
            IsolatorState::Engaged => {
                self.precharge.set_low();
                self.negative.set_high();
                self.positive.set_high();
                defmt::trace!("contactors engaged.");
            }
        }

        match self.state {
            IsolatorState::Isolated => {}
            IsolatorState::Precharging { state } => match state {
                PrechargeState::Negative { start } => {
                    let duration = MillisDurationU64::from_ticks(1000);
                    let elapsed = time.checked_duration_since(start).unwrap();

                    if elapsed > duration {
                        self.state = IsolatorState::Precharging {
                            state: PrechargeState::Charging { start: time },
                        };
                    }
                }
                PrechargeState::Charging { start } => {
                    let duration = MillisDurationU64::from_ticks(1000);
                    let elapsed = time.checked_duration_since(start).unwrap();

                    if elapsed > duration {
                        self.state = IsolatorState::Engaged;
                    }
                }
            },
            IsolatorState::Engaged => {}
        }
    }
}
