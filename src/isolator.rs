//! Isolation controller state machine.

use cortex_m::prelude::_embedded_hal_PwmPin;
use stm32l4xx_hal::{
    device::TIM2,
    gpio::{ErasedPin, Output, PushPull},
    pwm::{Pwm, C3, C4},
};

use crate::app::monotonics::MonoTimer as monotonic;

type Duration = crate::app::Duration;
type Instant = crate::app::Instant;

/// Isolator state.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum IsolatorState {
    Isolated,
    Precharging { state: PrechargeState },
    Engaged,
}

/// Precharge state.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PrechargeState {
    Negative { start: Instant },
    Charging { start: Instant },
}

/// Contactor group.
pub struct Contactors {
    pub precharge: ErasedPin<Output<PushPull>>,
    pub negative: Pwm<TIM2, C4>,
    pub positive: Pwm<TIM2, C3>,
}

/// Isolator instance.
pub struct Isolator {
    state: IsolatorState,
    contactors: Contactors,
    comms_watchdog_last_fed: Option<Instant>,
}

impl Isolator {
    /// Create a new isolator instance.
    pub fn new(contactors: Contactors) -> Self {
        Isolator {
            state: IsolatorState::Isolated,
            contactors,
            comms_watchdog_last_fed: None,
        }
    }

    /// Retreive the current state.
    pub fn status(&self) -> IsolatorState {
        self.state
    }

    /// Start the precharge process.
    ///
    /// This will panic if not in the isolated state.
    pub fn engage(&mut self) {
        // update state if necessary
        match self.state {
            IsolatorState::Isolated => {
                defmt::trace!("contactors common negative.");
                self.state = IsolatorState::Precharging {
                    state: PrechargeState::Negative {
                        start: monotonic::now(),
                    },
                };
                self.update_outputs();
            }
            _ => {} // precharge has already started
        }

        // feed watchdog
        self.comms_watchdog_last_fed = Some(monotonic::now());
    }

    /// Move the isolator into the isolated state instantly.
    pub fn isolate(&mut self) {
        self.state = IsolatorState::Isolated;
        self.comms_watchdog_last_fed = None;
        self.update_outputs();
        defmt::trace!("contactors isolated.");
    }

    /// Update the output states given the current state.
    fn update_outputs(&mut self) {
        match self.state {
            IsolatorState::Isolated => {
                self.contactors.precharge.set_low();
                self.contactors.negative.disable();
                self.contactors.positive.disable();
            }
            IsolatorState::Precharging { state } => match state {
                PrechargeState::Negative { .. } => {
                    self.contactors.precharge.set_low();

                    self.contactors.negative.enable();
                    self.contactors
                        .negative
                        .set_duty(self.contactors.negative.get_max_duty());

                    self.contactors.positive.disable();
                }
                PrechargeState::Charging { .. } => {
                    self.contactors.precharge.set_high();

                    self.contactors.negative.enable();
                    self.contactors
                        .negative
                        .set_duty(self.contactors.negative.get_max_duty());

                    self.contactors.positive.disable();
                }
            },
            IsolatorState::Engaged => {
                self.contactors.precharge.set_low();

                self.contactors.negative.enable();
                self.contactors
                    .negative
                    .set_duty(self.contactors.negative.get_max_duty());

                self.contactors.positive.enable();
                self.contactors
                    .positive
                    .set_duty(self.contactors.positive.get_max_duty());
            }
        }
    }

    /// Run the state machine, making necessary state transitions and then
    // updating the output states.
    pub fn run(&mut self) {
        let now = monotonic::now();

        // check isolator validity
        if self.state != IsolatorState::Isolated {
            if !self.comms_watchdog_valid() {
                self.isolate();
                return;
            }
        }

        match self.state {
            IsolatorState::Precharging { state } => match state {
                PrechargeState::Negative { start } => {
                    // only enough time for the negative contactor to swing over
                    let duration = Duration::millis(250);
                    let elapsed = now.checked_duration_since(start).unwrap();

                    if elapsed > duration {
                        self.state = IsolatorState::Precharging {
                            state: PrechargeState::Charging { start: now },
                        };
                        defmt::trace!("contactors charging load.");
                    }
                }
                PrechargeState::Charging { start } => {
                    // charging time
                    let duration = Duration::millis(2500);
                    let elapsed = now.checked_duration_since(start).unwrap();

                    if elapsed > duration {
                        self.state = IsolatorState::Engaged;
                        defmt::trace!("contactors engaged.");
                    }
                }
            },
            _ => {} // do nothing for other states
        }

        self.update_outputs();
    }

    fn comms_watchdog_valid(&self) -> bool {
        match self.comms_watchdog_last_fed {
            None => false,
            Some(then) => {
                let now = monotonic::now();
                match now.checked_duration_since(then) {
                    Some(duration) => duration < Duration::millis(200),
                    None => false
                }
            }
        }
    }
}
