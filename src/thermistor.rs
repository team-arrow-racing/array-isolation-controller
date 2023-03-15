
use cortex_m::prelude::_embedded_hal_adc_OneShot;
use stm32l4xx_hal::{
    adc::{ADC, Resolution},
    gpio::Analog,
    gpio::{Pin, L8}
};

pub struct Thermistor {
    pin: Pin<Analog, L8, 'A', 0>,
    r_25: f32,
    r_85: f32,
}

impl Thermistor {
    pub fn new(pin: Pin<Analog, L8, 'A', 0>, r_25: f32, r_85: f32) -> Self {
        Self {
            pin,
            r_25,
            r_85
        }
    }

    pub fn read(&mut self, adc: &mut ADC) -> f32 {
        adc.set_resolution(Resolution::Bits12);
        let value = adc.read(&mut self.pin).unwrap() as f32 / adc.get_max_value() as f32;
        
        let slope: f32 = (self.r_85 - self.r_25) / (85 - 25) as f32;

        // y = mx + c
        let temperature = (value * -slope as f32) - 25.0;

        temperature
    }
}