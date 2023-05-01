use cortex_m::prelude::_embedded_hal_adc_OneShot;
use stm32l4xx_hal::{
    adc::ADC,
    gpio::Analog,
    gpio::{Pin, L8},
};

/// Thermistor instance.
pub struct Thermistor {
    pin: Pin<Analog, L8, 'A', 0>,
    slope: f32,
}

impl Thermistor {
    /// Create a new thermistor instance.
    ///
    /// See the datasheet for r25 and r85 values.
    /// For example: https://www.vishay.com/docs/29094/ntcalug02a.pdf
    pub fn new(pin: Pin<Analog, L8, 'A', 0>, r_25: f32, r_85: f32) -> Self {
        // calculate the voltage-to-temperature slope.
        let slope: f32 = (r_85 - r_25) / (85 - 25) as f32;

        Self { pin, slope }
    }

    /// Read the thermistor temperature.
    pub fn read(&mut self, adc: &mut ADC) -> Result<f32, &'static str> {
        match adc.read(&mut self.pin) {
            Ok(value) => {
                // convert to scale
                let value = value as f32 / adc.get_max_value() as f32;

                // y = mx + c
                // slope is negative as to match the negative temperature
                // coefficient.
                Ok((value * -self.slope as f32) - 25.0)
            }
            Err(_) => Err("failed to get ADC reading."),
        }
    }
}
