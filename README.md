# Array Isolation Controller

Responsible for the safe isolation of the solar array from the rest of the high voltage wiring in the vehicle.

## Operation

The following will compile, flash and debug the program.

```shell
DEFMT_LOG=info cargo run
```

## References

- [probe-run](https://github.com/knurling-rs/probe-run)
- [Rust HAL Documentation](https://docs.rs/stm32l4xx-hal/latest/stm32l4xx_hal/)
- [RM0394 Reference Manual for STM32L41xxx/42xxx/43xxx/44xxx/45xxx/46xxx](https://www.st.com/resource/en/reference_manual/dm00151940-stm32l41xxx42xxx43xxx44xxx45xxx46xxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
