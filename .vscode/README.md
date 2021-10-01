# Reference
[example-stm32l432kc](https://github.com/stm32-rs/stm32-usbd-examples/tree/master/example-stm32l432kc)

# Run

```bash
$ cargo build
$ openocd -f openocd.cfg &
$ arm-none-eabi-gdb -x flash.gdb
```