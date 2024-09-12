### Dependencies
- `arm-none-eabi` compiler toolchain
- STM32Cube package for your MCU family like the one STM32CubeIDE downloads from a repository

### Build process
Configure:
```
cmake -DSTM32_CUBE_<family>_PATH=<cube_path> -DSTM32_TOOLCHAIN_PATH=<toolchain_path> -S . -B build
# <family> is F0, F1, ... H7, WL
```

Build:
```
cmake --build build
```

Flash:
```
st-flash --reset write build/obko-test.bin 0x08000000
```

