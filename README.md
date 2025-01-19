### Dependencies
- CMake and GNU Make
- `arm-none-eabi` compiler toolchain (from STM32CubeIDE package directory or [here](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads))
- STM32Cube package for your MCU family like the one STM32CubeIDE downloads (from STM32CubeIDE package directory, ST website or [GitHub](https://github.com/STMicroelectronics/STM32Cube_MCU_Overall_Offer#stm32cube-mcu-packages))

### Cloning the repo
```
git clone --recurse-submodules https://github.com/ar7eniyan/obko-test.git
```

### Build process
Create your `CMakeUserPresets.json`:
- `mv CMakeUserPresets.json.template CMakeUserPresets.json`
- Modify the configure options if you need
- Fill in `STM32_CUBE_<family>_PATH` and `STM32_TOOLCHAIN_PATH` variables with your paths

Configure:
```
cmake --preset default
```

Build:
```
cmake --build --preset default
# or (to build and then flash with st-flash)
cmake --build --preset flash
```

Flash (manually):
```
st-flash --reset write build/obko-test.bin 0x08000000
```

