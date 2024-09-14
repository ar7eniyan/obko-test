### Dependencies
- `arm-none-eabi` compiler toolchain
- STM32Cube package for your MCU family like the one STM32CubeIDE downloads from a repository

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

