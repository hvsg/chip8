# Chip-8 Interpreter

![Screenshot of the interpreter running the Cavern ROM.](images/chip8-cavern.png)

Written in Rust, no emulator specific tutorials were used.
Supports regular 64x32 Chip-8 programs and some HiRes 64x64 Chip-8 programs.
The 1-bit color palette is from [here](https://lospec.com/palette-list/paperback-2).
## Usage
Requires Rust to be installed.
Many ROMs available on GitHub.

1. Clone or download and extract repository.
2. `cd path_to_repo`
3. `cargo run path_to_rom_file`

## Controls
Valid keys are 0-9 (row or numpad) and A-F.
Press ESC or close window to exit. To reset, press R.

## Known Issues
- Some test ROMs expect different behaviour from the technical reference

## Library
The package can be built as a library. To view documentation run `cargo doc --open`.

## Tests
Tests are implemented for instructions with most significant 4-bits in range 0-C (up to Cxnn). 
Run tests with `cargo test`.

## ROMs
- [chip8-test-rom](https://github.com/corax89/chip8-test-rom)
- [CHIP-8 Archive](https://johnearnest.github.io/chip8Archive/)

## References
 - Cowgod's Chip-8 Technical Reference
 - Chip-8 Opcode table on Wikipedia
 - User discussion for behaviour of some instructions like Fx0A