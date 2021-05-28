# Chip-8 Interpreter

Written in Rust without viewing tutorials or other implementations.
The 1-bit color palette is from [here](https://lospec.com/palette-list/paperback-2).
## Usage
Requires Rust to be installed.
Many ROMs available on GitHub.

1. Clone or download and extract repository.
2. `cd path_to_repo`
3. `cargo run path_to_rom_file`

## Controls
Valid keys are 0-9 (row or numpad) and A-F.

## Sound
The single tone buzzer is not yet implemented.

## Bugs
There may be some bugs since some parts of the specification are unclear and expected behaviour between test ROMs differ.

## Library Documentation
If needed the package can be built as a library. To view documentation run `cargo doc --open`.

## Tests
Tests are implemented for instructions with most significant 4-bits 0-C (up to Cxnn). Run tests with `cargo test`.

## References
 - Cowgod's Chip-8 Technical Reference
 - Chip-8 Opcode table on Wikipedia
 - User discussion for behaviour of some instructions like Fx0A