/// Big Endian
/// Each address refers to a single byte
/// last byte is at index 4095
const MAX_ADDR: usize = 0xFFF;
/// Size of the memory space in bytes
const MEMORY_SIZE: usize = MAX_ADDR + 1;
/// Program Counter, 16 bits:
/// upper 8 bits at 0x0, lower 8 bits at 0x01
const PC_ADDR: usize = 0x0;
/// Stack Pointer
const SP_ADDR: usize = 0x2;
/// Base address of the stack
const STACK_ADDR: usize = Register::ST as usize;
/// Number of bytes used for the stack
const STACK_SIZE: usize = 16 * 2;
/// Stack address should be less than this address
const STACK_END_ADDR: usize = STACK_ADDR + STACK_SIZE;

/// Contains 8-bit register addresses
#[repr(usize)]
enum Register {
    V0 = SP_ADDR + 1,
    V1,
    V2,
    V3,
    V4,
    V5,
    V6,
    V7,
    V8,
    V9,
    VA,
    VB,
    VC,
    VD,
    VE,
    VF,
    DT,
    ST,
}

/// Data structure for the Chip-8 Interpreter
struct Chip8 {
    // RAM
    memory: [u8; MEMORY_SIZE],
}

impl Chip8 {
    fn new() -> Self {
        Self {
            memory: [0; MEMORY_SIZE],
        }
    }

    /// Access 8 bit register at address
    fn reg8(&mut self, address: Register) -> &mut u8 {
        &mut self.memory[address as usize]
    }

    /// Call at 60 Hz
    fn update(&mut self) {
        
        // Delay Timer
        if *self.reg8(Register::DT) > 0 {
            *self.reg8(Register::DT) -= 1;
        }
        
        // Sound Timer
        if *self.reg8(Register::ST) > 0 {
            *self.reg8(Register::ST) -= 1;
            // TODO: Play tone
        }
    }
}

fn main() {
    let mut chip8 = Chip8::new();
    println!("Hello, world!");
}
