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

    /// Read 8 bit value at address
    fn read8(&self, address: Register) -> u8 {
        self.memory[address as usize]
    }

    /// Write 8 bit value at address
    fn write8(&mut self, address: Register, value: u8) {
        self.memory[address as usize] = value;
    }

    /// Execute instruction i
    fn execute_instruction(&mut self, i: u16) {

    }
    
    /// Read 16 bit value at address
    fn read16(&self, address: usize) -> u16 {
        let upper = self.memory[address];
        let lower = self.memory[address + 1];
        u16::from_be_bytes([upper, lower])
    }

    /// Write 16 bit value at address
    fn write16(&mut self, address: usize, value: u16) {
        let bytes = value.to_be_bytes();
        self.memory[address] = bytes[0];
        self.memory[address + 1] = bytes[1];
    }

    /// Call at 60 Hz
    fn update(&mut self) {

        // Update program counter
        let pc = self.read16(PC_ADDR);
        let instruction = self.read16(pc as usize);
        self.execute_instruction(instruction);
        
        // Delay Timer
        let dt = self.read8(Register::DT);
        if dt > 0 {
            self.write8(Register::DT, dt - 1);
        }
        
        // Sound Timer
        let st = self.read8(Register::ST);
        if st > 0 {
            self.write8(Register::ST, st - 1);
            // TODO: Play tone
        }
    }
}

fn main() {
    let mut chip8 = Chip8::new();
    println!("Hello, world!");
}
