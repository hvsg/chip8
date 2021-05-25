/// Big Endian
/// Each address refers to a single byte
/// last byte is at index 4095
pub const MAX_ADDR: usize = 0xFFF;
/// Size of the memory space in bytes
pub const MEMORY_SIZE: usize = MAX_ADDR + 1;
/// Stack Pointer
pub const SP_ADDR: usize = Reg16::I as usize + 2;
/// Base address of the stack
pub const STACK_ADDR: usize = Reg8::ST as usize;
/// Number of bytes used for the stack
pub const STACK_SIZE: usize = 16 * 2;
/// Stack address should be less than this address
pub const STACK_LAST_ADDR: usize = STACK_ADDR + STACK_SIZE - 2;

/// Contains 16-bit register addresses
#[repr(usize)]
pub enum Reg16 {
    PC = 0x0,
    I = 0x02
}

/// Contains 8-bit register addresses
#[repr(usize)]
pub enum Reg8 {
    SP = SP_ADDR,
    V0,
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
pub struct Chip8 {
    // RAM
    memory: [u8; MEMORY_SIZE],
}

impl Chip8 {
    pub fn new() -> Self {
        Self {
            memory: [0; MEMORY_SIZE],
        }
    }

    /// Read 8-bit register
    fn read_reg8(&self, register: Reg8) -> u8 {
        self.read8(register as usize)
    }

    /// Write 8-bit register
    fn write_reg8(&mut self, register: Reg8, value: u8) {
        self.write8(register as usize, value);
    }
    
    /// Read 16-bit register
    fn read_reg16(&self, register: Reg16) -> u16 {
        self.read16(register as usize)
    }

    /// Write 16-bit reigster
    fn write_reg16(&mut self, register: Reg16, value: u16) {
        self.write16(register as usize, value);
    }

    /// Read 8 bit value at address
    fn read8(&self, address: usize) -> u8 {
        self.memory[address as usize]
    }

    /// Write 8 bit value at address
    fn write8(&mut self, address: usize, value: u8) {
        self.memory[address as usize] = value;
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

    /// Execute instruction i
    fn execute_instruction(&mut self, i: u16) {
        if i == 0x00E0 { // CLS
            // TODO: Clear screen
        }
        else if i == 0x00EE { // RET
            let sp = self.read_reg8(Reg8::SP);
            self.write_reg16(Reg16::PC, sp as u16);
            self.write_reg8(Reg8::SP, sp - 2);
        }
        else if i & 0xF000 == 0x1000 { // JP addr
            self.write_reg16(Reg16::PC, 0x0FFF & i);
        }
        else if i & 0xF000 == 0x2000 { // CALL addr
            let mut sp = self.read_reg8(Reg8::SP);
            // check if maximum stack levels exceeded
            sp += 2;
            if sp > STACK_LAST_ADDR as u8 {
                panic!("Chip-8: Stack Overflow");
            }
            // Increment stack pointer
            self.write_reg8(Reg8::SP, sp);
            // Put current PC on top of stack
            let pc = self.read_reg16(Reg16::PC);
            self.write16(sp as usize, pc);
            // Update PC
            self.write_reg16(Reg16::PC, 0x0FFF & i);
        }
    }

    /// Call at 60 Hz
    pub fn update(&mut self) {

        // Update program counter
        let pc = self.read_reg16(Reg16::PC);
        let instruction = self.read16(pc as usize);
        self.execute_instruction(instruction);
        
        // Delay Timer
        let dt = self.read_reg8(Reg8::DT);
        if dt > 0 {
            self.write_reg8(Reg8::DT, dt - 1);
        }
        
        // Sound Timer
        let st = self.read_reg8(Reg8::ST);
        if st > 0 {
            self.write_reg8(Reg8::ST, st - 1);
            // TODO: Play tone
        }
    }
}

