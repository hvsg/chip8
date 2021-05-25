use rand;

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
/// Screen width in pixels
pub const SCREEN_WIDTH: usize = 64;
/// Screen height in pixels
pub const SCREEN_HEIGHT: usize = 32;
/// Number of Pixels
pub const NUM_PIXELS: usize = SCREEN_WIDTH * SCREEN_HEIGHT;
/// Size in bytes of program counter
pub const ADDR_SIZE: u16 = 2;

/// Contains 16-bit register addresses
#[repr(usize)]
#[derive(Clone, Copy)]
pub enum Reg16 {
    PC = 0x0,
    I = 0x02,
}

/// Contains 8-bit register addresses
#[repr(usize)]
#[derive(Clone, Copy)]
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

/// Convert unsize to general purpose register (V0-VF)
fn to_general_reg8(x: usize) -> Reg8 {
    assert!(x >= Reg8::V0 as usize);
    assert!(x <= Reg8::VF as usize);
    unsafe { std::mem::transmute(x) }
}

/// Data structure for the Chip-8 Interpreter
pub struct Chip8 {
    // RAM
    memory: [u8; MEMORY_SIZE],
    // Display memory, each byte corresponds to a pixel
    display: [u8; NUM_PIXELS],
}

impl Chip8 {
    pub fn new() -> Self {
        let mut s = Self {
            memory: [0; MEMORY_SIZE],
            display: [0; NUM_PIXELS],
        };
        // Initialize registers
        s.write_reg16(Reg16::PC, 0x0600);
        s.write_reg8(Reg8::SP, (STACK_ADDR - ADDR_SIZE as usize) as u8);
        s
    }

    fn clear_screen(&mut self) {
        self.display = [0; NUM_PIXELS];
    }

    /// XOR byte for given (x, y) coordinates
    /// where (0, 0) is top left and (width-1, height-1) is bottom right
    /// Returns 1 if bit was erased, otherwise 0
    fn draw(&mut self, x: usize, y: usize, value: u8) -> u8 {
        // compute address
        let index = y * SCREEN_WIDTH + x;
        let pixel = self.display[index];
        self.display[index] ^= value;
        (pixel == value) as u8
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
        let advance_pc: bool = if i == 0x00E0 {
            // CLS
            self.clear_screen();
            true
        } else if i == 0x00EE {
            // RET
            let sp = self.read_reg8(Reg8::SP);
            // Check if stack is empty
            if sp < STACK_ADDR as u8 {
                panic!("Chip-8: Stack Underflow");
            }
            self.write_reg16(Reg16::PC, sp as u16);
            self.write_reg8(Reg8::SP, sp - ADDR_SIZE as u8);
            false
        } else if i & 0xF000 == 0x1000 {
            // JP addr
            self.write_reg16(Reg16::PC, 0x0FFF & i);
            false
        } else if i & 0xF000 == 0x2000 {
            // CALL addr
            let mut sp = self.read_reg8(Reg8::SP);
            // check if stack is full
            sp += ADDR_SIZE as u8;
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
            false
        } else if i & 0xF000 == 0x3000 {
            // Skip if register Vx equals lower byte
            let x = (i & 0x0F00) >> 8;
            let vx = to_general_reg8(x as usize);
            let lower: u8 = (0x00FF & i) as u8;
            // Skip instruction
            if self.read_reg8(vx) == lower {
                let pc = self.read_reg16(Reg16::PC);
                self.write_reg16(Reg16::PC, pc + 2 * ADDR_SIZE);
            }
            false
        } else if i & 0xF000 == 0x4000 {
            // Skip if register Vx neq to lower byte
            let x = (i & 0x0F00) >> 8;
            let vx = to_general_reg8(x as usize);
            let lower = (0x00FF & i) as u8;
            // Skip instruction
            if self.read_reg8(vx) != lower {
                let pc = self.read_reg16(Reg16::PC);
                self.write_reg16(Reg16::PC, pc + 2 * ADDR_SIZE);
            }
            false
        } else if i & 0xF000 == 0x5000 {
            // Skip if Vx == Vy
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            let vy = to_general_reg8(((i & 0x00F0) >> 4) as usize);
            if self.read_reg8(vx) == self.read_reg8(vy) {
                let pc = self.read_reg16(Reg16::PC);
                self.write_reg16(Reg16::PC, pc + 2 * ADDR_SIZE);
            }
            false
        } else if i & 0xF000 == 0x6000 {
            // Put lower byte value into Vx
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            let lower = (0x00FF & i) as u8;
            self.write_reg8(vx, lower);
            true
        } else if i & 0xF000 == 0x7000 {
            // Vx += lower
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            let lower = (0x00FF & i) as u8;
            let value = self.read_reg8(vx).overflowing_add(lower).0;
            self.write_reg8(vx, value);
            true
        } else if i & 0xF000 == 0x8000 {
            // Vx = Vy
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            let vy = to_general_reg8(((i & 0x00F0) >> 4) as usize);
            self.write_reg8(vx, self.read_reg8(vy));
            true
        } else if i & 0xF00F == 0x8001 {
            // Vx |= Vy
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            let vy = to_general_reg8(((i & 0x00F0) >> 4) as usize);
            let value = self.read_reg8(vx) | self.read_reg8(vy);
            self.write_reg8(vx, value);
            true
        } else if i & 0xF00F == 0x8002 {
            // Vx &= Vy
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            let vy = to_general_reg8(((i & 0x00F0) >> 4) as usize);
            let value = self.read_reg8(vx) & self.read_reg8(vy);
            self.write_reg8(vx, value);
            true
        } else if i & 0xF00F == 0x8003 {
            // Vx ^= Vy
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            let vy = to_general_reg8(((i & 0x00F0) >> 4) as usize);
            let value = self.read_reg8(vx) ^ self.read_reg8(vy);
            self.write_reg8(vx, value);
            true
        } else if i & 0xF00F == 0x8004 {
            // Vx  += Vy and VF = carry
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            let vy = to_general_reg8(((i & 0x00F0) >> 4) as usize);
            let (value, carry) = self.read_reg8(vx).overflowing_add(self.read_reg8(vy));
            self.write_reg8(vx, value);
            self.write_reg8(Reg8::VF, carry as u8);
            true
        } else if i & 0xF00F == 0x8005 {
            // VF = Vx > Vy (!borrow) and Vx -= Vy
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            let vy = to_general_reg8(((i & 0x00F0) >> 4) as usize);

            let (value, borrow) = self.read_reg8(vx).overflowing_sub(self.read_reg8(vy));
            self.write_reg8(vx, value);
            self.write_reg8(Reg8::VF, !borrow as u8);
            true
        } else if i & 0xF00F == 0x8006 {
            // VF = LSB(Vx) and Vx /= 2
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            let x = self.read_reg8(vx);
            self.write_reg8(Reg8::VF, x & 0x1);
            self.write_reg8(vx, x >> 1);
            true
        } else if i & 0xF00F == 0x8007 {
            // VF = Vy > Vx, Vx = Vy - Vx
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            let vy = to_general_reg8(((i & 0x00F0) >> 4) as usize);
            let x = self.read_reg8(vx);
            let y = self.read_reg8(vy);
            let (value, underflow) = y.overflowing_sub(x);
            self.write_reg8(vx, value);
            self.write_reg8(Reg8::VF, !underflow as u8);
            true
        } else if i & 0xF00F == 0x800E {
            // Vx = Vx << 1 and VF = overflow
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            let x = self.read_reg8(vx);
            self.write_reg8(Reg8::VF, x & (0x1 << 7));
            self.write_reg8(vx, x << 1);
            true
        } else if i & 0xF00F == 0x9000 {
            // Skip if Vx != Vy
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            let vy = to_general_reg8(((i & 0x00F0) >> 4) as usize);
            if self.read_reg8(vx) != self.read_reg8(vy) {
                let pc = self.read_reg16(Reg16::PC);
                self.write_reg16(Reg16::PC, pc + 2 * ADDR_SIZE);
            }
            false
        } else if i & 0xF000 == 0xA000 {
            // I = value
            self.write_reg16(Reg16::I, i & 0x0FFF);
            true
        } else if i & 0xF000 == 0xB000 {
            // JP V0 + value
            let v0 = self.read_reg8(Reg8::V0);
            self.write_reg16(Reg16::PC, (i & 0x0FFF) + v0 as u16);
            false
        } else if i & 0xF000 == 0xC000 {
            //  Vx = random & lower
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            let x = self.read_reg8(vx);
            let r: u8 = rand::random();
            self.write_reg8(vx, r & x);
            true
        } else if i & 0xF000 == 0xD000 {
            // XOR sprite of n bytes at x and y and set VF to 1 if pixel erased
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            let vy = to_general_reg8(((i & 0x00F0) >> 4) as usize);
            let n = (i & 0x000F) as u8;

            let x = self.read_reg8(vx);
            let mut draw_y = self.read_reg8(vy);

            let sprite = self.read_reg16(Reg16::I);
            let mut erased: u8 = 0;

            for j in 0..n {
                // load byte
                let mut byte = self.read8(sprite.saturating_add(j as u16) as usize);
                // assuming that vx,vy corresponds to first byte (top-left corner) of sprite
                for k in 0..8 {
                    let draw_x = x.overflowing_add(k).0;
                    erased |= self.draw(draw_x as usize, draw_y as usize, byte & (0x1 << 7));
                    byte <<= 1;
                }
                draw_y = draw_y.overflowing_add(1).0;
            }
            self.write_reg8(Reg8::VF, erased);
            true
        } else if i & 0xF0FF == 0xE09E {
            // Skip if Vx is pressed
            unimplemented!();
        } else if i & 0xF0FF == 0xE0A1 {
            // Skip if Vx is not pressed
            unimplemented!();
        } else if i & 0xF0FF == 0xF007 {
            // Vx = DT
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            let dt = self.read_reg8(Reg8::DT);
            self.write_reg8(vx, dt);
            true
        } else if i & 0xF0FF == 0xF00A {
            // Wait for key press and store key value in Vx
            unimplemented!()
        } else if i & 0xF0FF == 0xF015 {
            // DT = Vx
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            self.write_reg8(Reg8::DT, self.read_reg8(vx));
            true
        } else if i & 0xF0FF == 0xF018 {
            // ST = Vx
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            self.write_reg8(Reg8::ST, self.read_reg8(vx));
            true
        } else if i & 0xF0FF == 0xF01E {
            // I += Vx
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            let x = self.read_reg8(vx);
            let value = self.read_reg16(Reg16::I);
            self.write_reg16(Reg16::I, value.overflowing_add(x as u16).0);
            true
        } else if i & 0xF0FF == 0xF029 {
            unimplemented!()
        } else if i & 0xF0FF == 0xF033 {
            unimplemented!()
        } else if i & 0xF0FF == 0xF055 {
            unimplemented!()
        } else if i & 0xF0FF == 0xF065 {
            unimplemented!()
        } else {
            panic!("Chip-8: Invalid instruction {:X}", i);
        };
        if advance_pc {
            let pc = self.read_reg16(Reg16::PC);
            self.write_reg16(Reg16::PC, pc + ADDR_SIZE);
        }
    }

    /// Call at 60 Hz
    pub fn update(&mut self) {
        // Execute instruction at program counter
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
