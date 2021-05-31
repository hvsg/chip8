/// Big Endian
/// Each address refers to a single byte
/// last byte is at index 4095
pub const MAX_ADDR: usize = 0xFFF;
/// Size of the memory space in bytes
pub const MEMORY_SIZE: usize = MAX_ADDR + 1;
/// Stack Pointer
pub const SP_ADDR: usize = Reg16::I as usize + 2;
/// Base address of the stack
pub const STACK_ADDR: usize = Reg8::ST as usize + 1;
/// Number of bytes used for the stack
pub const STACK_SIZE: usize = 16 * 2;
/// Stack address should be less than this address
pub const MAXIMUM_STACK_ADDR: usize = STACK_ADDR + STACK_SIZE - 2;
/// Screen width in pixels
const SCREEN_WIDTH: usize = 64;
/// Screen height in pixels
const SCREEN_HEIGHT: usize = 32;
/// Size in bytes of program counter
pub const ADDR_SIZE: u16 = 2;
/// Base address of built-in sprites
pub const SPRITE_BASE_ADDR: usize = STACK_ADDR + STACK_SIZE;
/// Size of built-in alpha-numeric sprites in bytes.
/// Built-in sprites include 0-9, A-F
pub const SPRITE_SIZE: usize = 5;
/// Number of built-in sprites
pub const NUM_SPRITES: usize = 16;
/// Start address of program
pub const PROGRAM_START_ADDR: usize = 0x200;
/// Start address of hires mode program
pub const HIRES_PROGRAM_START_ADDR: usize = 0x2C0;

// From Cowgod's Chip-8 Technical Reference v1.0 by Thomas P. Greene
pub const HEX_SPRITES: [u8; NUM_SPRITES * SPRITE_SIZE] = [
    0xF0, 0x90, 0x90, 0x90, 0xF0, // 0
    0x20, 0x60, 0x20, 0x20, 0x70, // 1
    0xF0, 0x10, 0xF0, 0x80, 0xF0, // 2
    0xF0, 0x10, 0xF0, 0x10, 0xF0, // 3
    0x90, 0x90, 0xF0, 0x10, 0x10, // 4
    0xF0, 0x80, 0xF0, 0x10, 0xF0, // 5
    0xF0, 0x80, 0xF0, 0x90, 0xF0, // 6
    0xF0, 0x10, 0x20, 0x40, 0x40, // 7
    0xF0, 0x90, 0xF0, 0x90, 0xF0, // 8
    0xF0, 0x90, 0xF0, 0x10, 0xF0, // 9
    0xF0, 0x90, 0xF0, 0x90, 0x90, // A
    0xE0, 0x90, 0xE0, 0x90, 0xE0, // B
    0xF0, 0x80, 0x80, 0x80, 0xF0, // C
    0xE0, 0x90, 0x90, 0x90, 0xE0, // D
    0xF0, 0x80, 0xF0, 0x80, 0xF0, // E
    0xF0, 0x80, 0xF0, 0x80, 0x80, // F
];

/// Contains 16-bit register addresses
#[repr(usize)]
#[derive(Clone, Copy)]
pub enum Reg16 {
    /// Program Counter
    PC = 0x0,
    /// Previous Keyboard Input
    PK = 0x2,
    /// Current Keyboard Input
    KB = 0x4,
    /// Register I
    I = 0x6,
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
    let x = x + Reg8::V0 as usize;
    assert!(x >= Reg8::V0 as usize);
    assert!(x <= Reg8::VF as usize);
    unsafe { std::mem::transmute(x) }
}

/// Data structure for the Chip-8 Interpreter
pub struct Chip8 {
    /// RAM
    memory: [u8; MEMORY_SIZE],
    /// Display memory, each byte corresponds to a pixel
    display: Vec<u8>,
    /// Screen dimensions
    dimensions: (usize, usize),
}

impl Default for Chip8 {
    /// Initializes default instance of Chip-8
    fn default() -> Self {
        Self::new()
    }
}

impl Chip8 {
    /// Creates and initializes new instance of Chip-8
    pub fn new() -> Self {
        static_assertions::const_assert!(
            PROGRAM_START_ADDR > (SPRITE_BASE_ADDR + SPRITE_SIZE * NUM_SPRITES)
        );

        let mut s = Self {
            memory: [0; MEMORY_SIZE],
            display: Vec::new(),
            dimensions: (0, 0),
        };
        // Initialize registers
        s.write_reg16(Reg16::PC, PROGRAM_START_ADDR as u16);
        s.write_reg8(Reg8::SP, (STACK_ADDR - ADDR_SIZE as usize) as u8);
        // Load sprites
        s.load_sprites();
        s
    }

    /// Returns slice to screen texture in format of monochromatic single byte texture.
    /// Should call after loading ROM.
    pub fn get_screen_texture(&self) -> &[u8] {
        &self.display
    }

    /// Returns tuple of screen (width, height) in pixels.
    /// Should call after loading ROM
    pub fn get_dimensions(&self) -> (usize, usize) {
        self.dimensions
    }

    /// Accepts hex value input (0-F) representing key and updates input state.
    pub fn read_input(&mut self, input: u8, pressed: bool) {
        let mask = 0x1u16 << input;
        let kb = self.read_reg16(Reg16::KB);
        let value = if pressed { kb | mask } else { kb & !mask };
        self.write_reg16(Reg16::KB, value);
    }

    /// Loads the ROM into memory.
    /// Should only be called once before initializing the display.
    pub fn load_rom(&mut self, rom: &[u8]) {
        let dst = &mut self.memory[PROGRAM_START_ADDR..(PROGRAM_START_ADDR + rom.len())];
        dst.copy_from_slice(rom);

        // Initialize display
        let x1 = rom[0];
        let x0 = rom[1];
        let i = u16::from_be_bytes([x1, x0]);
        // Check for high-res mode
        let (w, h) = if i == 0x1260 {
            self.write_reg16(Reg16::PC, HIRES_PROGRAM_START_ADDR as u16);
            (SCREEN_WIDTH, SCREEN_HEIGHT * 2)
        } else {
            self.write_reg16(Reg16::PC, PROGRAM_START_ADDR as u16);
            (SCREEN_WIDTH, SCREEN_HEIGHT)
        };

        if self.dimensions != (0, 0) {
            eprintln!("Warning: Tried to load ROM with display already initialized.")
        }

        self.display = vec![0; w * h];
        self.dimensions = (w, h);
    }

    /// Returns true if buzzer is on, false otherwise
    pub fn buzzer_active(&mut self) -> bool {
        self.read_reg8(Reg8::ST) > 0
    }

    /// Get memory address of hex digit (0-9 or A-F)
    fn get_digit_location(digit: u8) -> u16 {
        assert!(digit <= 0xF);
        SPRITE_BASE_ADDR as u16 + (digit as u16) * SPRITE_SIZE as u16
    }

    /// Load built-in sprites into memory
    fn load_sprites(&mut self) {
        let dest =
            &mut self.memory[SPRITE_BASE_ADDR..(SPRITE_BASE_ADDR + SPRITE_SIZE * NUM_SPRITES)];
        dest.copy_from_slice(&HEX_SPRITES);
    }

    /// Turns off all pixels
    pub fn clear_screen(&mut self) {
        // self.display = [0; NUM_PIXELS];
        self.display.iter_mut().for_each(|byte| {
            *byte = 0;
        });
    }

    /// XOR byte for given (x, y) coordinates
    /// where (0, 0) is top left and (width-1, height-1) is bottom right
    /// Returns 1 if bit was erased, otherwise 0
    fn draw(&mut self, x: usize, y: usize, value: u8) -> u8 {
        // compute address
        let (w, _) = self.get_dimensions();
        let index = y * w + x;
        let pixel = self.display[index];
        self.display[index] ^= value;
        (pixel > 0 && value > 0) as u8
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
        let advance_pc: bool = if i == 0x00E0 || i == 0x230 {
            // CLS
            self.clear_screen();
            true
        } else if i == 0x00EE {
            // RET
            // Check if stack is empty
            let sp = self.read_reg8(Reg8::SP);
            if (sp as usize) < STACK_ADDR {
                panic!("Chip-8: Stack Underflow");
            }
            // get return address stored at stack pointer
            let addr = self.read16(sp as usize);
            self.write_reg16(Reg16::PC, addr);
            // decrement stack pointer
            self.write_reg8(Reg8::SP, sp - ADDR_SIZE as u8);
            true
        } else if i & 0xF000 == 0x1000 {
            // JP addr
            self.write_reg16(Reg16::PC, 0x0FFF & i);
            false
        } else if i & 0xF000 == 0x2000 {
            // CALL addr
            let sp = self.read_reg8(Reg8::SP);
            // check if stack is full
            if sp == MAXIMUM_STACK_ADDR as u8 {
                panic!("Chip-8: Stack Overflow");
            }
            // Increment stack pointer
            self.write_reg8(Reg8::SP, sp + ADDR_SIZE as u8);
            // Put current PC on top of stack (store PC at stack pointer)
            self.write16((sp + ADDR_SIZE as u8) as usize, self.read_reg16(Reg16::PC));
            // Update PC (JP)
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
                false
            } else {
                true
            }
        } else if i & 0xF000 == 0x4000 {
            // Skip if register Vx neq to lower byte
            let x = (i & 0x0F00) >> 8;
            let vx = to_general_reg8(x as usize);
            let lower = (0x00FF & i) as u8;
            // Skip instruction
            if self.read_reg8(vx) != lower {
                let pc = self.read_reg16(Reg16::PC);
                self.write_reg16(Reg16::PC, pc + 2 * ADDR_SIZE);
                false
            } else {
                true
            }
        } else if i & 0xF00F == 0x5000 {
            // Skip if Vx == Vy
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            let vy = to_general_reg8(((i & 0x00F0) >> 4) as usize);
            if self.read_reg8(vx) == self.read_reg8(vy) {
                let pc = self.read_reg16(Reg16::PC);
                self.write_reg16(Reg16::PC, pc + 2 * ADDR_SIZE);
                false
            } else {
                true
            }
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
        } else if i & 0xF00F == 0x8000 {
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
            let x = self.read_reg8(vx);
            let y = self.read_reg8(vy);
            let (value, _) = x.overflowing_sub(y);
            self.write_reg8(vx, value);
            // self.write_reg8(Reg8::VF, !borrow as u8);
            self.write_reg8(Reg8::VF, (x > y) as u8);
            true
        } else if i & 0xF00F == 0x8006 {
            // VF = LSB(Vx) and Vx /= 2
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            let x = self.read_reg8(vx);
            self.write_reg8(Reg8::VF, x & 0x1u8);
            self.write_reg8(vx, x >> 1);
            true
        } else if i & 0xF00F == 0x8007 {
            // VF = Vy > Vx, Vx = Vy - Vx
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            let vy = to_general_reg8(((i & 0x00F0) >> 4) as usize);
            let x = self.read_reg8(vx);
            let y = self.read_reg8(vy);
            let (value, _) = y.overflowing_sub(x);
            self.write_reg8(vx, value);
            // self.write_reg8(Reg8::VF, !underflow as u8);
            self.write_reg8(Reg8::VF, (y > x) as u8);
            true
        } else if i & 0xF00F == 0x800E {
            // Vx = Vx << 1 and VF = overflow
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            let x = self.read_reg8(vx);
            self.write_reg8(Reg8::VF, (x >> 7) & 0x1u8);
            self.write_reg8(vx, x << 1);
            true
        } else if i & 0xF00F == 0x9000 {
            // Skip if Vx != Vy
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            let vy = to_general_reg8(((i & 0x00F0) >> 4) as usize);
            if self.read_reg8(vx) != self.read_reg8(vy) {
                let pc = self.read_reg16(Reg16::PC);
                self.write_reg16(Reg16::PC, pc + 2 * ADDR_SIZE);
                false
            } else {
                true
            }
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
            let nn = (i & 0x00FF) as u8;
            let r: u8 = rand::random();
            self.write_reg8(vx, r & nn);
            true
        } else if i & 0xF000 == 0xD000 {
            // XOR sprite of n bytes at x and y and set VF to 1 if pixel erased
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            let vy = to_general_reg8(((i & 0x00F0) >> 4) as usize);
            let num_bytes = (i & 0x000F) as u8;

            let x = self.read_reg8(vx);
            let mut draw_y = self.read_reg8(vy);

            let sprite = self.read_reg16(Reg16::I);
            let mut erased: u8 = 0;
            let (w, h) = self.get_dimensions();
            for j in 0..num_bytes {
                // load byte
                let mut byte = self.read8(sprite.saturating_add(j as u16) as usize);
                // assuming that vx,vy corresponds to first byte (top-left corner) of sprite
                for k in 0..8 {
                    let draw_x = x.overflowing_add(k).0 % w as u8;
                    let brightness = if (byte & (0x1u8 << 7)) > 0 {
                        u8::MAX
                    } else {
                        0u8
                    };
                    erased |= self.draw(draw_x as usize, draw_y as usize, brightness);
                    byte <<= 1;
                }
                draw_y = draw_y.overflowing_add(1).0 % h as u8;
            }
            self.write_reg8(Reg8::VF, erased);
            true
        } else if i & 0xF0FF == 0xE09E {
            // Skip if Vx is pressed
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            let value = self.read_reg8(vx);
            let kb = self.read_reg16(Reg16::KB);

            if kb & (1u16 << value) == (1u16 << value) {
                let pc = self.read_reg16(Reg16::PC);
                self.write_reg16(Reg16::PC, pc + 2 * ADDR_SIZE);
                false
            } else {
                true
            }
        } else if i & 0xF0FF == 0xE0A1 {
            // Skip if Vx is not pressed
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            let value = self.read_reg8(vx);
            let kb = self.read_reg16(Reg16::KB);

            if kb & (1u16 << value) != (1u16 << value) {
                let pc = self.read_reg16(Reg16::PC);
                self.write_reg16(Reg16::PC, pc + 2 * ADDR_SIZE);
                false
            } else {
                true
            }
        } else if i & 0xF0FF == 0xF007 {
            // Vx = DT
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            let dt = self.read_reg8(Reg8::DT);
            self.write_reg8(vx, dt);
            true
        } else if i & 0xF0FF == 0xF00A {
            // Wait for key press and store key value in Vx
            // This should only trigger for new key presses
            let mut kb = self.read_reg16(Reg16::KB);
            let pkb = self.read_reg16(Reg16::PK);
            let mut value = 0;
            while kb > 0 {
                kb >>= 1;
                value += 1;
            }
            // If there was a *new* key press
            // let mask: u8 =
            if value > 0 && ((0x1 << (value - 1)) & pkb) == 0 {
                let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
                self.write_reg8(vx, value - 1);
                true
            } else {
                false
            }
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
            // I = sprite for Vx
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            let digit = self.read_reg8(vx);
            let location = Self::get_digit_location(digit);
            self.write_reg16(Reg16::I, location);
            true
        } else if i & 0xF0FF == 0xF033 {
            // Store BCD representation of Vx starting at I
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            let mut value = self.read_reg8(vx);
            let start = self.read_reg16(Reg16::I);
            let h = value / 100;
            self.write8(start as usize, h);
            value -= h * 100;
            let t = value / 10;
            self.write8((start + 1) as usize, t);
            self.write8((start + 2) as usize, value - t * 10);
            true
        } else if i & 0xF0FF == 0xF055 {
            // write v0-vx into memory starting at I
            let start = self.read_reg16(Reg16::I);
            let x = (i & 0x0F00) >> 8;
            for j in 0..=x {
                // Read value
                let register = to_general_reg8(j as usize);
                let value = self.read_reg8(register);
                // store
                self.write8((start + j) as usize, value);
            }
            true
        } else if i & 0xF0FF == 0xF065 {
            // read memory from I into v0-vx
            let start = self.read_reg16(Reg16::I);
            let x = (i & 0x0F00) >> 8;
            for j in 0..=x {
                // Read value
                let value = self.read8((start + j) as usize);
                // store
                let register = to_general_reg8(j as usize);
                self.write_reg8(register, value);
            }
            true
        } else {
            panic!("Chip-8: Invalid instruction {:X}", i);
        };
        if advance_pc {
            let pc = self.read_reg16(Reg16::PC);
            self.write_reg16(Reg16::PC, pc + ADDR_SIZE);
        }
    }

    /// Call at ~500 Hz
    pub fn update_logic(&mut self) {
        // Execute instruction at program counter
        let pc = self.read_reg16(Reg16::PC);
        let instruction = self.read16(pc as usize);
        self.execute_instruction(instruction);
        // Store previous keyboard input
        let kb = self.read_reg16(Reg16::KB);
        self.write_reg16(Reg16::PK, kb);
    }

    /// Call at 60 Hz
    pub fn update_timers(&mut self) {
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

#[cfg(test)]
mod tests {
    use super::{Chip8, Reg16, Reg8, ADDR_SIZE, STACK_ADDR};
    #[test]
    fn initialization() {
        let c = Chip8::new();
        assert_eq!(c.read_reg16(Reg16::PC), 0x200);
        assert_eq!(c.read_reg16(Reg16::I), 0x0);
        assert_eq!(c.read_reg16(Reg16::KB), 0x0);
    }

    #[test]
    fn read_write_registers() {
        let mut c = Chip8::new();
        c.write_reg8(Reg8::V0, u8::MAX);
        assert_eq!(c.read_reg8(Reg8::V0), u8::MAX);

        c.write_reg16(Reg16::I, 0xFA);
        assert_eq!(c.read_reg16(Reg16::I), 0xFA);
    }

    #[test]
    fn cls() {
        let mut c = Chip8::new();
        c.display = [u8::MAX; 64 * 32].to_vec();
        c.load_rom(&[0x00, 0xE0]);
        c.update_logic();
        assert_eq!(c.display, [0u8; 64 * 32]);
        assert_eq!(c.read_reg16(Reg16::PC), 0x200 + ADDR_SIZE);
    }

    #[test]
    fn jp() {
        let mut c = Chip8::new();
        c.load_rom(&[0x1F, 0xF2]);
        c.update_logic();
        assert_eq!(c.read_reg16(Reg16::PC), 0xFF2);
    }

    #[test]
    fn call_ret() {
        let mut c = Chip8::new();
        c.load_rom(&[0x22, 0x04, 0x00, 0x00, 0x00, 0xEE]);

        // CALL: Test stack pointer and return address
        c.update_logic();
        assert_eq!(c.read_reg16(Reg16::PC), (0x204));
        let sp = c.read_reg8(Reg8::SP);
        assert_eq!(sp, STACK_ADDR as u8);
        // RET: Test stack pointer and restored address
        c.update_logic();
        assert_eq!(c.read_reg16(Reg16::PC), 0x202);
        assert_eq!(c.read_reg8(Reg8::SP), (STACK_ADDR as u8 - 2));
    }

    #[test]
    fn call_ret_16() {
        let mut c = Chip8::new();
        for i in 1..=16 {
            c.execute_instruction(0x2200 + i * ADDR_SIZE);
            assert_eq!(c.read_reg16(Reg16::PC), (0x200 + i * ADDR_SIZE));
            let sp = c.read_reg8(Reg8::SP);
            assert_eq!(sp, (STACK_ADDR as u8 + ((i - 1) * ADDR_SIZE) as u8));
        }
        // RET: Test stack pointer and restored address
        for i in (0..=15).rev() {
            c.execute_instruction(0x00EE);
            assert_eq!(c.read_reg16(Reg16::PC), (0x200 + (i + 1) * ADDR_SIZE));
            let sp = c.read_reg8(Reg8::SP);
            assert_eq!(sp, (STACK_ADDR - 2 + 2 * (i as usize)) as u8);
        }
    }
    #[test]
    fn i3xnn_skip_eq() {
        let mut c = Chip8::new();
        c.write_reg8(Reg8::VA, 0xAA);
        c.load_rom(&[0x3A, 0xAB, 0x3A, 0xAA]);

        // Test skip
        c.update_logic();
        assert_eq!(c.read_reg16(Reg16::PC), (0x200 + ADDR_SIZE));
        c.update_logic();
        assert_eq!(c.read_reg16(Reg16::PC), (0x200 + 3 * ADDR_SIZE));
    }

    #[test]
    fn i4xnn_skip_neq() {
        let mut c = Chip8::new();
        c.write_reg8(Reg8::VA, 0xAA);
        c.load_rom(&[0x4A, 0xAA, 0x4A, 0xAB]);

        // Test skip
        c.update_logic();
        assert_eq!(c.read_reg16(Reg16::PC), (0x200 + ADDR_SIZE));
        c.update_logic();
        assert_eq!(c.read_reg16(Reg16::PC), (0x200 + 3 * ADDR_SIZE));
    }

    #[test]
    fn i5xy0_skip_xy_eq() {
        let mut c = Chip8::new();
        c.write_reg8(Reg8::VA, 0xAA);
        c.write_reg8(Reg8::VB, 0xAB);
        c.write_reg8(Reg8::VC, 0xAA);

        c.load_rom(&[0x5A, 0xB0, 0x5A, 0xC0]);

        // Test skip
        c.update_logic();
        assert_eq!(c.read_reg16(Reg16::PC), (0x200 + ADDR_SIZE));
        c.update_logic();
        assert_eq!(c.read_reg16(Reg16::PC), (0x200 + 3 * ADDR_SIZE));
    }

    #[test]
    fn i6xnn_load() {
        let mut c = Chip8::new();

        c.load_rom(&[0x6A, 0xBC]);
        // Test register for loaded value
        c.update_logic();
        assert_eq!(c.read_reg16(Reg16::PC), (0x200 + ADDR_SIZE));
        assert_eq!(c.read_reg8(Reg8::VA), 0xBC);
    }

    #[test]
    fn i7xnn_add() {
        let mut c = Chip8::new();
        c.write_reg8(Reg8::VA, 0xFF);
        c.load_rom(&[0x7A, 0x01]);
        // Test registers
        c.update_logic();
        assert_eq!(c.read_reg16(Reg16::PC), (0x200 + ADDR_SIZE));
        assert_eq!(c.read_reg8(Reg8::VA), 0x00);
    }

    #[test]
    fn i8xy0_load() {
        let mut c = Chip8::new();
        c.write_reg8(Reg8::VB, 0xFF);
        c.load_rom(&[0x8A, 0xB0]);
        // Test registers
        c.update_logic();
        assert_eq!(c.read_reg16(Reg16::PC), (0x200 + ADDR_SIZE));
        assert_eq!(c.read_reg8(Reg8::VA), 0xFF);
    }

    #[test]
    fn i8xy1_or() {
        let mut c = Chip8::new();
        c.write_reg8(Reg8::VA, 0b10101010);
        c.write_reg8(Reg8::VB, 0b01010101);
        c.load_rom(&[0x8A, 0xB1]);
        // Test registers
        c.update_logic();
        assert_eq!(c.read_reg16(Reg16::PC), (0x200 + ADDR_SIZE));
        assert_eq!(c.read_reg8(Reg8::VA), 0xFF);
    }

    #[test]
    fn i8xy2_and() {
        let mut c = Chip8::new();
        c.write_reg8(Reg8::VA, 0b10101010);
        c.write_reg8(Reg8::VB, 0b01010111);
        c.load_rom(&[0x8A, 0xB2]);
        // Test registers
        c.update_logic();
        assert_eq!(c.read_reg16(Reg16::PC), (0x200 + ADDR_SIZE));
        assert_eq!(c.read_reg8(Reg8::VA), 0x2);
    }

    #[test]
    fn i8xy3_xor() {
        let mut c = Chip8::new();
        c.write_reg8(Reg8::VA, 0b10101010);
        c.write_reg8(Reg8::VB, 0b01010111);
        c.load_rom(&[0x8A, 0xB3]);
        // Test registers
        c.update_logic();
        assert_eq!(c.read_reg16(Reg16::PC), (0x200 + ADDR_SIZE));
        assert_eq!(c.read_reg8(Reg8::VA), 0b11111101);
    }

    #[test]
    fn i8xy4_add_carry() {
        let mut c = Chip8::new();
        c.write_reg8(Reg8::VA, 0xFF);
        c.write_reg8(Reg8::VB, 0x01);
        c.load_rom(&[0x8A, 0xB4]);
        // Test registers
        c.update_logic();
        assert_eq!(c.read_reg16(Reg16::PC), (0x200 + ADDR_SIZE));
        assert_eq!(c.read_reg8(Reg8::VA), 0);
        assert_eq!(c.read_reg8(Reg8::VF), 1);
    }

    #[test]
    fn i8xy5_sub() {
        let mut c = Chip8::new();
        c.write_reg8(Reg8::VA, 0xFF);
        c.write_reg8(Reg8::VB, 0x01);
        c.write_reg8(Reg8::VC, 0x01);
        c.write_reg8(Reg8::VD, 0x01);

        c.load_rom(&[0x8A, 0xB5, 0x8B, 0xA5, 0x8C, 0xD5]);
        // Test registers
        c.update_logic();
        assert_eq!(c.read_reg16(Reg16::PC), (0x200 + ADDR_SIZE));
        assert_eq!(c.read_reg8(Reg8::VA), 0xFE);
        assert_eq!(c.read_reg8(Reg8::VF), 1);
        c.update_logic();
        assert_eq!(c.read_reg16(Reg16::PC), (0x200 + 2 * ADDR_SIZE));
        assert_eq!(c.read_reg8(Reg8::VB), 1u8.overflowing_sub(0xFE).0);
        assert_eq!(c.read_reg8(Reg8::VF), 0);

        // Test Vf when Vx == Vy
        c.update_logic();
        assert_eq!(c.read_reg8(Reg8::VC), 0);
        assert_eq!(c.read_reg8(Reg8::VF), 0);
    }

    #[test]
    fn i8xy6_shr() {
        let mut c = Chip8::new();
        c.write_reg8(Reg8::VA, 0b10101001);
        c.load_rom(&[0x8A, 0xB6, 0x8A, 0xB6]);
        // Test registers
        c.update_logic();
        assert_eq!(c.read_reg16(Reg16::PC), (0x200 + ADDR_SIZE));
        assert_eq!(c.read_reg8(Reg8::VA), 0b01010100);
        assert_eq!(c.read_reg8(Reg8::VF), 1);
        c.update_logic();
        assert_eq!(c.read_reg8(Reg8::VA), 0b00101010);
        assert_eq!(c.read_reg8(Reg8::VF), 0);
    }

    #[test]
    fn i8xy7_subn() {
        let mut c = Chip8::new();
        c.write_reg8(Reg8::VA, 0x01);
        c.write_reg8(Reg8::VB, 0xFF);
        c.write_reg8(Reg8::VC, 0x01);
        c.write_reg8(Reg8::VD, 0x01);

        c.load_rom(&[0x8A, 0xB7, 0x8C, 0xD5]);
        // Test registers
        c.update_logic();
        assert_eq!(c.read_reg16(Reg16::PC), (0x200 + ADDR_SIZE));
        assert_eq!(c.read_reg8(Reg8::VA), 0xFE);
        assert_eq!(c.read_reg8(Reg8::VF), 1);
        // Test Vf when Vx == Vy
        c.update_logic();
        assert_eq!(c.read_reg8(Reg8::VC), 0);
        assert_eq!(c.read_reg8(Reg8::VF), 0);
    }

    #[test]
    fn i8xye_shl() {
        let mut c = Chip8::new();
        c.write_reg8(Reg8::VA, 0b10101010);
        c.load_rom(&[0x8A, 0xBE]);
        // Test registers
        c.update_logic();
        assert_eq!(c.read_reg16(Reg16::PC), (0x200 + ADDR_SIZE));
        assert_eq!(c.read_reg8(Reg8::VA), 0b01010100);
        assert_eq!(c.read_reg8(Reg8::VF), 1);
    }

    #[test]
    fn i9xy0_skip_neq() {
        let mut c = Chip8::new();
        c.write_reg8(Reg8::VA, 0xAA);
        c.write_reg8(Reg8::VB, 0xAA);
        c.write_reg8(Reg8::VC, 0xAB);

        c.load_rom(&[0x9A, 0xB0, 0x9A, 0xC0]);
        // Test registers
        c.update_logic();
        assert_eq!(c.read_reg16(Reg16::PC), (0x200 + ADDR_SIZE));
        c.update_logic();
        assert_eq!(c.read_reg16(Reg16::PC), (0x200 + 3 * ADDR_SIZE));
    }
    #[test]
    fn iannn_load_register_i() {
        let mut c = Chip8::new();
        c.load_rom(&[0xAF, 0xBE]);
        // Test registers
        c.update_logic();
        assert_eq!(c.read_reg16(Reg16::PC), (0x200 + ADDR_SIZE));
        assert_eq!(c.read_reg16(Reg16::I), 0xFBE);
    }

    #[test]
    fn ibnnn_jp_v0_plus_nnn() {
        let mut c = Chip8::new();
        c.write_reg8(Reg8::V0, 0x02);
        c.load_rom(&[0xB2, 0x08]);
        // Test registers
        c.update_logic();
        assert_eq!(c.read_reg16(Reg16::PC), 0x20A);
    }

    #[test]
    fn icxnn_vx_rand_and_nn() {
        let mut c = Chip8::new();
        c.write_reg8(Reg8::V0, 0x02);
        c.load_rom(&[0xC0, 0x08]);
        // Test registers
        c.update_logic();
        assert_eq!(c.read_reg16(Reg16::PC), 0x200 + ADDR_SIZE);
    }
}
