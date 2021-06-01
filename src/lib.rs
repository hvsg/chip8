mod instructions;
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

fn invalid(i: u16) {
    panic!("Chip-8: Invalid instruction {:X}", i);
}

/// Convert unsize to general purpose register (V0-VF)
pub fn to_general_reg8(x: usize) -> Reg8 {
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

    /// Increments Program Counter once
    pub fn increment_pc(&mut self) {
        let pc = self.read_reg16(Reg16::PC);
        self.write_reg16(Reg16::PC, pc + ADDR_SIZE);
    }

    /// Increments Program Counter twice
    pub fn skip_next_instruction(&mut self) {
        let pc = self.read_reg16(Reg16::PC);
        self.write_reg16(Reg16::PC, pc + 2 * ADDR_SIZE);
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
        match (i & 0xF000) >> 12 {
            0 => match i {
                0x00E0 | 0x230 => self.inst_cls(i),
                0x00EE => self.inst_ret(i),
                _ => invalid(i),
            },
            1 => self.inst_jp(i),
            2 => self.inst_call(i),
            3 => self.inst_skip_eq_const(i),
            4 => self.inst_skip_neq_const(i),
            5 => match i & 0xF {
                0 => self.inst_skip_eq(i),
                _ => invalid(i),
            },
            6 => self.inst_load_const(i),
            7 => self.inst_add_const(i),
            8 => match i & 0xF {
                0 => self.inst_assign(i),
                1 => self.inst_or(i),
                2 => self.inst_and(i),
                3 => self.inst_xor(i),
                4 => self.inst_add(i),
                5 => self.inst_sub(i),
                6 => self.inst_shr(i),
                7 => self.inst_subn(i),
                0xE => self.inst_shl(i),
                _ => invalid(i),
            },
            9 => match i & 0xF {
                0 => self.inst_skip_neq(i),
                _ => invalid(i),
            },
            0xA => self.inst_set_i(i),
            0xB => self.inst_jp_v0_value(i),
            0xC => self.inst_rand(i),
            0xD => self.inst_draw_sprite(i),
            0xE => match i & 0xFF {
                0x9E => self.inst_skip_pressed(i),
                0xA1 => self.inst_skip_not_pressed(i),
                _ => invalid(i),
            },
            0xF => match i & 0xFF {
                0x07 => self.inst_read_dt(i),
                0x0A => self.inst_wait_for_key(i),
                0x15 => self.inst_set_dt(i),
                0x18 => self.inst_set_st(i),
                0x1E => self.inst_add_vx_to_i(i),
                0x29 => self.inst_get_sprite_location(i),
                0x33 => self.inst_store_bcd(i),
                0x55 => self.inst_store_registers(i),
                0x65 => self.inst_load_registers(i),
                _ => invalid(i),
            },

            _ => panic!("Chip-8: Invalid instruction {:X}", i),
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
