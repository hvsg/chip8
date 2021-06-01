use crate::to_general_reg8;
use crate::Chip8;
use crate::{Reg16, Reg8, ADDR_SIZE, MAXIMUM_STACK_ADDR, STACK_ADDR};

impl Chip8 {
    pub(crate) fn inst_cls(&mut self) {
        // CLS
        self.clear_screen();
        self.increment_pc();
    }
    pub(crate) fn inst_ret(&mut self) {
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
        self.increment_pc();
    }
    pub(crate) fn inst_jp(&mut self, i: u16) {
        // JP addr
        self.write_reg16(Reg16::PC, 0x0FFF & i);
    }
    pub(crate) fn inst_call(&mut self, i: u16) {
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
    }
    pub(crate) fn inst_skip_eq_const(&mut self, i: u16) {
        // Skip if register Vx equals lower byte
        let x = (i & 0x0F00) >> 8;
        let vx = to_general_reg8(x as usize);
        let lower: u8 = (0x00FF & i) as u8;
        // Skip instruction
        if self.read_reg8(vx) == lower {
            self.skip_next_instruction();
        } else {
            self.increment_pc();
        }
    }

    pub(crate) fn inst_skip_neq_const(&mut self, i: u16) {
        // Skip if register Vx neq to lower byte
        let x = (i & 0x0F00) >> 8;
        let vx = to_general_reg8(x as usize);
        let lower = (0x00FF & i) as u8;
        // Skip instruction
        if self.read_reg8(vx) != lower {
            self.skip_next_instruction();
        } else {
            self.increment_pc();
        }
    }
    pub(crate) fn inst_skip_eq(&mut self, i: u16) {
        // Skip if Vx == Vy
        let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
        let vy = to_general_reg8(((i & 0x00F0) >> 4) as usize);
        if self.read_reg8(vx) == self.read_reg8(vy) {
            self.skip_next_instruction();
        } else {
            self.increment_pc();
        }
    }

    pub(crate) fn inst_load_const(&mut self, i: u16) {
        // Put lower byte value into Vx
        let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
        let lower = (0x00FF & i) as u8;
        self.write_reg8(vx, lower);
        self.increment_pc();
    }

    pub(crate) fn inst_add_const(&mut self, i: u16) {
        // Vx += lower
        let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
        let lower = (0x00FF & i) as u8;
        let value = self.read_reg8(vx).overflowing_add(lower).0;
        self.write_reg8(vx, value);
        self.increment_pc();
    }

    pub(crate) fn inst_assign(&mut self, i: u16) {
        // Vx = Vy
        let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
        let vy = to_general_reg8(((i & 0x00F0) >> 4) as usize);
        self.write_reg8(vx, self.read_reg8(vy));
        self.increment_pc();
    }

    pub(crate) fn inst_or(&mut self, i: u16) {
        // Vx |= Vy
        let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
        let vy = to_general_reg8(((i & 0x00F0) >> 4) as usize);
        let value = self.read_reg8(vx) | self.read_reg8(vy);
        self.write_reg8(vx, value);
        self.increment_pc();
    }

    pub(crate) fn inst_and(&mut self, i: u16) {
        // Vx &= Vy
        let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
        let vy = to_general_reg8(((i & 0x00F0) >> 4) as usize);
        let value = self.read_reg8(vx) & self.read_reg8(vy);
        self.write_reg8(vx, value);
        self.increment_pc();
    }
    pub(crate) fn inst_xor(&mut self, i: u16) {
        // Vx ^= Vy
        let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
        let vy = to_general_reg8(((i & 0x00F0) >> 4) as usize);
        let value = self.read_reg8(vx) ^ self.read_reg8(vy);
        self.write_reg8(vx, value);
        self.increment_pc();
    }
    pub(crate) fn inst_add(&mut self, i: u16) {
        // Vx  += Vy and VF = carry
        let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
        let vy = to_general_reg8(((i & 0x00F0) >> 4) as usize);
        let (value, carry) = self.read_reg8(vx).overflowing_add(self.read_reg8(vy));
        self.write_reg8(vx, value);
        self.write_reg8(Reg8::VF, carry as u8);
        self.increment_pc();
    }
    pub(crate) fn inst_sub(&mut self, i: u16) {
        // VF = Vx > Vy (!borrow) and Vx -= Vy
        let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
        let vy = to_general_reg8(((i & 0x00F0) >> 4) as usize);
        let x = self.read_reg8(vx);
        let y = self.read_reg8(vy);
        let (value, _) = x.overflowing_sub(y);
        self.write_reg8(vx, value);
        self.write_reg8(Reg8::VF, (x > y) as u8);
        self.increment_pc();
    }

    pub(crate) fn inst_shr(&mut self, i: u16) {
        // VF = LSB(Vx) and Vx /= 2
        let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
        let x = self.read_reg8(vx);
        self.write_reg8(Reg8::VF, x & 0x1u8);
        self.write_reg8(vx, x >> 1);
        self.increment_pc();
    }
    pub(crate) fn inst_subn(&mut self, i: u16) {
        // VF = Vy > Vx, Vx = Vy - Vx
        let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
        let vy = to_general_reg8(((i & 0x00F0) >> 4) as usize);
        let x = self.read_reg8(vx);
        let y = self.read_reg8(vy);
        let (value, _) = y.overflowing_sub(x);
        self.write_reg8(vx, value);
        self.write_reg8(Reg8::VF, (y > x) as u8);
        self.increment_pc();
    }

    pub(crate) fn inst_shl(&mut self, i: u16) {
        // Vx = Vx << 1 and VF = overflow
        let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
        let x = self.read_reg8(vx);
        self.write_reg8(Reg8::VF, (x >> 7) & 0x1u8);
        self.write_reg8(vx, x << 1);
        self.increment_pc();
    }
    pub(crate) fn inst_skip_neq(&mut self, i: u16) {
        // Skip if Vx != Vy
        let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
        let vy = to_general_reg8(((i & 0x00F0) >> 4) as usize);
        if self.read_reg8(vx) != self.read_reg8(vy) {
            self.skip_next_instruction();
        } else {
            self.increment_pc();
        }
    }

    pub(crate) fn inst_set_i(&mut self, i: u16) {
        // I = value
        self.write_reg16(Reg16::I, i & 0x0FFF);
        self.increment_pc();
    }

    pub(crate) fn inst_jp_v0_value(&mut self, i: u16) {
        // JP V0 + value
        let v0 = self.read_reg8(Reg8::V0);
        self.write_reg16(Reg16::PC, (i & 0x0FFF) + v0 as u16);
    }

    pub(crate) fn inst_rand(&mut self, i: u16) {
        //  Vx = random & lower
        let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
        let nn = (i & 0x00FF) as u8;
        let r: u8 = rand::random();
        self.write_reg8(vx, r & nn);
        self.increment_pc();
    }

    pub(crate) fn inst_skip_pressed(&mut self, i: u16) {
        // Skip if Vx is pressed
        let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
        let value = self.read_reg8(vx);
        let kb = self.read_reg16(Reg16::KB);

        if kb & (1u16 << value) == (1u16 << value) {
            self.skip_next_instruction();
        } else {
            self.increment_pc();
        }
    }

    pub(crate) fn inst_skip_not_pressed(&mut self, i: u16) {
        // Skip if Vx is not pressed
        let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
        let value = self.read_reg8(vx);
        let kb = self.read_reg16(Reg16::KB);

        if kb & (1u16 << value) != (1u16 << value) {
            self.skip_next_instruction();
        } else {
            self.increment_pc();
        }
    }

    pub(crate) fn inst_read_dt(&mut self, i: u16) {
        // Vx = DT
        let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
        let dt = self.read_reg8(Reg8::DT);
        self.write_reg8(vx, dt);
        self.increment_pc();
    }

    pub(crate) fn inst_wait_for_key(&mut self, i: u16) {
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
        if value > 0 && ((0x1 << (value - 1)) & pkb) == 0 {
            let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
            self.write_reg8(vx, value - 1);
            self.increment_pc();
        }
    }

    pub(crate) fn inst_set_dt(&mut self, i: u16) {
        // DT = Vx
        let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
        self.write_reg8(Reg8::DT, self.read_reg8(vx));
        self.increment_pc();
    }

    pub(crate) fn inst_set_st(&mut self, i: u16) {
        // ST = Vx
        let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
        self.write_reg8(Reg8::ST, self.read_reg8(vx));
        self.increment_pc();
    }

    pub(crate) fn inst_add_vx_to_i(&mut self, i: u16) {
        // I += Vx
        let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
        let x = self.read_reg8(vx);
        let value = self.read_reg16(Reg16::I);
        self.write_reg16(Reg16::I, value.overflowing_add(x as u16).0);
        self.increment_pc();
    }

    pub(crate) fn inst_draw_sprite(&mut self, i: u16) {
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
        self.increment_pc();
    }

    pub(crate) fn inst_store_bcd(&mut self, i: u16) {
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
        self.increment_pc();
    }

    pub(crate) fn inst_store_registers(&mut self, i: u16) {
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
        self.increment_pc();
    }

    pub(crate) fn inst_load_registers(&mut self, i: u16) {
        // read memory from I into v0-vx
        let start = self.read_reg16(Reg16::I);
        let x = (i & 0x0F00) >> 8;
        for j in 0..=x {
            // Read jth value and store at register j
            let value = self.read8((start + j) as usize);
            let register = to_general_reg8(j as usize);
            self.write_reg8(register, value);
        }
        self.increment_pc();
    }

    pub(crate) fn inst_get_sprite_location(&mut self, i: u16) {
        // I = sprite for Vx
        let vx = to_general_reg8(((i & 0x0F00) >> 8) as usize);
        let digit = self.read_reg8(vx);
        let location = Self::get_digit_location(digit);
        self.write_reg16(Reg16::I, location);
        self.increment_pc();
    }
}
