use crate::to_general_reg8;
use crate::Chip8;
use crate::{Reg16, Reg8};

impl Chip8 {
    // pub(in crate) fn do_something(&self) {println!("hi!");}
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
    }
}
