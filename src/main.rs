
const MAX_ADDR:u16 = 0xFFF;
const MEMORY_SIZE: usize = MAX_ADDR as usize + 1;
struct Chip8 {
    // memory
    memory: [u8; MEMORY_SIZE]
}

impl Chip8 {
    fn new() -> Self {
        Self{
            memory: [0; MEMORY_SIZE],
        }
    }
}

fn main() {
    let mut chip8 = Chip8::new();
    println!("Hello, world!");
}