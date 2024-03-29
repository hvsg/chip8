use chip8::Chip8;
use winit::event::VirtualKeyCode;
mod audio;
use audio::Buzzer;

const LOGIC_DT: f64 = 1.0 / 500.0;
const TIMER_DT: f64 = 1.0 / 60.0;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Loading rom...");
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 2 {
        println!("Please provide a single argument as a path to the rom.");
        Err("No rom path!")?
    }
    let rom = std::fs::read(&args[1]).expect("Failed to read rom at path");

    println!("Initializing Chip-8!");
    let mut chip8 = Chip8::new();
    chip8.load_rom(rom.as_slice());
    let (w, h) = chip8.get_dimensions();
    let event_loop = winit::event_loop::EventLoop::new();
    let physical_size = winit::dpi::PhysicalSize::new(w as u32, h as u32);
    let window = winit::window::WindowBuilder::new()
        .with_title("Chip-8 Interpreter")
        .build(&event_loop)
        .expect("Failed to create window.");
    let scale_factor = 0.1;
    let logical_size = physical_size.to_logical::<f64>(scale_factor);
    window.set_inner_size(logical_size);

    let texture = pixels::SurfaceTexture::new(
        logical_size.width.round() as u32,
        logical_size.height.round() as u32,
        &window,
    );

    let mut pixels = pixels::PixelsBuilder::new(w as u32, h as u32, texture)
        .request_adapter_options(pixels::wgpu::RequestAdapterOptions {
            compatible_surface: None,
            power_preference: pixels::wgpu::PowerPreference::HighPerformance,
            force_fallback_adapter: false,
        })
        .build()
        .expect("Failed to create pixel buffer");

    let mut buzzer = Buzzer::new(1046)?;

    let mut time = std::time::Instant::now();
    let mut logic_accum = 0.0;
    let mut timer_accum = 0.0;

    event_loop.run(move |event, _, control_flow| match event {
        winit::event::Event::WindowEvent {
            window_id: _,
            event,
        } => match event {
            winit::event::WindowEvent::CloseRequested => {
                *control_flow = winit::event_loop::ControlFlow::Exit;
            }
            winit::event::WindowEvent::Resized(physical_size) => {
                if pixels
                    .resize_surface(physical_size.width, physical_size.height)
                    .is_err()
                {
                    *control_flow = winit::event_loop::ControlFlow::Exit;
                }
            }
            winit::event::WindowEvent::KeyboardInput {
                device_id: _,
                input,
                is_synthetic: _,
            } => {
                if let Some(keycode) = input.virtual_keycode {
                    let hex = match keycode {
                        VirtualKeyCode::Key0 | VirtualKeyCode::Numpad0 => 0x0,
                        VirtualKeyCode::Key1 | VirtualKeyCode::Numpad1 => 0x1,
                        VirtualKeyCode::Key2 | VirtualKeyCode::Numpad2 => 0x2,
                        VirtualKeyCode::Key3 | VirtualKeyCode::Numpad3 => 0x3,
                        VirtualKeyCode::Key4 | VirtualKeyCode::Numpad4 => 0x4,
                        VirtualKeyCode::Key5 | VirtualKeyCode::Numpad5 => 0x5,
                        VirtualKeyCode::Key6 | VirtualKeyCode::Numpad6 => 0x6,
                        VirtualKeyCode::Key7 | VirtualKeyCode::Numpad7 => 0x7,
                        VirtualKeyCode::Key8 | VirtualKeyCode::Numpad8 => 0x8,
                        VirtualKeyCode::Key9 | VirtualKeyCode::Numpad9 => 0x9,
                        VirtualKeyCode::A => 0xA,
                        VirtualKeyCode::B => 0xB,
                        VirtualKeyCode::C => 0xC,
                        VirtualKeyCode::D => 0xD,
                        VirtualKeyCode::E => 0xE,
                        VirtualKeyCode::F => 0xF,
                        VirtualKeyCode::R => {
                            chip8.reset();
                            chip8.load_rom(rom.as_slice());
                            0xFF
                        }
                        VirtualKeyCode::Escape => {
                            *control_flow = winit::event_loop::ControlFlow::Exit;
                            0xFF
                        }
                        _ => 0xFF,
                    };
                    if hex != 0xFF {
                        let pressed = input.state == winit::event::ElementState::Pressed;
                        chip8.read_input(hex, pressed);
                    }
                }
            }
            _ => {}
        },
        winit::event::Event::MainEventsCleared => {
            let current = std::time::Instant::now();
            let delta = (current - time).as_secs_f64();
            logic_accum += delta;
            timer_accum += delta;
            time = current;

            while logic_accum >= LOGIC_DT {
                chip8.update_logic();
                blit_ru8_to_rgbau8(chip8.get_screen_texture(), pixels.frame_mut());
                window.request_redraw();
                logic_accum -= LOGIC_DT;
            }
            buzzer.set_active(chip8.buzzer_active());
            while timer_accum >= TIMER_DT {
                chip8.update_timers();
                timer_accum -= TIMER_DT;
            }
        }
        winit::event::Event::RedrawRequested(_window_id) => {
            pixels.render().expect("Error rendering display.");
        }
        _ => {}
    });
}

fn blit_ru8_to_rgbau8(src: &[u8], dst: &mut [u8]) {
    assert_eq!(src.len() * 4, dst.len());
    let colors = [[0x38, 0x2B, 0x26], [0xB8, 0xC2, 0xB9]];
    src.iter().enumerate().for_each(|(i, pixel)| {
        let c = (0x1 & *pixel) as usize;
        dst[4 * i] = colors[c][0];
        dst[4 * i + 1] = colors[c][1];
        dst[4 * i + 2] = colors[c][2];
        dst[4 * i + 3] = u8::MAX;
    });
}
