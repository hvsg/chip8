use chip8::Chip8;
use pixels;
use winit;
use winit::event::VirtualKeyCode;

fn blit_ru8_to_rgbau8(src: &[u8], dst: &mut [u8]) {
    assert_eq!(src.len() * 4, dst.len());
    src.iter().enumerate().for_each(|(i, pixel)| {
        dst[4 * i] = *pixel;
        dst[4 * i + 1] = *pixel;
        dst[4 * i + 2] = *pixel;
        dst[4 * i + 3] = u8::MAX;
    });
}
fn main() {
    println!("Loading rom...");
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 2 {
        println!("Please provide a single argument as a path to the rom.");
        return;
    }
    let rom = std::fs::read(&args[1]).expect("Failed to read rom at path");

    println!("Initializing Chip-8!");
    let mut chip8 = Chip8::new();
    chip8.load_rom(rom.as_slice());
    let event_loop = winit::event_loop::EventLoop::new();
    let physical_size =
        winit::dpi::PhysicalSize::new(chip8::SCREEN_WIDTH as u32, chip8::SCREEN_HEIGHT as u32);
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
    let mut pixels = pixels::PixelsBuilder::new(
        chip8::SCREEN_WIDTH as u32,
        chip8::SCREEN_HEIGHT as u32,
        texture,
    )
    .request_adapter_options(pixels::wgpu::RequestAdapterOptions {
        compatible_surface: None,
        power_preference: pixels::wgpu::PowerPreference::HighPerformance,
    })
    .build()
    .expect("Failed to create pixel buffer");

    event_loop.run(move |event, target, control_flow| match event {
        winit::event::Event::WindowEvent { window_id, event } => match event {
            winit::event::WindowEvent::CloseRequested => {
                *control_flow = winit::event_loop::ControlFlow::Exit;
            }
            winit::event::WindowEvent::Resized(physical_size) => {
                pixels.resize_surface(physical_size.width, physical_size.height);
            }
            winit::event::WindowEvent::KeyboardInput {
                device_id,
                input,
                is_synthetic,
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
            window.request_redraw();
            chip8.update();
            blit_ru8_to_rgbau8(chip8.get_screen_texture(), pixels.get_frame());
        }
        winit::event::Event::RedrawRequested(window_id) => {
            pixels.render().expect("Error rendering display.");
        }
        _ => {}
    });
}
