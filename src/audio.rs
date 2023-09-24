use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::thread::JoinHandle;
pub struct Buzzer {
    active: Arc<AtomicBool>,
    thread: JoinHandle<()>,
}

impl Buzzer {
    pub fn new(frequency: u32) -> Result<Self, Box<dyn std::error::Error>> {
        let active = Arc::new(AtomicBool::new(false));
        let stream_active = active.clone();

        let thread = std::thread::spawn(move || {
            let host = cpal::default_host();
            let device = host
                .default_output_device()
                .expect("Could not get a default audio output device!");

            let config = device
                .default_output_config()
                .expect("Could not get default audio output config!");

            println!("config: {:?}", config);
            println!("Starting audio stream!");
            let stream = match config.sample_format() {
                cpal::SampleFormat::F64 => start_stream::<f64>(&device, config, stream_active, frequency),
                cpal::SampleFormat::F32 => start_stream::<f32>(&device, config, stream_active, frequency),
                cpal::SampleFormat::I64 => start_stream::<i64>(&device, config, stream_active, frequency),
                cpal::SampleFormat::I32 => start_stream::<i32>(&device, config, stream_active, frequency),
                cpal::SampleFormat::I16 => start_stream::<i16>(&device, config, stream_active, frequency),
                cpal::SampleFormat::I8 => start_stream::<i8>(&device, config, stream_active, frequency),
                cpal::SampleFormat::U64 => start_stream::<u64>(&device, config, stream_active, frequency),
                cpal::SampleFormat::U32 => start_stream::<u32>(&device, config, stream_active, frequency),
                cpal::SampleFormat::U16 => start_stream::<u16>(&device, config, stream_active, frequency),
                cpal::SampleFormat::U8 => start_stream::<u8>(&device, config, stream_active, frequency),
                _ => {eprintln!("Unsupported audio format!"); return;}
            };
  
            stream.play();
            println!("Parking audio thread!");
            std::thread::park();
            println!("Terminating audio thread!");
        });

        Ok(Buzzer { active, thread })
    }
    pub fn set_active(&mut self, active: bool) {
        self.active.store(active, Ordering::SeqCst)
    }
}

impl Drop for Buzzer {
    fn drop(&mut self) {
        self.thread.thread().unpark();
    }
}

fn start_stream<T: cpal::SizedSample + cpal::FromSample<f32>>(
    device: &cpal::Device,
    config: cpal::SupportedStreamConfig,
    stream_active: Arc<AtomicBool>,
    frequency: u32,
) -> cpal::Stream {
    let wave = generate_sine_wave(frequency, config.sample_rate().0);
    let mut offset = 0usize;
    let channels = config.channels();

    device
        .build_output_stream(
            &config.into(),
            move |data: &mut [T], _: &cpal::OutputCallbackInfo| {
                let multiplier = if stream_active.load(Ordering::SeqCst) {
                    0.5
                } else {
                    0.0
                };

                // let multiplier = 0.5;

                data.chunks_mut(channels as usize).for_each(|frame| {
                    let value = multiplier * wave[offset];
                    // let converted = cpal::Sample::from(&value);
                    // let converted = value.to_sample::<T>();
                    // let converted = value;
                    let converted = T::from_sample(value);
                    offset = (offset + 1) % wave.len();
                    for sample in frame.iter_mut() {
                        *sample = converted;
                    }
                });
            },
            move |err| eprintln!("Audio Output Stream Error: {}", err),
        None
    )
        .expect("Failed to build output audio stream!")
}

/// Generates sine wave with samples for 1 second given sample rate and frequency
fn generate_sine_wave(frequency: u32, sample_rate: u32) -> Vec<f32> {
    let w = 2.0 * std::f32::consts::PI * frequency as f32;
    (0..sample_rate)
        .map(|i| f32::sin(w * i as f32 / sample_rate as f32))
        .collect()
}
