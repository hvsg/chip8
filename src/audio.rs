use cpal;
use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::sync::Arc;
pub struct Buzzer {
    host: cpal::Host,
    device: cpal::Device,
    active: Arc<AtomicBool>,
    stream: cpal::Stream,
}

impl Buzzer {
    pub fn new() -> Self {
        let host = cpal::default_host();
        let device = host
            .default_output_device()
            .expect("Could not get a default audio output device!");

        let mut supported_configs_range = device
            .supported_output_configs()
            .expect("Could not get any supported configs!");

        let config = device
            .default_output_config()
            .expect("Could not get default audio output config!");

        let mut active = Arc::new(AtomicBool::new(false));
        let mut stream_active = active.clone();

        println!("config: {:?}", config);
        let stream = match config.sample_format() {
            cpal::SampleFormat::F32 => start_stream::<f32>(&device, config, stream_active),
            cpal::SampleFormat::I16 => start_stream::<i16>(&device, config, stream_active),
            cpal::SampleFormat::U16 => start_stream::<u16>(&device, config, stream_active),
        };

        Buzzer {
            host,
            device,
            active,
            stream,
        }
    }
    pub fn set_active(&mut self, active: bool) {
        self.active.store(active, Ordering::SeqCst)
    }
}

fn start_stream<T: cpal::Sample>(
    device: &cpal::Device,
    config: cpal::SupportedStreamConfig,
    stream_active: Arc<AtomicBool>,
) -> cpal::Stream {
    let wave = generate_sine_wave(262, config.sample_rate().0);
    let mut offset = 0usize;
    let channels = config.channels();

    let stream = device
        .build_output_stream(
            &config.into(),
            move |data: &mut [T], _: &cpal::OutputCallbackInfo| {
                let multiplier = if stream_active.load(Ordering::SeqCst) {
                    1.0
                } else {
                    0.0
                };

                data.chunks_mut(channels as usize).for_each(|frame| {
                    let value = multiplier * wave[offset];
                    let converted = cpal::Sample::from(&value);
                    offset = (offset + 1) % wave.len();
                    for sample in frame.iter_mut() {
                        *sample = converted;
                    }
                });
            },
            move |err| eprintln!("Audio Output Stream Error: {}", err),
        )
        .expect("Failed to build output audio stream!");
    stream
}

/// Generates sine wave with samples for 1 second given sample rate and frequency
fn generate_sine_wave(frequency: u32, sample_rate: u32) -> Vec<f32> {
    let w = 2.0 * std::f32::consts::PI * frequency as f32;
    // let dt = 1.0/(sample_rate as f32);
    (0..sample_rate)
        .map(|i| f32::sin(w * i as f32 / sample_rate as f32))
        .collect()
}
