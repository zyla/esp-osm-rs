#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_closure)]
#![allow(clippy::upper_case_acronyms)]

extern crate alloc;
use core::future::Future;
use core::{alloc::GlobalAlloc, ops::ControlFlow, str};
use embassy_net::{
    dns::DnsSocket,
    tcp::client::{TcpClient, TcpClientState},
    Config, Stack, StackResources,
};
use embassy_time::{Duration, Timer};
use embedded_graphics::pixelcolor::{Rgb565, Rgb666};
use embedded_graphics::{
    pixelcolor::{Gray8, Rgb888},
    prelude::Dimensions,
};
use embedded_svc::wifi::{ClientConfiguration, Configuration, Wifi};
use esp_backtrace as _;
use esp_println::println;
use esp_wifi::wifi::{WifiController, WifiDevice, WifiEvent, WifiMode, WifiState};
use esp_wifi::{initialize, EspWifiInitFor};

#[cfg(feature = "esp32")]
pub use esp32_hal as hal;
#[cfg(feature = "esp32c3")]
pub use esp32c3_hal as hal;
#[cfg(feature = "esp32c6")]
pub use esp32c6_hal as hal;

#[cfg(any(feature = "esp32c3", feature = "esp32c6"))]
use hal::systimer::SystemTimer;
use hal::Rng;
use hal::{
    clock::ClockControl, embassy, gpio::*, peripherals::Peripherals, prelude::*, timer::TimerGroup,
    Rtc, IO,
};

use embedded_graphics::draw_target::DrawTarget;
use embedded_io_async::Read;
use incremental_png::{
    dechunker::Dechunker,
    inflater::{self, Inflater},
    stream_decoder::{Event, ImageHeader, StreamDecoder},
};
use reqwless::client::HttpClient;
use reqwless::request::Method;
use static_cell::make_static;

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

// Ideally we'd like to get rid of the allocator, but some dependency requires it and it's hard to
// find out which one.
// See
// <https://users.rust-lang.org/t/rust-no-std-find-why-global-memory-allocator-is-required/77679>
#[global_allocator]
static ALLOCATOR: FakeAllocator = FakeAllocator;

struct FakeAllocator;

unsafe impl GlobalAlloc for FakeAllocator {
    unsafe fn alloc(&self, _: core::alloc::Layout) -> *mut u8 {
        panic!("who needs the allocator?");
    }
    unsafe fn dealloc(&self, _: *mut u8, _: core::alloc::Layout) {
        panic!("who needs the allocator?");
    }
    unsafe fn alloc_zeroed(&self, _: core::alloc::Layout) -> *mut u8 {
        panic!("who needs the allocator?");
    }
    unsafe fn realloc(&self, _: *mut u8, _: core::alloc::Layout, _: usize) -> *mut u8 {
        panic!("who needs the allocator?");
    }
}

#[cfg(any(feature = "esp32c3", feature = "esp32c6"))]
type BUTTON = Gpio9<Input<PullUp>>;
#[cfg(feature = "esp32")]
type BUTTON = Gpio0<Input<PullUp>>;

mod display {
    use super::*;
    use display_interface::DisplayError;
    use display_interface_spi::SPIInterfaceNoCS;
    use embedded_graphics::pixelcolor::Rgb565;
    use embedded_graphics::prelude::RgbColor;
    use hal::peripherals::SPI2;
    use hal::spi::FullDuplexMode;
    use hal::Spi;
    use mipidsi::*;

    pub type SPI = Spi<'static, SPI2, FullDuplexMode>;
    pub type DISPLAY<'a> = Display<
        SPIInterfaceNoCS<SPI, GpioPin<Output<PushPull>, 2>>,
        models::ILI9341Rgb565,
        GpioPin<Output<PushPull>, 15>,
    >;

    pub type Color = Rgb565;
    pub const BACKGROUND: Color = Rgb565::BLACK;
    pub const TEXT: Color = Rgb565::RED;

    pub fn flush(_display: &mut DISPLAY) -> Result<(), ()> {
        // no-op
        Ok(())
    }

    pub fn set_pixel(
        display: &mut DISPLAY,
        x: u32,
        y: u32,
        color: Color,
    ) -> Result<(), DisplayError> {
        display.set_pixel(x as u16, y as u16, color)
    }
}

use display::DISPLAY;

#[embassy_macros::main_riscv(entry = "hal::entry")]
async fn main(spawner: embassy_executor::Spawner) {
    let peripherals = Peripherals::take();
    #[cfg(feature = "esp32c3")]
    let mut system = peripherals.SYSTEM.split();
    #[cfg(feature = "esp32c6")]
    let mut system = peripherals.PCR.split();
    #[cfg(feature = "esp32")]
    let mut system = peripherals.DPORT.split();

    let clocks = ClockControl::max(system.clock_control).freeze();

    #[cfg(any(feature = "esp32c3", feature = "esp32"))]
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    #[cfg(feature = "esp32c6")]
    let mut rtc = Rtc::new(peripherals.LP_CLKRST);

    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;

    // disable watchdog timers
    #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3))]
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    esp_println::logger::init_logger(log::LevelFilter::Info);

    let mut rng = Rng::new(peripherals.RNG);
    #[cfg(any(feature = "esp32c3", feature = "esp32c6"))]
    let timer = SystemTimer::new(peripherals.SYSTIMER).alarm0;
    #[cfg(feature = "esp32")]
    let timer = timer_group1.timer0;
    let init = initialize(
        EspWifiInitFor::Wifi,
        timer,
        rng,
        system.radio_clock_control,
        &clocks,
    )
    .unwrap();

    let (wifi, ..) = peripherals.RADIO.split();
    let (wifi_interface, controller) =
        esp_wifi::wifi::new_with_mode(&init, wifi, WifiMode::Sta).unwrap();

    embassy::init(&clocks, timer_group0.timer0);

    let dhcp4_config = embassy_net::DhcpConfig::default();
    let config = Config::dhcpv4(dhcp4_config);

    let seed = rng.random();

    let stack = &*make_static!(Stack::new(
        wifi_interface,
        config,
        make_static!(StackResources::<3>::new()),
        seed.into()
    ));

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    #[cfg(any(feature = "esp32c3", feature = "esp32c6"))]
    let input = io.pins.gpio9.into_pull_up_input();
    #[cfg(feature = "esp32")]
    let input = io.pins.gpio0.into_pull_up_input();

    hal::interrupt::enable(
        hal::peripherals::Interrupt::GPIO,
        hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    use display::*;
    let mut display: DISPLAY = {
        use display_interface_spi::SPIInterfaceNoCS;
        use hal::{spi::SpiMode, Delay, Spi};
        use mipidsi::*;

        // Define the Data/Command select pin as a digital output
        let dc = io.pins.gpio2.into_push_pull_output();

        let mut backlight = io.pins.gpio21.into_push_pull_output();
        backlight.set_high().unwrap();

        // Define the SPI pins and create the SPI interface
        let sck = io.pins.gpio14;
        let miso = io.pins.gpio12;
        let mosi = io.pins.gpio13;
        let cs = io.pins.gpio15;
        let spi = Spi::new(
            peripherals.SPI2,
            sck,
            mosi,
            miso,
            cs,
            80_u32.MHz(),
            SpiMode::Mode0,
            &mut system.peripheral_clock_control,
            &clocks,
        );

        // Define the display interface with no chip select
        let di = SPIInterfaceNoCS::new(spi, dc);

        let mut delay = Delay::new(&clocks);
        let display = Builder::ili9341_rgb565(di)
            .with_orientation(Orientation::Landscape(false))
            .init(&mut delay, None)
            .unwrap();

        display
    };

    display.clear(display::BACKGROUND).unwrap();
    display::flush(&mut display).unwrap();

    //   spawner.spawn(connection_wifi(controller)).ok();
    //   spawner.spawn(net_task(stack)).ok();
    spawner
        .spawn(task(input, stack, seed.into(), display, rtc))
        .ok();
}

#[embassy_executor::task]
async fn task(
    mut input: BUTTON,
    stack: &'static Stack<WifiDevice<'static>>,
    _seed: u64,
    mut display: DISPLAY<'static>,
    rtc: Rtc<'static>,
) {
    let png_bytes = include_bytes!("../test-tile.png");

    println!("num png bytes: {}", png_bytes.len());
    let start = rtc.get_time_us();
    let pixel_buffer = make_static!([0u8; 256 * 256 + 256]);
    let image_data = minipng::decode_png(png_bytes, pixel_buffer).unwrap();
    let end = rtc.get_time_us();
    println!("minipng decoded in {}us", end - start);

    loop {
        Timer::after(Duration::from_millis(0)).await;

        println!("render start");

        let start = rtc.get_time_us();
        display
            .set_pixels(
                0,
                0,
                255,
                239,
                image_data
                    .pixels()
                    .iter()
                    .map(|&index| {
                        let rgb = image_data.palette(index);
                        Rgb666::new(rgb[1], rgb[2], rgb[0]).into()
                    })
                    .take(256 * 240),
            )
            .unwrap();
        let end = rtc.get_time_us();
        println!("rendered in {}us", end - start);
    }
}

// TODO: something like this should be in `incremental-png` itself
struct PngReader {
    dechunker: Dechunker,
    sd: StreamDecoder,
    inflater: Inflater<16384>,
}

impl PngReader {
    fn new() -> Self {
        Self {
            dechunker: Dechunker::new(),
            sd: StreamDecoder::new(),
            inflater: Inflater::new(),
        }
    }

    async fn process_data<B, R>(
        &mut self,
        mut input: &[u8],
        mut block: impl FnMut(inflater::Event) -> R,
    ) -> ControlFlow<B>
    where
        R: Future<Output = ControlFlow<B>>,
    {
        while !input.is_empty() {
            let (consumed, mut dc_event) = self
                .dechunker
                .update(&input[..core::cmp::min(1024000, input.len())])
                .unwrap();

            while let Some(e) = dc_event {
                let (leftover, mut sd_event) = self.sd.update(e).unwrap();
                match sd_event {
                    None => println!("None"),
                    Some(Event::ImageHeader(_)) => println!("ImageHeader"),
                    Some(Event::ImageData(buf)) => println!("ImageData({})", buf.len()),
                    Some(Event::End) => println!("End"),
                }

                while let Some(e) = sd_event {
                    let (leftover, i_event) = self.inflater.update(e).unwrap();

                    if let Some(e) = i_event {
                        block(e).await?;
                    }

                    sd_event = leftover;
                }

                dc_event = leftover;
            }

            input = &input[consumed..];
        }
        ControlFlow::Continue(())
    }
}

#[embassy_executor::task]
async fn connection_wifi(mut controller: WifiController<'static>) {
    println!("Start connect with wifi (SSID: {:?}) task", SSID);
    loop {
        if matches!(esp_wifi::wifi::get_wifi_state(), WifiState::StaConnected) {
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
            Timer::after(Duration::from_millis(5000)).await
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = Configuration::Client(ClientConfiguration {
                ssid: SSID.into(),
                password: PASSWORD.into(),
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            println!("Starting wifi");
            controller.start().await.unwrap();
            println!("Wifi started!");
        }

        match controller.connect().await {
            Ok(_) => println!("Wifi connected!"),
            Err(e) => {
                println!("Failed to connect to wifi: {e:?}");
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<WifiDevice<'static>>) {
    stack.run().await
}
