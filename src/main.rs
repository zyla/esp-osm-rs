#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(iter_collect_into)]
#![feature(async_closure)]
#![feature(const_mut_refs)]
#![feature(generic_arg_infer)]
#![feature(cell_update)]
#![feature(const_trait_impl)]
#![allow(clippy::upper_case_acronyms)]
#![allow(non_camel_case_types)]
#![allow(unused)]
extern crate alloc;
use crate::hal::dma::DmaPriority;
use crate::hal::pdma::Dma;
use crate::hal::pdma::Spi2DmaChannelCreator;
use crate::tile_cache::TileData;
use crate::tile_cache::TileDescriptor;
use crate::tile_cache::TileId;
use crate::tile_cache::TILE_SIZE_BYTES;
use core::cell::Cell;
use core::fmt::Write;
use core::future::Future;
use core::{alloc::GlobalAlloc, ops::ControlFlow, str};
use embassy_net::{
    dns::DnsSocket,
    tcp::client::{TcpClient, TcpClientState},
    Config, Stack, StackResources,
};
use embassy_time::{Duration, Ticker, Timer};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::IntoStorage;
use embedded_graphics::prelude::Point;
use embedded_graphics::{
    pixelcolor::{Gray8, Rgb888},
    prelude::Dimensions,
};
use embedded_svc::wifi::{ClientConfiguration, Configuration, Wifi};
use esp_backtrace as _;
use esp_println::print;
use esp_println::println;
use esp_wifi::wifi::{WifiController, WifiDevice, WifiEvent, WifiMode, WifiState};
use esp_wifi::{initialize, EspWifiInitFor};
use incremental_png::Palette;

#[cfg(feature = "esp32")]
pub use esp32_hal as hal;
#[cfg(feature = "esp32c3")]
pub use esp32c3_hal as hal;
#[cfg(feature = "esp32c6")]
pub use esp32c6_hal as hal;

use hal::dma::Channel;
use hal::pdma::Spi2DmaChannel;
use hal::spi::SpiBusDevice;
#[cfg(any(feature = "esp32c3", feature = "esp32c6"))]
use hal::systimer::SystemTimer;
use hal::FlashSafeDma;
use hal::{
    clock::ClockControl, embassy, gpio::*, peripherals::Peripherals, prelude::*, timer::TimerGroup,
    Rtc, IO,
};
use hal::{peripherals, Delay, Rng};

use embedded_graphics::draw_target::DrawTarget;
use embedded_io_async::Read;
use incremental_png::{
    dechunker::Dechunker,
    inflater::{self, Inflater},
    stream_decoder::{Event, ImageHeader, StreamDecoder},
};
use mipidsi::dcs::SetColumnAddress;
use reqwless::client::HttpClient;
use reqwless::request::Method;
use smoltcp::wire::TcpControl;
use spin::barrier::BarrierWaitResult;
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

type TP_IRQ = Gpio36<Input<PullUp>>;

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
use tile_cache::TileCache;
use tile_cache::TileDataGuard;
use tile_cache::BYTES_PER_PIXEL;
use tile_cache::TILE_WIDTH;

mod touch {
    use super::*;
    use crate::hal::spi::FullDuplexMode;
    use crate::peripherals::SPI3;

    pub type CS = GpioPin<Output<PushPull>, 33>;
    pub type FAKE_CS = GpioPin<Output<PushPull>, 22>;
    pub type TOUCH =
        Xpt2046<SpiBusDevice<'static, 'static, SPI3, FAKE_CS, FullDuplexMode>, CS, MyIrq>;

    pub struct MyIrq(pub TP_IRQ);

    impl MyIrq {
        pub fn new(pin: TP_IRQ) {}
    }

    impl xpt2046::Xpt2046Exti for MyIrq {
        type Exti = ();

        fn clear_interrupt(&mut self) {
            self.0.clear_interrupt()
        }
        fn disable_interrupt(&mut self, exti: &mut Self::Exti) {
            hal::interrupt::disable(hal::get_core(), hal::peripherals::Interrupt::GPIO)
        }

        fn enable_interrupt(&mut self, exti: &mut Self::Exti) {
            hal::interrupt::enable(
                hal::peripherals::Interrupt::GPIO,
                hal::interrupt::Priority::Priority1,
            )
            .unwrap()
        }

        fn is_high(&self) -> bool {
            self.0.is_high().unwrap()
        }

        fn is_low(&self) -> bool {
            self.0.is_low().unwrap()
        }
    }
}

use touch::MyIrq;
use xpt2046::Xpt2046;

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
    let rtc = make_static!(Rtc::new(peripherals.RTC_CNTL));
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

    hal::interrupt::enable(
        hal::peripherals::Interrupt::SPI2_DMA,
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
            40_u32.MHz(),
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

    let dma = Dma::new(system.dma, &mut system.peripheral_clock_control);

    display.clear(display::BACKGROUND).unwrap();
    display::flush(&mut display).unwrap();

    use hal::spi::SpiBusController;
    use hal::{spi::SpiMode, Delay, Spi};
    use xpt2046::{Orientation, Xpt2046};

    let touch_irq = io.pins.gpio36.into_pull_up_input();

    // Define the SPI pins and create the SPI interface
    let sck = io.pins.gpio25;
    let miso = io.pins.gpio39;
    let mosi = io.pins.gpio32;
    let cs = io.pins.gpio33.into_push_pull_output();
    let fake_cs = io.pins.gpio22.into_push_pull_output();
    let spi = Spi::new_no_cs(
        peripherals.SPI3,
        sck,
        mosi,
        miso,
        2_u32.MHz(),
        SpiMode::Mode3,
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let mut delay = Delay::new(&clocks);

    let spi_bus_controller = make_static!(SpiBusController::from_spi(spi));

    let spi_2 = spi_bus_controller.add_device(fake_cs);

    let mut xpt_drv: touch::TOUCH =
        Xpt2046::new(spi_2, cs, MyIrq(touch_irq), Orientation::LandscapeFlipped);
    xpt_drv.init(&mut delay).unwrap();

    let touch_state = make_static!(TouchState::new());
    let view_state = make_static!(ViewState::new());

    spawner.spawn(on_touch(xpt_drv, touch_state)).ok();

    let delay = Delay::new(&clocks);

    #[cfg(feature = "psram")]
    hal::psram::init_psram(peripherals.PSRAM);

    let tile_cache = make_static!(init_tile_cache());

    spawner.spawn(connection_wifi(controller)).ok();
    spawner.spawn(net_task(stack)).ok();
    spawner
        .spawn(render_task(
            input,
            stack,
            seed.into(),
            display,
            delay,
            rtc,
            dma.spi2channel,
            tile_cache,
            touch_state,
            view_state,
        ))
        .ok();
    spawner
        .spawn(loader_task(stack, rtc, tile_cache, view_state))
        .ok();
}

struct ViewState {
    tile_needed: Cell<Option<TileId>>,
}

impl ViewState {
    pub fn new() -> Self {
        Self {
            tile_needed: Cell::new(None),
        }
    }
}

struct TouchState {
    drag_offset: Cell<Point>,
}

impl TouchState {
    pub fn new() -> Self {
        Self {
            drag_offset: Cell::new(Point::zero()),
        }
    }

    /// Get the current drag offset and zero it.
    /// The effect is that each call to take_drag_offset returns offset since the last call.
    pub fn take_drag_offset(&self) -> Point {
        self.drag_offset.replace(Point::zero())
    }

    pub fn add_drag_offset(&self, delta: Point) {
        self.drag_offset.update(|x| x + delta);
    }
}

#[embassy_executor::task]
async fn on_touch(mut xpt: touch::TOUCH, touch_state: &'static TouchState) {
    loop {
        xpt.irq.0.wait_for_low().await.unwrap();
        println!("touched!");
        let mut ticker = Ticker::every(Duration::from_millis(1));
        Timer::after_millis(1).await;
        let mut last_point = xpt.read_touch_point().unwrap();
        loop {
            ticker.next().await;
            let p = xpt.read_touch_point().unwrap();
            if !xpt.irq.0.is_low().unwrap() {
                break;
            }
            let delta = p - last_point;
            last_point = p;

            // Convert between embedded_graphics_core versions. Ugh
            let delta = Point::new(delta.x, -delta.y);
            if delta.x.abs() > 10 || delta.y.abs() > 10 {
                //println!("Ignoring big delta: {} {}", delta.x, delta.y);
                continue;
            }
            touch_state.add_drag_offset(delta);
        }
    }
}

static mut TILES_1: [tile_cache::TileData; 5] = tile_cache::empty();
#[link_section = ".dram2_uninit"]
static mut TILES_2: [tile_cache::TileData; 5] = tile_cache::empty();

type TILE_CACHE = TileCache<'static>;

#[cfg(feature = "psram")]
const NUM_PSRAM_TILES: usize =
    //    hal::psram::PSRAM_BYTES / core::mem::size_of::<tile_cache::TileData>()
    60;
#[cfg(not(feature = "psram"))]
const NUM_PSRAM_TILES: usize = 0;

fn init_tile_cache() -> TILE_CACHE {
    #[cfg(feature = "psram")]
    let psram_tiles = (0..NUM_PSRAM_TILES).map(|i| unsafe {
        &mut *((hal::psram::PSRAM_VADDR_START + i * core::mem::size_of::<TileData>())
            as *mut TileData)
    });

    #[cfg(not(feature = "psram"))]
    let psram_tiles = core::iter::empty();

    let descriptors: &mut heapless::Vec<
        tile_cache::TileDescriptor<'static>,
        { 5 + 5 + NUM_PSRAM_TILES },
    > = make_static!(heapless::Vec::from_iter(
        psram_tiles
            .chain(unsafe { &mut TILES_1 }.iter_mut())
            .chain(unsafe { &mut TILES_2 }.iter_mut())
            .map(TileDescriptor::new)
    ));

    TileCache::new(descriptors.as_slice())
}

#[embassy_executor::task]
async fn render_task(
    mut input: BUTTON,
    stack: &'static Stack<WifiDevice<'static>>,
    _seed: u64,
    display: DISPLAY<'static>,
    mut delay: Delay,
    rtc: &'static Rtc<'static>,
    dma_channel: Spi2DmaChannelCreator,
    tile_cache: &'static TILE_CACHE,
    touch_state: &'static TouchState,
    view_state: &'static ViewState,
) {
    let (di, _, _) = display.release();
    let (spi, dc) = di.release();

    let descriptors = make_static!([0u32; 8 * 3]);
    let rx_descriptors = make_static!([0u32; 8 * 3]);

    let spi = FlashSafeDma::<_, 8>::new(spi.with_dma(dma_channel.configure(
        false,
        descriptors,
        rx_descriptors,
        DmaPriority::Priority0,
    )));

    let local_tile_buffer: &'static mut TileData = make_static!([0; TILE_SIZE_BYTES]);

    let mut display = Display2::new(spi, dc, local_tile_buffer);

    let mut zoom_level = 0;
    let mut global_x: usize = 0;
    let mut global_y: usize = 0;

    zoom_level = 18;
    global_x = 146372 * 256;
    global_y = 86317 * 256;

    loop {
        // React to drag events
        let drag_offset = touch_state.take_drag_offset();
        global_x = (global_x as i32 - drag_offset.x) as usize;
        global_y = (global_y as i32 - drag_offset.y) as usize;

        let start = rtc.get_time_us();

        let first_tile_x = global_x / TILE_WIDTH;
        let first_tile_y = global_y / TILE_WIDTH;

        let mut total = 0;

        let mut tile_needed = None;

        for tile_x in first_tile_x..first_tile_x + DISPLAY_WIDTH / TILE_WIDTH + 2 {
            for tile_y in first_tile_y..first_tile_y + DISPLAY_HEIGHT / TILE_WIDTH + 2 {
                let tile_id = TileId {
                    zoom_level,
                    x: tile_x as u32,
                    y: tile_y as u32,
                };

                let target_x = -((global_x % TILE_WIDTH) as i32)
                    + ((tile_x - first_tile_x) * TILE_WIDTH) as i32;
                let target_y = -((global_y % TILE_WIDTH) as i32)
                    + ((tile_y - first_tile_y) * TILE_WIDTH) as i32;
                //               println!(
                //                   "Draw tile ({},{}) at ({}, {})",
                //                   tile_id.x, tile_id.y, target_x, target_y
                //               );

                if let Some(data) = tile_cache.lookup(tile_id).and_then(|td| td.try_lock_data()) {
                    // Note: can't deduplicate, lifetime problems
                    display
                        .draw_tile(&rtc, target_x, target_y, Some(*data))
                        .await;
                } else {
                    display.draw_tile(&rtc, target_x, target_y, None).await;
                    tile_needed = Some(tile_id);
                }
            }
        }

        view_state.tile_needed.replace(tile_needed);

        let end = rtc.get_time_us();
        print!("rendered in {}us\r", end - start);

        //        input.wait_for_rising_edge().await;
    }
}

const DISPLAY_WIDTH: usize = 320;
const DISPLAY_HEIGHT: usize = 240;

struct Display2<SPI, DC> {
    spi: SPI,
    dc: DC,
    local_tile_buffer: &'static mut TileData,
}
use crate::hal::prelude::_embedded_hal_async_spi_SpiBus as SpiBus;
use crate::hal::prelude::_embedded_hal_blocking_spi_Write as SpiWrite;

impl<SPI: SpiBus + SpiWrite<u8>, DC: _embedded_hal_digital_v2_OutputPin> Display2<SPI, DC>
where
    <SPI as SpiWrite<u8>>::Error: core::fmt::Debug,
    <DC as _embedded_hal_digital_v2_OutputPin>::Error: core::fmt::Debug,
{
    pub fn new(spi: SPI, dc: DC, local_tile_buffer: &'static mut TileData) -> Self {
        Self {
            spi,
            dc,
            local_tile_buffer,
        }
    }

    pub async fn draw_tile(
        &mut self,
        rtc: &Rtc<'static>,
        target_x: i32,
        target_y: i32,
        data: Option<&[u8]>,
    ) {
        // Clip the tile if needed
        let tile_offset_x = -core::cmp::min(target_x, 0);
        let tile_offset_y = -core::cmp::min(target_y, 0);
        let width = core::cmp::max(
            0,
            core::cmp::min(
                TILE_WIDTH as i32 - tile_offset_x,
                DISPLAY_WIDTH as i32 - target_x,
            ),
        );
        let height = core::cmp::max(
            0,
            core::cmp::min(
                TILE_WIDTH as i32 - tile_offset_y,
                DISPLAY_HEIGHT as i32 - target_y,
            ),
        );

        if width == 0 || height == 0 {
            // Tile is offscreen
            return;
        }

        // Clipping in X axis requires more DMA transfers, unfortunately
        let clipped_x = tile_offset_x > 0 || width < TILE_WIDTH as i32;

        let display_col_start: u16 = core::cmp::max(target_x, 0) as u16;
        let display_col_end: u16 = display_col_start + width as u16 - 1;
        let display_row_start: u16 = core::cmp::max(target_y, 0) as u16;
        let display_row_end: u16 = display_row_start + height as u16 - 1;

        let mut params = [0u8; 4];

        // Set Column Address
        self.dc.set_low().unwrap();
        SpiWrite::write(&mut self.spi, &[0x2A]).unwrap();
        self.dc.set_high().unwrap();
        params[0..2].copy_from_slice(&display_col_start.to_be_bytes());
        params[2..4].copy_from_slice(&display_col_end.to_be_bytes());
        SpiWrite::write(&mut self.spi, &params).unwrap();

        // Set Page Address
        self.dc.set_low().unwrap();
        SpiWrite::write(&mut self.spi, &[0x2B]).unwrap();
        self.dc.set_high().unwrap();
        params[0..2].copy_from_slice(&display_row_start.to_be_bytes());
        params[2..4].copy_from_slice(&display_row_end.to_be_bytes());
        SpiWrite::write(&mut self.spi, &params).unwrap();

        // SpiWrite Memory Start
        self.dc.set_low().unwrap();
        SpiWrite::write(&mut self.spi, &[0x2C]).unwrap();
        self.dc.set_high().unwrap();

        //        let start = rtc.get_time_us();

        //       println!("Set_Column_Address({display_col_start}, {display_col_end})");
        //       println!("Set_Page_Address({display_row_start}, {display_row_end})");
        //       println!("Tile rect: ({tile_offset_x}, {tile_offset_y}, {width}, {height})");
        if let Some(data) = data {
            let num_bytes = width as usize * height as usize * BYTES_PER_PIXEL;
            if !clipped_x {
                // Easy case: one transfer
                self.local_tile_buffer[..num_bytes].copy_from_slice(
                    &data[tile_offset_y as usize * TILE_WIDTH * BYTES_PER_PIXEL
                        ..(tile_offset_y + height) as usize * TILE_WIDTH * BYTES_PER_PIXEL],
                )
            } else {
                // Transfer each row separately

                let mut offset = 0;
                for y in tile_offset_y..tile_offset_y + height {
                    let off = y as usize * TILE_WIDTH * BYTES_PER_PIXEL
                        + tile_offset_x as usize * BYTES_PER_PIXEL;
                    let n = width as usize * BYTES_PER_PIXEL;
                    self.local_tile_buffer[offset..offset + n].copy_from_slice(&data[off..off + n]);
                    offset += n;
                }
            }
            if num_bytes >= MIN_ASYNC_TRANSFER_SIZE {
                SpiBus::write(&mut self.spi, &self.local_tile_buffer[..num_bytes])
                    .await
                    .unwrap();
            } else {
                SpiWrite::write(&mut self.spi, &self.local_tile_buffer[..num_bytes]).unwrap();
            }
        } else {
            // Drawing an empty tile
            let mut num_bytes = (width * height) as usize * BYTES_PER_PIXEL;
            while num_bytes > 0 {
                let n = core::cmp::min(num_bytes, ZEROS.len());
                if n >= MIN_ASYNC_TRANSFER_SIZE {
                    SpiBus::write(&mut self.spi, &ZEROS[..n]).await.unwrap();
                } else {
                    SpiWrite::write(&mut self.spi, &ZEROS[..n]).unwrap();
                }
                num_bytes -= n;
            }
        }
        //       let end = rtc.get_time_us();
        //       print!("transfer took {}us\n", end - start);
    }
}

const MIN_ASYNC_TRANSFER_SIZE: usize = 32;

#[link_section = ".bss"]
static ZEROS: [u8; 256] = [0; _];

/// Standard slippy map tile address - based on <https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames>
#[derive(PartialEq, Eq, Hash, Debug, Clone, Copy)]
pub struct BigTileId {
    pub zoom_level: u8,
    pub x: u32,
    pub y: u32,
}

struct TileDecoder<'a> {
    cache: &'a TileCache<'a>,
    tile_id: BigTileId,
    png: PngReader,
    current_row_tiles: [Option<TileDataGuard<'a>>; 4],
    x: usize,
    y: usize,
}

const BIG_TILE_WIDTH: usize = 256;

impl<'a> TileDecoder<'a> {
    fn new(cache: &'a TileCache<'a>, tile_id: BigTileId) -> Self {
        Self {
            cache,
            tile_id,
            png: PngReader::new(),
            current_row_tiles: Default::default(),
            x: 0,
            y: 0,
        }
    }

    fn finish(&mut self) {
        // Unlock tiles
        self.current_row_tiles = Default::default();
    }

    fn update_tiles(
        cache: &'a TileCache<'a>,
        current_row_tiles: &mut [Option<TileDataGuard<'a>>; 4],
        tile_id: BigTileId,
        y: usize,
    ) {
        for (i, g) in current_row_tiles.iter_mut().enumerate() {
            match cache.find_empty() {
                None => {
                    // No space in cache, skip the tile
                    // TODO: evict something
                    *g = None;
                    println!("No space in cache");
                }
                Some(td) => {
                    let tile_id = TileId {
                        zoom_level: tile_id.zoom_level,
                        x: tile_id.x * 4 + i as u32,
                        y: tile_id.y * 4 + y as u32 / tile_cache::TILE_WIDTH as u32,
                    };
                    println!("Saving {:?} in cache", tile_id);
                    *g = Some(td.try_lock_data().expect(
                        "Tile descriptor was supposed to be empty, data shouldn't be locked",
                    ));
                    td.set_id(tile_id);
                }
            }
        }
    }

    fn process_data(&mut self, data: &[u8]) {
        self.png.process_data::<()>(data, |palette, event| {
            match event {
                inflater::Event::ImageHeader(header) => {
                    println!("{:?}", header);
                    Self::update_tiles(
                        self.cache,
                        &mut self.current_row_tiles,
                        self.tile_id,
                        self.y,
                    );
                }
                inflater::Event::ImageData(pixels) => {
                    for &index in pixels {
                        let rgb = palette.color_at(index);
                        let value =
                            Rgb565::new(rgb[2] >> 3, rgb[1] >> 2, rgb[0] >> 3).into_storage();

                        let tile_x = self.x % tile_cache::TILE_WIDTH;
                        let tile_y = self.y % tile_cache::TILE_WIDTH;

                        if self.x != BIG_TILE_WIDTH {
                            if let Some(data) =
                                &mut self.current_row_tiles[self.x / tile_cache::TILE_WIDTH]
                            {
                                let offset =
                                    (tile_y * tile_cache::TILE_WIDTH + tile_x) * BYTES_PER_PIXEL;
                                data[offset..offset + 2].copy_from_slice(&value.to_be_bytes());
                            }
                        }

                        self.x += 1;
                        if self.x == BIG_TILE_WIDTH + 1 {
                            self.x = 0;
                            self.y += 1;
                            if self.y % tile_cache::TILE_WIDTH == 0 && self.y < BIG_TILE_WIDTH {
                                Self::update_tiles(
                                    self.cache,
                                    &mut self.current_row_tiles,
                                    self.tile_id,
                                    self.y,
                                );
                            }
                        }
                    }
                }
                inflater::Event::End => {}
            }
            ControlFlow::Continue(())
        });
    }
}

// TODO: something like this should be in `incremental-png` itself
struct PngReader {
    dechunker: Dechunker,
    sd: StreamDecoder,
    inflater: Inflater<1024>,
}

impl PngReader {
    fn new() -> Self {
        Self {
            dechunker: Dechunker::new(),
            sd: StreamDecoder::new(),
            inflater: Inflater::new(),
        }
    }

    fn process_data<B>(
        &mut self,
        mut input: &[u8],
        mut block: impl FnMut(&Palette, inflater::Event) -> ControlFlow<B>,
    ) -> ControlFlow<B> {
        while !input.is_empty() {
            let (consumed, mut dc_event) = self.dechunker.update(&input).unwrap();

            while let Some(e) = dc_event {
                let (leftover, mut sd_event) = self.sd.update(e).unwrap();
                match sd_event {
                    None => {}
                    Some(Event::ImageHeader(_)) => println!("ImageHeader"),
                    Some(Event::ImageData(buf)) => println!("ImageData({})", buf.len()),
                    Some(Event::End) => println!("End"),
                }

                while let Some(e) = sd_event {
                    let (leftover, i_event) = self.inflater.update(e).unwrap();

                    if let Some(e) = i_event {
                        block(self.sd.palette(), e)?;
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

mod tile_cache {
    use core::{marker::PhantomData, sync::atomic::AtomicBool};

    use spin::mutex::{SpinMutex, SpinMutexGuard};

    pub const BYTES_PER_PIXEL: usize = 2;
    pub const TILE_WIDTH: usize = 64;
    pub const TILE_SIZE_BYTES: usize = TILE_WIDTH * TILE_WIDTH * BYTES_PER_PIXEL;

    pub type TileData = [u8; TILE_SIZE_BYTES];

    pub const fn empty<const N: usize>() -> [TileData; N] {
        [[0u8; TILE_SIZE_BYTES]; N]
    }

    /// Tile addressing - based on <https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames>, but
    /// slightly modified.
    ///
    /// Zoom level is as in the original scheme. However, x and y a are 2 bits larger, since every
    /// standard tile contains 16 mini tiles.
    ///
    /// In the original scheme coordinates have max 18 bits. In our scheme - 20 bits.
    /// We could use a more efficient representation (48 bits are enough).
    #[derive(PartialEq, Eq, Hash, Debug, Clone, Copy)]
    pub struct TileId {
        pub zoom_level: u8,
        pub x: u32,
        pub y: u32,
    }

    pub struct TileDescriptor<'d> {
        /// Only lock for very short critical sections (to copy the id)
        id: SpinMutex<Option<TileId>>,
        /// Can lock for longer, but always use `try_lock`
        data: SpinMutex<&'d mut TileData>,
    }

    impl<'d> TileDescriptor<'d> {
        pub const fn new(data: &'d mut TileData) -> Self {
            Self {
                id: SpinMutex::new(None),
                data: SpinMutex::new(data),
            }
        }

        /// Note: acquires the ID spinlock
        pub fn get_id(&self) -> Option<TileId> {
            *self.id.lock()
        }

        /// Note: acquires the ID spinlock
        pub fn set_id(&self, id: TileId) {
            *self.id.lock() = Some(id);
        }

        pub fn try_lock_data(&'d self) -> Option<TileDataGuard<'d>> {
            self.data.try_lock()
        }
    }

    pub type TileDataGuard<'d> = SpinMutexGuard<'d, &'d mut TileData>;

    pub struct TileCache<'d> {
        descriptors: &'d [TileDescriptor<'d>],
    }

    impl<'d> TileCache<'d> {
        pub const fn new(ds: &'d [TileDescriptor<'d>]) -> Self {
            Self { descriptors: ds }
        }

        pub fn lookup(&self, id: TileId) -> Option<&TileDescriptor<'d>> {
            self.descriptors
                .as_ref()
                .iter()
                .find(|&d| d.get_id() == Some(id))
        }

        /// Returns any empty tile descriptor.
        ///
        /// (In the future we'll return not only empty ones, it will be an eviction policy)
        pub fn find_empty(&self) -> Option<&TileDescriptor<'d>> {
            self.descriptors
                .as_ref()
                .iter()
                .find(|&d| d.get_id().is_none())
        }
    }
}

#[embassy_executor::task]
async fn loader_task(
    stack: &'static Stack<WifiDevice<'static>>,
    rtc: &'static Rtc<'static>,
    tile_cache: &'static TILE_CACHE,
    view_state: &'static ViewState,
) {
    let mut rx_buffer = [0; 2048];
    let client_state = TcpClientState::<1, 2048, 2048>::new();
    let tcp_client = TcpClient::new(stack, &client_state);
    let dns = DnsSocket::new(stack);

    let mut ticker = Ticker::every(Duration::from_millis(100));
    loop {
        ticker.next().await;
        let Some(tile_id) = view_state.tile_needed.get() else {
            continue;
        };

        let big_tile_id = BigTileId {
            zoom_level: tile_id.zoom_level,
            x: tile_id.x / 4,
            y: tile_id.y / 4,
        };

        println!("\nLoading tile ({}, {})", big_tile_id.x, big_tile_id.y);

        stack.wait_config_up().await;
        loop {
            if let Some(config) = stack.config_v4() {
                println!("Got IP: {}", config.address);
                break;
            }
            Timer::after(Duration::from_millis(500)).await;
        }

        let start = rtc.get_time_us();

        let mut url = heapless::String::<128>::new();
        write!(
            url,
            "http://192.168.43.129:8081/tile/{}/{}/{}.png",
            big_tile_id.zoom_level, big_tile_id.x, big_tile_id.y
        );

        let mut http_client = HttpClient::new(&tcp_client, &dns);
        let mut request = http_client.request(Method::GET, "").await.unwrap();

        let response = request.send(&mut rx_buffer).await.unwrap();
        let mut td = TileDecoder::new(tile_cache, big_tile_id);
        let mut reader = response.body().reader();

        let mut buf = [0; 2048];
        let mut total_bytes_read = 0;
        loop {
            let n = reader.read(&mut buf).await.unwrap();
            if n == 0 {
                break;
            }
            total_bytes_read += n;
            println!("Received {} bytes (total {})", n, total_bytes_read);

            td.process_data(&buf[..n]);
        }
        td.finish();

        let end = rtc.get_time_us();
        println!("incremental-png decoded in {}us", end - start);
    }
}
