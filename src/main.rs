#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

extern crate alloc;
use alloc::boxed::Box;
use alloc::rc::Rc;

use display_interface_spi::SPIInterfaceNoCS;
use embedded_hal::digital::v2::OutputPin;
use esp32s3_hal::{
    clock::ClockControl,
    gpio::IO,
    pac::Peripherals,
    prelude::*,
    spi::{Spi, SpiMode},
    timer::TimerGroup,
    Delay, Rtc,
};
use esp_alloc::EspHeap;
use esp_backtrace as _;
use mipidsi::{Display, DisplayOptions, Orientation};
use xtensa_lx_rt::entry;

#[global_allocator]
static ALLOCATOR: EspHeap = EspHeap::empty();

#[alloc_error_handler]
fn oom(layout: core::alloc::Layout) -> ! {
    panic!("Out of memory {:?}", layout);
}

slint::include_modules!();

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let mut delay = Delay::new(&clocks);
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let sclk = io.pins.gpio7;
    let mosi = io.pins.gpio6;

    let spi = Spi::new_no_cs_no_miso(
        peripherals.SPI2,
        sclk,
        mosi,
        4u32.MHz(),
        SpiMode::Mode0,
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let dc = io.pins.gpio4.into_push_pull_output();
    let rst = io.pins.gpio48.into_push_pull_output();

    let di = SPIInterfaceNoCS::new(spi, dc);
    let mut display = Display::ili9342c_rgb565(di, rst);

    display
        .init(
            &mut delay,
            DisplayOptions {
                orientation: Orientation::PortraitInverted(false),
                ..DisplayOptions::default()
            },
        )
        .unwrap();

    let mut backlight = io.pins.gpio45.into_push_pull_output();
    backlight.set_high().unwrap();

    const HEAP_SIZE: usize = 256 * 1024;
    static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
    unsafe { ALLOCATOR.init(&mut HEAP as *mut u8, core::mem::size_of_val(&HEAP)) }

    let mut buffer_provider = DrawBuffer {
        display,
        buffer: &mut [slint::platform::swrenderer::Rgb565Pixel(0); 320],
    };

    let window = slint::platform::swrenderer::MinimalSoftwareWindow::new();

    window.set_size((320, 240).into());

    slint::platform::set_platform(Box::new(EspBackend {
        window: window.clone(),
    }))
    .unwrap();

    let app = App::new();
    app.show();

    loop {
        //slint::platform::update_timers_and_animations();

        window.draw_if_needed(|renderer| {
            renderer.render_by_line(&mut buffer_provider);
        });
        if window.has_active_animations() {
            continue;
        }

        // go to sleep
    }
}

struct EspBackend {
    window: Rc<slint::platform::swrenderer::MinimalSoftwareWindow<1>>,
}

impl slint::platform::Platform for EspBackend {
    fn create_window_adapter(&self) -> alloc::rc::Rc<dyn slint::re_exports::WindowAdapter> {
        self.window.clone()
    }
}

struct DrawBuffer<'a, Display> {
    display: Display,
    buffer: &'a mut [slint::platform::swrenderer::Rgb565Pixel],
}

impl<
        DI: display_interface::WriteOnlyDataCommand,
        RST: OutputPin<Error = core::convert::Infallible>,
        MODEL: mipidsi::models::Model<ColorFormat = embedded_graphics::pixelcolor::Rgb565>,
    > slint::platform::swrenderer::LineBufferProvider
    for &mut DrawBuffer<'_, Display<DI, RST, MODEL>>
{
    type TargetPixel = slint::platform::swrenderer::Rgb565Pixel;

    fn process_line(
        &mut self,
        line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [slint::platform::swrenderer::Rgb565Pixel]),
    ) {
        let buffer = &mut self.buffer[range.clone()];

        render_fn(buffer);

        // We send empty data just to get the device in the right window
        self.display
            .set_pixels(
                range.start as u16,
                line as _,
                range.end as u16,
                line as u16,
                buffer
                    .iter()
                    .map(|x| embedded_graphics::pixelcolor::raw::RawU16::new(x.0).into()),
            )
            .unwrap();
    }
}

#[no_mangle]
extern "C" fn rust_begin_unwind() {}

#[no_mangle]
extern "C" fn fmaxf(a: f32, b: f32) -> f32 {
    if a > b {
        a
    } else {
        b
    }
}
#[no_mangle]
extern "C" fn fminf(a: f32, b: f32) -> f32 {
    if a < b {
        a
    } else {
        b
    }
}
#[no_mangle]
extern "C" fn fmodf() {
    unimplemented!("fmodf");
}
#[no_mangle]
extern "C" fn fmod(a: f64, b: f64) -> f64 {
    ((a as u32) % (b as u32)) as f64
}
