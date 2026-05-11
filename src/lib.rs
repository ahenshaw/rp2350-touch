#![no_std]

use embedded_graphics::{pixelcolor::Rgb565, prelude::*, primitives::Rectangle};
use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin},
    i2c::I2c as I2cDevice,
};
use hal::{
    clocks::{init_clocks_and_plls, Clock},
    fugit::RateExtU32,
    gpio::{FunctionI2c, FunctionPio0, Pins, PullUp},
    pac,
    pio::{Buffers, PIOBuilder, PIOExt, PinDir, ShiftDirection, Tx},
    Sio, Timer, Watchdog, I2C,
};
use pio::pio_asm;
use rp235x_hal as hal;

// ── Display geometry ──────────────────────────────────────────────────────────

pub const W: u16 = 466;
pub const H: u16 = 466;
const X_OFFSET: u16 = 6;

// ── Framebuffer ───────────────────────────────────────────────────────────────

// 466×466×2 ≈ 424 KB; fits in the RP2350's 520 KB RAM.
static mut FRAMEBUF: [u16; W as usize * H as usize] = [0; W as usize * H as usize];

pub fn framebuf_mut() -> &'static mut [u16] {
    unsafe { &mut *core::ptr::addr_of_mut!(FRAMEBUF) }
}

type PioTx = Tx<(pac::PIO0, hal::pio::SM0)>;

// ── RNG ───────────────────────────────────────────────────────────────────────

pub struct Rng(u32);

impl Rng {
    pub fn new(seed: u32) -> Self {
        Self(seed)
    }

    pub fn next(&mut self) -> u32 {
        self.0 = self.0.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        self.0
    }

    pub fn range(&mut self, lo: u32, hi: u32) -> u32 {
        lo + self.next() % (hi - lo)
    }
}

// ── Touch controller (FT6146) ─────────────────────────────────────────────────

const FT6146_ADDR: u8 = 0x38;

pub struct Touch<I: I2cDevice, INT: InputPin, RST: OutputPin> {
    i2c: I,
    // Owned to prevent reuse; not read at runtime.
    _int: INT,
    _rst: RST,
}

impl<I: I2cDevice, INT: InputPin, RST: OutputPin> Touch<I, INT, RST> {
    fn new(mut i2c: I, int: INT, mut rst: RST, delay: &mut impl DelayNs) -> Self {
        rst.set_low().ok();
        delay.delay_ms(10);
        rst.set_high().ok();
        delay.delay_ms(50);
        // Switch to polling mode (register G_MODE = 0)
        i2c.write(FT6146_ADDR, &[0xA4, 0x00]).ok();
        Self { i2c, _int: int, _rst: rst }
    }

    /// Returns `Some((x, y))` when a finger is on the screen, `None` otherwise.
    pub fn read(&mut self) -> Option<(i32, i32)> {
        let mut buf = [0u8; 5];
        self.i2c.write_read(FT6146_ADDR, &[0x02], &mut buf).ok()?;
        if buf[0] & 0x0F == 0 {
            return None;
        }
        // buf[1] = XH: event[7:6], x_high[3:0]
        // buf[2] = XL: x_low[7:0]
        // buf[3] = YH: touch_id[7:4], y_high[3:0]
        // buf[4] = YL: y_low[7:0]
        let event = (buf[1] >> 6) & 0x03;
        if event == 1 {
            return None; // lift-up
        }
        let x = (((buf[1] & 0x0F) as i32) << 8) | buf[2] as i32;
        let y = (((buf[3] & 0x0F) as i32) << 8) | buf[4] as i32;
        Some((x, y))
    }
}

// ── Display driver ────────────────────────────────────────────────────────────

pub struct Display<CS: OutputPin, RST: OutputPin, PWR: OutputPin> {
    tx: PioTx,
    cs: CS,
    rst: RST,
    pwr: PWR,
}

/// Initialise all board hardware and return a display handle, touch handle, and a delay timer.
pub fn init() -> (
    Display<impl OutputPin + use<>, impl OutputPin + use<>, impl OutputPin + use<>>,
    Touch<impl I2cDevice + use<>, impl InputPin + use<>, impl OutputPin + use<>>,
    impl DelayNs + use<>,
) {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let clocks = init_clocks_and_plls(
        12_000_000u32, pac.XOSC, pac.CLOCKS, pac.PLL_SYS, pac.PLL_USB,
        &mut pac.RESETS, &mut watchdog,
    ).ok().unwrap();
    let mut timer = Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);
    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);

    // ── Touch ─────────────────────────────────────────────────────────────────
    let sda = pins.gpio6.reconfigure::<FunctionI2c, PullUp>();
    let scl = pins.gpio7.reconfigure::<FunctionI2c, PullUp>();
    let i2c = I2C::i2c1(pac.I2C1, sda, scl, 400u32.kHz(), &mut pac.RESETS, clocks.system_clock.freq());
    let touch_int = pins.gpio20.into_pull_up_input();
    let touch_rst = pins.gpio17.into_push_pull_output();
    let touch = Touch::new(i2c, touch_int, touch_rst, &mut timer);

    // ── Display ───────────────────────────────────────────────────────────────
    // Assign QSPI clock and data pins to PIO0; hardware retains the
    // function after the typed handles are dropped.
    let _ = (
        pins.gpio10.into_function::<FunctionPio0>(), // SCLK
        pins.gpio11.into_function::<FunctionPio0>(), // D0
        pins.gpio12.into_function::<FunctionPio0>(), // D1
        pins.gpio13.into_function::<FunctionPio0>(), // D2
        pins.gpio14.into_function::<FunctionPio0>(), // D3
    );
    let display = Display::new(
        pac.PIO0, &mut pac.RESETS,
        pins.gpio15.into_push_pull_output(), // CS
        pins.gpio16.into_push_pull_output(), // RST
        pins.gpio19.into_push_pull_output(), // PWR
    );

    (display, touch, timer)
}

impl<CS: OutputPin, RST: OutputPin, PWR: OutputPin> Display<CS, RST, PWR> {
    fn new(
        pio0: pac::PIO0,
        resets: &mut pac::RESETS,
        cs: CS,
        rst: RST,
        pwr: PWR,
    ) -> Self {
        let qspi_prog = pio_asm!(
            ".side_set 1",
            ".wrap_target",
            "out pins, 4   side 0",
            "nop           side 1",
            ".wrap",
        );

        let (mut pio, sm0, _, _, _) = pio0.split(resets);
        let installed = pio.install(&qspi_prog.program).unwrap();

        // 125 MHz ÷ (1 + 171/256) ≈ 74.9 MHz PIO → ~37.4 MHz SCLK × 4 wires.
        let (mut sm, _, tx) = PIOBuilder::from_installed_program(installed)
            .out_pins(11, 4)
            .side_set_pin_base(10)
            .out_shift_direction(ShiftDirection::Left)
            .autopull(true)
            .pull_threshold(32)
            .buffers(Buffers::OnlyTx)
            .clock_divisor_fixed_point(1, 171)
            .build(sm0);

        sm.set_pindirs([
            (10u8, PinDir::Output),
            (11u8, PinDir::Output),
            (12u8, PinDir::Output),
            (13u8, PinDir::Output),
            (14u8, PinDir::Output),
        ]);
        let _ = sm.start();

        let mut display = Self { tx, cs, rst, pwr };
        display.cs.set_high().ok();
        display.rst.set_high().ok();
        display
    }

    /// Power-on and hardware reset sequence.
    pub fn power_on_reset(&mut self, delay: &mut impl DelayNs) {
        self.pwr.set_high().ok();
        delay.delay_ms(10);
        self.rst.set_low().ok();
        delay.delay_ms(100);
        self.rst.set_high().ok();
        delay.delay_ms(200);
    }

    /// CO5300 initialisation sequence.
    pub fn init(&mut self, delay: &mut impl DelayNs) {
        self.cmd(0x11, &[]);
        delay.delay_ms(120);
        self.cmd(0xC4, &[0x80]);
        self.cmd(0x44, &[0x01, 0xD7]);
        self.cmd(0x35, &[0x00]);
        self.cmd(0x53, &[0x20]);
        delay.delay_ms(10);
        self.cmd(0x29, &[]);
        delay.delay_ms(10);
        self.cmd(0x51, &[0xFF]);
        self.cmd(0x20, &[]);
        self.cmd(0x36, &[0x00]);
        self.cmd(0x3A, &[0x05]);
    }

    /// Clear the framebuffer to black and push the full frame.
    pub fn clear(&mut self) {
        unsafe { &mut *core::ptr::addr_of_mut!(FRAMEBUF) }.fill(0);
        self.flush_rect(0, 0, W as usize - 1, H as usize - 1);
    }

    /// Push the region of the framebuffer covering `bbox` to the display.
    pub fn flush(&mut self, bbox: Rectangle) {
        if let Some((x0, y0, x1, y1)) = clip_align(bbox) {
            self.flush_rect(x0, y0, x1, y1);
        }
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    fn cmd(&mut self, cmd: u8, data: &[u8]) {
        self.cs.set_low().ok();
        for &b in &[0x02u8, 0x00, cmd, 0x00] {
            push_1bit(&mut self.tx, b);
        }
        for &b in data {
            push_1bit(&mut self.tx, b);
        }
        pio_flush(&mut self.tx);
        self.cs.set_high().ok();
    }

    fn flush_rect(&mut self, x0: usize, y0: usize, x1: usize, y1: usize) {
        let cx0 = x0 as u16 + X_OFFSET;
        let cx1 = x1 as u16 + X_OFFSET;
        let y0u = y0 as u16;
        let y1u = y1 as u16;
        self.cmd(0x2A, &[(cx0 >> 8) as u8, cx0 as u8, (cx1 >> 8) as u8, cx1 as u8]);
        self.cmd(0x2B, &[(y0u >> 8) as u8, y0u as u8, (y1u >> 8) as u8, y1u as u8]);

        self.cs.set_low().ok();
        for &b in &[0x32u8, 0x00, 0x2C, 0x00] {
            push_1bit(&mut self.tx, b);
        }
        pio_flush(&mut self.tx);

        let buf = unsafe { &*core::ptr::addr_of!(FRAMEBUF) };
        let w = x1 - x0 + 1;
        for y in y0..=y1 {
            let base = y * W as usize + x0;
            for chunk in buf[base..base + w].chunks_exact(2) {
                let word = (chunk[0] as u32) << 16 | chunk[1] as u32;
                while !self.tx.write(word) {}
            }
        }
        pio_flush(&mut self.tx);
        self.cs.set_high().ok();
    }
}

impl<CS: OutputPin, RST: OutputPin, PWR: OutputPin> DrawTarget for Display<CS, RST, PWR> {
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Rgb565>>,
    {
        let buf = unsafe { &mut *core::ptr::addr_of_mut!(FRAMEBUF) };
        for Pixel(pt, color) in pixels {
            if pt.x >= 0 && pt.x < W as i32 && pt.y >= 0 && pt.y < H as i32 {
                buf[pt.y as usize * W as usize + pt.x as usize] = ((color.r() as u16) << 11)
                    | ((color.g() as u16) << 5)
                    |  (color.b() as u16);
            }
        }
        Ok(())
    }
}

impl<CS: OutputPin, RST: OutputPin, PWR: OutputPin> OriginDimensions for Display<CS, RST, PWR> {
    fn size(&self) -> Size {
        Size::new(W as u32, H as u32)
    }
}

// ── PIO helpers ───────────────────────────────────────────────────────────────

#[inline(always)]
fn encode_1bit(b: u8) -> u32 {
    let b = b as u32;
    ((b & 0x80) << 21) | ((b & 0x40) << 18) | ((b & 0x20) << 15) | ((b & 0x10) << 12)
        | ((b & 0x08) << 9) | ((b & 0x04) << 6) | ((b & 0x02) << 3) | (b & 0x01)
}

fn pio_flush(tx: &mut PioTx) {
    while !tx.is_empty() {}
    cortex_m::asm::delay(200);
}

#[inline(always)]
fn push_1bit(tx: &mut PioTx, b: u8) {
    while !tx.write(encode_1bit(b)) {}
}

// ── Geometry ──────────────────────────────────────────────────────────────────

fn clip_align(bbox: Rectangle) -> Option<(usize, usize, usize, usize)> {
    let x0 = bbox.top_left.x.max(0);
    let y0 = bbox.top_left.y.max(0);
    let x1 = (bbox.top_left.x + bbox.size.width as i32 - 1).min(W as i32 - 1);
    let y1 = (bbox.top_left.y + bbox.size.height as i32 - 1).min(H as i32 - 1);
    if x0 > x1 || y0 > y1 {
        return None;
    }
    let mut x0 = x0 as usize;
    let mut x1 = x1 as usize;
    if (x1 - x0 + 1) & 1 != 0 {
        if x1 + 1 < W as usize {
            x1 += 1;
        } else {
            x0 = x0.saturating_sub(1);
        }
    }
    Some((x0, y0 as usize, x1, y1 as usize))
}
