#![no_std]
#![no_main]

#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: rp235x_hal::block::ImageDef = rp235x_hal::block::ImageDef::secure_exe();

use cortex_m_rt::entry;
use embedded_graphics::{
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Circle, PrimitiveStyle},
};
use panic_halt as _;
use rp2350_touch::{init, Rng, H, W};

#[entry]
fn main() -> ! {
    let (mut display, mut timer) = init();
    display.power_on_reset(&mut timer);
    display.init(&mut timer);
    display.clear();

    let mut rng = Rng::new(0xDEAD_BEEF);
    loop {
        let cx    = rng.range(0, W as u32) as i32;
        let cy    = rng.range(0, H as u32) as i32;
        let r     = rng.range(20, 120) as i32;
        let color = Rgb565::new(
            (rng.next() & 0x1F) as u8,
            (rng.next() & 0x3F) as u8,
            (rng.next() & 0x1F) as u8,
        );
        let circle = Circle::new(Point::new(cx - r, cy - r), (2 * r + 1) as u32);
        circle.into_styled(PrimitiveStyle::with_fill(color))
              .draw(&mut display).ok();
        display.flush(circle.bounding_box());
    }
}
