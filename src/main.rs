#![no_std]
#![no_main]

#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: rp235x_hal::block::ImageDef = rp235x_hal::block::ImageDef::secure_exe();

use core::f32::consts::PI;
use cortex_m_rt::entry;
use embedded_graphics::{pixelcolor::Rgb565, prelude::*, primitives::Rectangle};
use embedded_hal::delay::DelayNs;
use micromath::F32Ext;
use panic_halt as _;
use rp235x_hal as hal;
use rp2350_touch::{battery_raw_to_pct, framebuf_mut, init, read_battery_raw, Display, Rng, H, W};

use embedded_hal::digital::OutputPin;

const CX: f32 = 233.0;
const CY: f32 = 233.0;
const ARENA_R: f32 = 215.0;

const BALL_R: i32 = 7;
const BALL_SPEED_INIT: f32 = 14.0;
const BALL_SPEED_MAX: f32 = 26.0;
const BALL_SPEED_INC: f32 = 0.5;

const PADDLE_THICK: u32 = 16;
const PADDLE_HALF_LEN: f32 = 50.0;
const GOAL_HALF: f32 = 0.78;

const PLAYER_BASE: f32 = PI * 0.5;
const COMPUTER_BASE: f32 = -PI * 0.5;
const DEFLECT_FACTOR: f32 = 1.5;

const RAW_BG: u16       = 0x0000;
const RAW_PLAYER: u16   = 0x07FF;
const RAW_COMPUTER: u16 = 0xF800;
const RAW_WALL: u16     = 0xFFFF;

const SEG: [u8; 10] = [63, 6, 91, 79, 102, 109, 125, 7, 127, 111];
const DIGIT_W: i32 = 26;
const DIGIT_H: i32 = 48;
const SCORE_GAP: i32 = 6;
const DASH_W: i32 = 14;
const SCORE_W: i32 = DIGIT_W + SCORE_GAP + DASH_W + SCORE_GAP + DIGIT_W;

// ── Paddle track ──────────────────────────────────────────────────────────────

struct PaddleTrack {
    mx: f32, my: f32,    // midpoint of the goal chord
    dx: f32, dy: f32,    // unit direction along the chord
    chord_half: f32,     // half the chord length
}

fn make_track(base: f32) -> PaddleTrack {
    let lx = CX + ARENA_R * (base - GOAL_HALF).cos();
    let ly = CY + ARENA_R * (base - GOAL_HALF).sin();
    let rx = CX + ARENA_R * (base + GOAL_HALF).cos();
    let ry = CY + ARENA_R * (base + GOAL_HALF).sin();
    let cdx = rx - lx;
    let cdy = ry - ly;
    let len = (cdx * cdx + cdy * cdy).sqrt();
    PaddleTrack {
        mx: (lx + rx) * 0.5,
        my: (ly + ry) * 0.5,
        dx: cdx / len,
        dy: cdy / len,
        chord_half: len * 0.5,
    }
}

// ── Geometry helpers ──────────────────────────────────────────────────────────

fn wrap(mut a: f32) -> f32 {
    while a > PI   { a -= 2.0 * PI; }
    while a <= -PI { a += 2.0 * PI; }
    a
}
fn angle_diff(a: f32, b: f32) -> f32 { wrap(a - b) }
fn in_zone(a: f32, center: f32, half: f32) -> bool { angle_diff(a, center).abs() <= half }

fn bb_union(a: Rectangle, b: Rectangle) -> Rectangle {
    let x0 = a.top_left.x.min(b.top_left.x);
    let y0 = a.top_left.y.min(b.top_left.y);
    let x1 = (a.top_left.x + a.size.width as i32).max(b.top_left.x + b.size.width as i32);
    let y1 = (a.top_left.y + a.size.height as i32).max(b.top_left.y + b.size.height as i32);
    Rectangle::new(Point::new(x0, y0), Size::new((x1 - x0) as u32, (y1 - y0) as u32))
}

fn paddle_dirty(track: &PaddleTrack, pos: f32, half_len: f32) -> Rectangle {
    let cx = track.mx + pos * track.dx;
    let cy = track.my + pos * track.dy;
    let margin = PADDLE_THICK as f32 / 2.0 + 5.0;
    let hl = half_len + margin;
    let ht = PADDLE_THICK as f32 / 2.0 + margin;
    // Paddle extends ±hl along chord and ±ht along the normal.
    let xspan = hl * track.dx.abs() + ht * track.dy.abs();
    let yspan = hl * track.dy.abs() + ht * track.dx.abs();
    Rectangle::new(
        Point::new((cx - xspan) as i32, (cy - yspan) as i32),
        Size::new((xspan * 2.0) as u32, (yspan * 2.0) as u32),
    )
}

// ── Framebuffer drawing ───────────────────────────────────────────────────────

fn fill_rect(buf: &mut [u16], x1: i32, y1: i32, x2: i32, y2: i32, color: u16) {
    let x1 = x1.max(0).min(W as i32);
    let x2 = x2.max(0).min(W as i32);
    let y1 = y1.max(0).min(H as i32);
    let y2 = y2.max(0).min(H as i32);
    for y in y1..y2 {
        for x in x1..x2 {
            buf[y as usize * W as usize + x as usize] = color;
        }
    }
}

fn fill_circle(buf: &mut [u16], cx: i32, cy: i32, r: i32, color: u16) {
    for y in (cy - r)..=(cy + r) {
        for x in (cx - r)..=(cx + r) {
            let dx = x - cx;
            let dy = y - cy;
            if dx * dx + dy * dy <= r * r {
                if (x as u32) < W as u32 && (y as u32) < H as u32 {
                    buf[y as usize * W as usize + x as usize] = color;
                }
            }
        }
    }
}

fn paint_paddle(buf: &mut [u16], track: &PaddleTrack, pos: f32, half_len: f32, thick: u32, color: u16) {
    let cx = track.mx + pos * track.dx;
    let cy = track.my + pos * track.dy;
    let x0 = cx - half_len * track.dx;
    let y0 = cy - half_len * track.dy;
    let x1 = cx + half_len * track.dx;
    let y1 = cy + half_len * track.dy;

    // Normal perpendicular to chord
    let nx = -track.dy;
    let ny =  track.dx;

    let n = (half_len * 2.0) as usize + 2;
    let half = thick as i32 / 2;
    for i in 0..=n {
        let t = i as f32 / n as f32;
        let px = x0 + t * (x1 - x0);
        let py = y0 + t * (y1 - y0);
        for d in -half..=half {
            let qx = (px + d as f32 * nx) as i32;
            let qy = (py + d as f32 * ny) as i32;
            if (qx as u32) < W as u32 && (qy as u32) < H as u32 {
                buf[qy as usize * W as usize + qx as usize] = color;
            }
        }
    }
}

fn paint_ring(buf: &mut [u16]) {
    let r_in  = (ARENA_R as i32) - 1;
    let r_out = (ARENA_R as i32) + 1;
    let r_in2  = r_in  * r_in;
    let r_out2 = r_out * r_out;
    let cx = CX as i32;
    let cy = CY as i32;
    for dy in -r_out..=r_out {
        for dx in -r_out..=r_out {
            let d2 = dx * dx + dy * dy;
            if d2 < r_in2 || d2 > r_out2 { continue; }
            let px = cx + dx;
            let py = cy + dy;
            if (px as u32) >= W as u32 || (py as u32) >= H as u32 { continue; }
            let angle = (dy as f32).atan2(dx as f32);
            if !in_zone(angle, PLAYER_BASE, GOAL_HALF) && !in_zone(angle, COMPUTER_BASE, GOAL_HALF) {
                buf[py as usize * W as usize + px as usize] = RAW_WALL;
            }
        }
    }
}

fn paint_circle_ring(buf: &mut [u16], cx: f32, cy: f32, r: f32, thick: i32, color: u16) {
    let half = thick / 2;
    let r_in  = (r as i32) - half;
    let r_out = (r as i32) + half;
    let r_in2  = r_in  * r_in;
    let r_out2 = r_out * r_out;
    let cxi = cx as i32;
    let cyi = cy as i32;
    for dy in -r_out..=r_out {
        for dx in -r_out..=r_out {
            let d2 = dx * dx + dy * dy;
            if d2 < r_in2 || d2 > r_out2 { continue; }
            let px = cxi + dx;
            let py = cyi + dy;
            if (px as u32) < W as u32 && (py as u32) < H as u32 {
                buf[py as usize * W as usize + px as usize] = color;
            }
        }
    }
}

fn draw_big_digit(buf: &mut [u16], x0: i32, y0: i32, digit: u8, color: u16) {
    let s = SEG[digit.min(9) as usize];
    let hh = DIGIT_H / 2;
    let t = 6_i32;
    if s & (1 << 0) != 0 { fill_rect(buf, x0+t/2,       y0,           x0+DIGIT_W-t/2, y0+t,            color); }
    if s & (1 << 1) != 0 { fill_rect(buf, x0+DIGIT_W-t, y0+t/2,       x0+DIGIT_W,     y0+hh,           color); }
    if s & (1 << 2) != 0 { fill_rect(buf, x0+DIGIT_W-t, y0+hh,        x0+DIGIT_W,     y0+DIGIT_H-t/2,  color); }
    if s & (1 << 3) != 0 { fill_rect(buf, x0+t/2,       y0+DIGIT_H-t, x0+DIGIT_W-t/2, y0+DIGIT_H,      color); }
    if s & (1 << 4) != 0 { fill_rect(buf, x0,           y0+hh,        x0+t,            y0+DIGIT_H-t/2,  color); }
    if s & (1 << 5) != 0 { fill_rect(buf, x0,           y0+t/2,       x0+t,            y0+hh,           color); }
    if s & (1 << 6) != 0 { fill_rect(buf, x0+t/2,       y0+hh-t/2,   x0+DIGIT_W-t/2, y0+hh+t/2,       color); }
}

fn draw_big_score(buf: &mut [u16], ps: u8, cs: u8) {
    let x0 = CX as i32 - SCORE_W / 2;
    let y0 = CY as i32 - DIGIT_H / 2;
    fill_rect(buf, x0 - 4, y0 - 4, x0 + SCORE_W + 4, y0 + DIGIT_H + 4, RAW_BG);
    draw_big_digit(buf, x0, y0, cs, RAW_COMPUTER);
    let dx = x0 + DIGIT_W + SCORE_GAP;
    fill_rect(buf, dx, y0 + DIGIT_H / 2 - 3, dx + DASH_W, y0 + DIGIT_H / 2 + 3, 0x7BEF);
    draw_big_digit(buf, x0 + DIGIT_W + SCORE_GAP + DASH_W + SCORE_GAP, y0, ps, RAW_PLAYER);
}

fn score_dirty() -> Rectangle {
    let x0 = CX as i32 - SCORE_W / 2 - 4;
    let y0 = CY as i32 - DIGIT_H / 2 - 4;
    Rectangle::new(Point::new(x0, y0), Size::new((SCORE_W + 8) as u32, (DIGIT_H + 8) as u32))
}

// ── Scale-up DrawTarget wrapper ───────────────────────────────────────────────

struct ScaleTarget<'a> {
    buf: &'a mut [u16],
    scale: i32,
    offset: Point,
}

impl DrawTarget for ScaleTarget<'_> {
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Rgb565>>,
    {
        for Pixel(pt, color) in pixels {
            let raw = ((color.r() as u16) << 11)
                | ((color.g() as u16) << 5)
                | (color.b() as u16);
            let bx = self.offset.x + pt.x * self.scale;
            let by = self.offset.y + pt.y * self.scale;
            fill_rect(self.buf, bx, by, bx + self.scale, by + self.scale, raw);
        }
        Ok(())
    }
}

impl OriginDimensions for ScaleTarget<'_> {
    fn size(&self) -> Size {
        Size::new(W as u32, H as u32)
    }
}

// ── End screen ────────────────────────────────────────────────────────────────

fn draw_end_screen(buf: &mut [u16], won: bool, player_score: u8, computer_score: u8) {
    use embedded_graphics::{
        mono_font::{ascii::FONT_10X20, MonoTextStyle},
        text::Text,
    };

    let color = if won { RAW_PLAYER } else { RAW_COMPUTER };

    // Score centered above face
    let score_y = 50_i32;
    let x0 = CX as i32 - SCORE_W / 2;
    draw_big_digit(buf, x0, score_y, computer_score, RAW_COMPUTER);
    let dx = x0 + DIGIT_W + SCORE_GAP;
    fill_rect(buf, dx, score_y + DIGIT_H / 2 - 3, dx + DASH_W, score_y + DIGIT_H / 2 + 3, 0x7BEF);
    draw_big_digit(buf, x0 + DIGIT_W + SCORE_GAP + DASH_W + SCORE_GAP, score_y, player_score, RAW_PLAYER);

    // Face centered on screen
    let face_cx = CX;
    let face_cy = CY;
    let face_r = 90.0_f32;
    paint_circle_ring(buf, face_cx, face_cy, face_r, 5, color);
    fill_circle(buf, (face_cx - 28.0) as i32, (face_cy - 25.0) as i32, 10, color);
    fill_circle(buf, (face_cx + 28.0) as i32, (face_cy - 25.0) as i32, 10, color);

    let mouth_base_y = (face_cy + 40.0) as i32;
    let mouth_w: i32 = 50;
    let amplitude = 22.0_f32;
    for xi in -mouth_w..=mouth_w {
        let t = xi as f32 / mouth_w as f32;
        let y_off = if won {
            (amplitude * (1.0 - t * t)) as i32
        } else {
            -(amplitude * (1.0 - t * t)) as i32
        };
        let px = CX as i32 + xi;
        let py = mouth_base_y + y_off;
        for dy in -3..=3_i32 {
            let py2 = py + dy;
            if (px as u32) < W as u32 && (py2 as u32) < H as u32 {
                buf[py2 as usize * W as usize + px as usize] = color;
            }
        }
    }

    // "You win!" text below face
    if won {
        let style = MonoTextStyle::new(&FONT_10X20, Rgb565::CYAN);
        let text_px_w = "You win!".len() as i32 * 10 * 3;
        let text_x = CX as i32 - text_px_w / 2;
        let mut target = ScaleTarget { buf, scale: 3, offset: Point::new(text_x, 385) };
        Text::new("You win!", Point::zero(), style).draw(&mut target).ok();
    }
}

// ── Physics ───────────────────────────────────────────────────────────────────

fn set_speed(vx: &mut f32, vy: &mut f32, speed: f32) {
    let mag = (*vx * *vx + *vy * *vy).sqrt();
    if mag > 1e-4 {
        *vx = *vx / mag * speed;
        *vy = *vy / mag * speed;
    }
}

fn bounce_off_paddle(
    vx: &mut f32, vy: &mut f32,
    nx: f32, ny: f32,
    delta_norm: f32,  // impact offset normalised to [-1, 1]
) -> bool {
    let dot = *vx * nx + *vy * ny;
    if dot <= 0.0 { return false; }
    *vx -= 2.0 * dot * nx;
    *vy -= 2.0 * dot * ny;
    let deflect = (delta_norm * DEFLECT_FACTOR).max(-0.87).min(0.87);
    let (c, s) = (deflect.cos(), deflect.sin());
    let (rvx, rvy) = (*vx, *vy);
    *vx = rvx * c - rvy * s;
    *vy = rvx * s + rvy * c;
    let d2 = *vx * nx + *vy * ny;
    if d2 > 0.0 { *vx -= d2 * nx; *vy -= d2 * ny; }
    true
}

fn reset_ball(bx: &mut f32, by: &mut f32, vx: &mut f32, vy: &mut f32,
              rng: &mut Rng, toward_player: bool) {
    *bx = CX; *by = CY;
    let vert_rad = rng.range(5, 40) as f32 / 180.0 * PI;
    let h = BALL_SPEED_INIT * vert_rad.sin();
    let v = BALL_SPEED_INIT * vert_rad.cos();
    *vx = if rng.range(0, 2) == 0 { h } else { -h };
    *vy = if toward_player { v } else { -v };
}

// ── Fireworks ─────────────────────────────────────────────────────────────────

#[derive(Copy, Clone)]
struct Particle {
    x: f32, y: f32,
    vx: f32, vy: f32,
    life: u8,
    max_life: u8,
    color: u16,
    active: bool,
}

const BLANK_P: Particle = Particle { x: 0.0, y: 0.0, vx: 0.0, vy: 0.0, life: 0, max_life: 1, color: 0, active: false };
const FW_COLORS: [u16; 6] = [0xFFFF, 0x07FF, 0xFFE0, 0xF81F, 0x07E0, 0xFD20];
const PARTICLE_R: i32 = 3;
const PARTICLE_HOLD: u8 = 15; // frames at full brightness before fading (~0.5 s at 33 ms/frame)

// ── Entry ─────────────────────────────────────────────────────────────────────

#[entry]
fn main() -> ! {
    let (mut display, mut touch, mut timer) = init();
    display.power_on_reset(&mut timer);
    display.init(&mut timer);
    display.clear();

    let mut rng = Rng::new(0xBEEF_CAFE);

    // ── Splash screen ─────────────────────────────────────────────────────────
    {
        let buf = framebuf_mut();
        let splash = include_bytes!("splash.raw");
        for (dst, src) in buf.iter_mut().zip(splash.chunks_exact(2)) {
            *dst = u16::from_be_bytes([src[0], src[1]]);
        }
        draw_battery(buf, battery_raw_to_pct(read_battery_raw()));
    }
    display.flush(Rectangle::new(Point::zero(), Size::new(W as u32, H as u32)));
    {
        let mut ms: u32 = 0;
        let mut idle_ms: u32 = 0;
        loop {
            timer.delay_ms(10u32);
            ms += 10;
            idle_ms += 10;
            if ms >= 1000 {
                ms = 0;
                let buf = framebuf_mut();
                draw_battery(buf, battery_raw_to_pct(read_battery_raw()));
                display.flush(battery_rect());
            }
            if idle_ms >= 30_000 { enter_dormant(&mut display); }
            if touch.read().is_some() { break; }
        }
    }
    loop { if touch.read().is_none() { break; } }

    // Track geometry is fixed by the arena constants — compute once.
    let p_track = make_track(PLAYER_BASE);
    let c_track = make_track(COMPUTER_BASE);

    'game: loop {
        // ── Difficulty selection ──────────────────────────────────────────────
        let (comp_spd, comp_half): (f32, f32) = {
            use embedded_graphics::{
                mono_font::{ascii::FONT_10X20, MonoTextStyle},
                text::Text,
            };
            let buf = framebuf_mut();
            buf.fill(RAW_BG);

            // Title
            // Buttons: cyan=Easy, yellow=Medium, red=Hard
            // Text offset = button_top + button_height/2 + baseline*scale/2 = top + 46
            let bw = 220_i32;
            let bx = CX as i32 - bw / 2;
            fill_rect(buf, bx,  60, bx + bw,  60 + 65, RAW_PLAYER);
            fill_rect(buf, bx, 200, bx + bw, 200 + 65, 0xFFE0);
            fill_rect(buf, bx, 340, bx + bw, 340 + 65, RAW_COMPUTER);

            {
                let style = MonoTextStyle::new(&FONT_10X20, Rgb565::BLACK);
                let x = CX as i32 - "EASY".len() as i32 * 10;
                let mut t = ScaleTarget { buf, scale: 2, offset: Point::new(x, 106) };
                Text::new("EASY", Point::zero(), style).draw(&mut t).ok();
            }
            {
                let style = MonoTextStyle::new(&FONT_10X20, Rgb565::BLACK);
                let x = CX as i32 - "MEDIUM".len() as i32 * 10;
                let mut t = ScaleTarget { buf, scale: 2, offset: Point::new(x, 246) };
                Text::new("MEDIUM", Point::zero(), style).draw(&mut t).ok();
            }
            {
                let style = MonoTextStyle::new(&FONT_10X20, Rgb565::BLACK);
                let x = CX as i32 - "HARD".len() as i32 * 10;
                let mut t = ScaleTarget { buf, scale: 2, offset: Point::new(x, 386) };
                Text::new("HARD", Point::zero(), style).draw(&mut t).ok();
            }

            display.flush(Rectangle::new(Point::zero(), Size::new(W as u32, H as u32)));

            // Sleep after 15 s of inactivity on the difficulty screen.
            let mut idle_ms: u32 = 0;
            let result = loop {
                if let Some((_tx, ty)) = touch.read() {
                    idle_ms = 0;
                    if ty >=  60 && ty < 125 { break (2.5_f32, 35.0_f32); }
                    if ty >= 200 && ty < 265 { break (4.5_f32, 50.0_f32); }
                    if ty >= 340 && ty < 405 { break (6.5_f32, 62.0_f32); }
                }
                timer.delay_ms(1u32);
                idle_ms += 1;
                if idle_ms >= 30_000 {
                    enter_dormant(&mut display);
                }
            };
            while touch.read().is_some() {}
            result
        };

        let mut player_pos: f32   = 0.0;  // signed px offset from track midpoint
        let mut computer_pos: f32 = 0.0;
        let mut ball_x = CX;
        let mut ball_y = CY;
        let mut ball_vx = 0.0_f32;
        let mut ball_vy = 0.0_f32;
        let mut ball_speed = BALL_SPEED_INIT;
        let mut player_score: u8   = 0;
        let mut computer_score: u8 = 0;

        let p_max = p_track.chord_half - PADDLE_HALF_LEN;
        let c_max = c_track.chord_half - comp_half;

        reset_ball(&mut ball_x, &mut ball_y, &mut ball_vx, &mut ball_vy, &mut rng, false);

        {
            let buf = framebuf_mut();
            buf.fill(RAW_BG);
            paint_ring(buf);
            paint_paddle(buf, &p_track, player_pos,   PADDLE_HALF_LEN, PADDLE_THICK, RAW_PLAYER);
            paint_paddle(buf, &c_track, computer_pos, comp_half, PADDLE_THICK, RAW_COMPUTER);
            draw_big_score(buf, player_score, computer_score);
            fill_circle(buf, ball_x as i32, ball_y as i32, BALL_R, 0xFFFF);
        }
        display.flush(Rectangle::new(Point::zero(), Size::new(W as u32, H as u32)));

        loop {
            let mut game_over: Option<bool> = None;
            let prev_player_pos   = player_pos;
            let prev_computer_pos = computer_pos;
            let prev_bxf = ball_x;   // f32 — needed for sub-pixel crossing detection
            let prev_byf = ball_y;
            let prev_bx = ball_x as i32;
            let prev_by = ball_y as i32;

            // ── Touch → player paddle ─────────────────────────────────────────
            if let Some((tx, ty)) = touch.read() {
                if (ty as f32) > CY {
                    let proj = (tx as f32 - p_track.mx) * p_track.dx
                             + (ty as f32 - p_track.my) * p_track.dy;
                    player_pos = proj.max(-p_max).min(p_max);
                }
            }

            // ── Computer AI ───────────────────────────────────────────────────
            if ball_vy < 0.0 {
                let ball_proj = (ball_x - c_track.mx) * c_track.dx
                              + (ball_y - c_track.my) * c_track.dy;
                let diff = ball_proj - computer_pos;
                let step = diff.signum() * diff.abs().min(comp_spd);
                computer_pos = (computer_pos + step).max(-c_max).min(c_max);
            }

            // ── Move ball ─────────────────────────────────────────────────────
            ball_x += ball_vx;
            ball_y += ball_vy;

            // ── Boundary collision ────────────────────────────────────────────
            //
            // For goal zones the collision surface is the flat track line, not
            // the arena wall.  Detect the frame the ball crosses each track by
            // checking the sign of its signed distance along the track's outward
            // normal: (track.dy, -track.dx).  Positive = in the goal zone.
            //
            // For the ring wall (non-goal), keep the radius check.

            let p_d_prev = (prev_bxf - p_track.mx) * p_track.dy
                         + (prev_byf - p_track.my) * (-p_track.dx);
            let p_d_curr = (ball_x   - p_track.mx) * p_track.dy
                         + (ball_y   - p_track.my) * (-p_track.dx);

            let c_d_prev = (prev_bxf - c_track.mx) * c_track.dy
                         + (prev_byf - c_track.my) * (-c_track.dx);
            let c_d_curr = (ball_x   - c_track.mx) * c_track.dy
                         + (ball_y   - c_track.my) * (-c_track.dx);

            // ── Player track crossing ─────────────────────────────────────────
            // Bounce fires at the front (arena-facing) face of the paddle.
            // Miss fires at the chord if the ball was not bounced.
            let half_thick = PADDLE_THICK as f32 / 2.0;
            let p_nx = p_track.dy;
            let p_ny = -p_track.dx;
            let p_bounced = if p_d_prev < -half_thick && p_d_curr >= -half_thick {
                let t = (p_d_prev + half_thick) / (p_d_prev - p_d_curr);
                let cross_x = prev_bxf + t * (ball_x - prev_bxf);
                let cross_y = prev_byf + t * (ball_y - prev_byf);
                let proj = (cross_x - p_track.mx) * p_track.dx
                         + (cross_y - p_track.my) * p_track.dy;
                if proj.abs() <= p_track.chord_half
                    && (proj - player_pos).abs() <= PADDLE_HALF_LEN
                {
                    let delta_norm = (proj - player_pos) / PADDLE_HALF_LEN;
                    if bounce_off_paddle(&mut ball_vx, &mut ball_vy, p_nx, p_ny, delta_norm) {
                        ball_x = cross_x - p_nx;
                        ball_y = cross_y - p_ny;
                        ball_speed = (ball_speed + BALL_SPEED_INC).min(BALL_SPEED_MAX);
                        set_speed(&mut ball_vx, &mut ball_vy, ball_speed);
                        true
                    } else { false }
                } else { false }
            } else { false };
            if !p_bounced && p_d_prev < 0.0 && p_d_curr >= 0.0 {
                let t = p_d_prev / (p_d_prev - p_d_curr);
                let cross_x = prev_bxf + t * (ball_x - prev_bxf);
                let cross_y = prev_byf + t * (ball_y - prev_byf);
                let proj = (cross_x - p_track.mx) * p_track.dx
                         + (cross_y - p_track.my) * p_track.dy;
                if proj.abs() <= p_track.chord_half
                    && (proj - player_pos).abs() > PADDLE_HALF_LEN
                {
                    computer_score += 1;
                    if computer_score >= 9 {
                        game_over = Some(false);
                    } else {
                        ball_speed = BALL_SPEED_INIT;
                        reset_ball(&mut ball_x, &mut ball_y, &mut ball_vx, &mut ball_vy, &mut rng, false);
                    }
                }
            }

            // ── Computer track crossing ───────────────────────────────────────
            let c_nx = c_track.dy;
            let c_ny = -c_track.dx;
            let c_bounced = if c_d_prev < -half_thick && c_d_curr >= -half_thick {
                let t = (c_d_prev + half_thick) / (c_d_prev - c_d_curr);
                let cross_x = prev_bxf + t * (ball_x - prev_bxf);
                let cross_y = prev_byf + t * (ball_y - prev_byf);
                let proj = (cross_x - c_track.mx) * c_track.dx
                         + (cross_y - c_track.my) * c_track.dy;
                if proj.abs() <= c_track.chord_half
                    && (proj - computer_pos).abs() <= comp_half
                {
                    let delta_norm = (proj - computer_pos) / comp_half;
                    if bounce_off_paddle(&mut ball_vx, &mut ball_vy, c_nx, c_ny, delta_norm) {
                        ball_x = cross_x - c_nx;
                        ball_y = cross_y - c_ny;
                        ball_speed = (ball_speed + BALL_SPEED_INC).min(BALL_SPEED_MAX);
                        set_speed(&mut ball_vx, &mut ball_vy, ball_speed);
                        true
                    } else { false }
                } else { false }
            } else { false };
            if !c_bounced && c_d_prev < 0.0 && c_d_curr >= 0.0 {
                let t = c_d_prev / (c_d_prev - c_d_curr);
                let cross_x = prev_bxf + t * (ball_x - prev_bxf);
                let cross_y = prev_byf + t * (ball_y - prev_byf);
                let proj = (cross_x - c_track.mx) * c_track.dx
                         + (cross_y - c_track.my) * c_track.dy;
                if proj.abs() <= c_track.chord_half
                    && (proj - computer_pos).abs() > comp_half
                {
                    player_score += 1;
                    if player_score >= 9 {
                        game_over = Some(true);
                    } else {
                        ball_speed = BALL_SPEED_INIT;
                        reset_ball(&mut ball_x, &mut ball_y, &mut ball_vx, &mut ball_vy, &mut rng, true);
                    }
                }
            }

            // ── Ring wall bounce (non-goal zone only) ─────────────────────────
            {
                let ddx = ball_x - CX;
                let ddy = ball_y - CY;
                let dist2 = ddx * ddx + ddy * ddy;
                let limit = ARENA_R - BALL_R as f32;
                if dist2 >= limit * limit {
                    let impact = ddy.atan2(ddx);
                    if !in_zone(impact, PLAYER_BASE, GOAL_HALF)
                        && !in_zone(impact, COMPUTER_BASE, GOAL_HALF)
                    {
                        let dist = dist2.sqrt();
                        let nx = ddx / dist;
                        let ny = ddy / dist;
                        ball_x = CX + nx * (limit - 1.0);
                        ball_y = CY + ny * (limit - 1.0);
                        let dot = ball_vx * nx + ball_vy * ny;
                        if dot > 0.0 {
                            ball_vx -= 2.0 * dot * nx;
                            ball_vy -= 2.0 * dot * ny;
                            set_speed(&mut ball_vx, &mut ball_vy, ball_speed);
                        }
                    }
                }
            }

            // ── Safety net: score if ball escaped the arena ───────────────────
            // Catches any edge case where the crossing detection was missed.
            if game_over.is_none() {
                let ex = ball_x - CX;
                let ey = ball_y - CY;
                if ex * ex + ey * ey > (ARENA_R + 30.0) * (ARENA_R + 30.0) {
                    let impact = ey.atan2(ex);
                    if in_zone(impact, PLAYER_BASE, GOAL_HALF) {
                        computer_score += 1;
                        if computer_score >= 9 { game_over = Some(false); }
                        else { ball_speed = BALL_SPEED_INIT; reset_ball(&mut ball_x, &mut ball_y, &mut ball_vx, &mut ball_vy, &mut rng, false); }
                    } else if in_zone(impact, COMPUTER_BASE, GOAL_HALF) {
                        player_score += 1;
                        if player_score >= 9 { game_over = Some(true); }
                        else { ball_speed = BALL_SPEED_INIT; reset_ball(&mut ball_x, &mut ball_y, &mut ball_vx, &mut ball_vy, &mut rng, true); }
                    } else {
                        reset_ball(&mut ball_x, &mut ball_y, &mut ball_vx, &mut ball_vy, &mut rng, true);
                    }
                }
            }

            // ── Framebuffer update ────────────────────────────────────────────
            let buf = framebuf_mut();

            paint_paddle(buf, &p_track, prev_player_pos,   PADDLE_HALF_LEN + 4.0,          PADDLE_THICK + 4, RAW_BG);
            paint_paddle(buf, &c_track, prev_computer_pos, comp_half + 4.0,  PADDLE_THICK + 4, RAW_BG);
            paint_paddle(buf, &p_track, player_pos,   PADDLE_HALF_LEN,          PADDLE_THICK, RAW_PLAYER);
            paint_paddle(buf, &c_track, computer_pos, comp_half, PADDLE_THICK, RAW_COMPUTER);

            fill_circle(buf, prev_bx, prev_by, BALL_R, RAW_BG);
            draw_big_score(buf, player_score, computer_score);
            paint_ring(buf);
            fill_circle(buf, ball_x as i32, ball_y as i32, BALL_R, 0xFFFF);

            // ── Flush dirty regions ───────────────────────────────────────────
            let p_dirty = bb_union(
                paddle_dirty(&p_track, prev_player_pos,   PADDLE_HALF_LEN + 4.0),
                paddle_dirty(&p_track, player_pos,        PADDLE_HALF_LEN),
            );
            let c_dirty = bb_union(
                paddle_dirty(&c_track, prev_computer_pos, comp_half + 4.0),
                paddle_dirty(&c_track, computer_pos,      comp_half),
            );
            let ball_d = (BALL_R * 2 + 6) as u32;
            let bm = BALL_R + 3;
            let ball_dirty = bb_union(
                Rectangle::new(Point::new(prev_bx - bm, prev_by - bm), Size::new(ball_d, ball_d)),
                Rectangle::new(Point::new(ball_x as i32 - bm, ball_y as i32 - bm), Size::new(ball_d, ball_d)),
            );
            display.flush(p_dirty);
            display.flush(c_dirty);
            display.flush(ball_dirty);
            display.flush(score_dirty());

            // ── Game over ─────────────────────────────────────────────────────
            if let Some(won) = game_over {
                {
                    let buf = framebuf_mut();
                    buf.fill(RAW_BG);
                    draw_end_screen(buf, won, player_score, computer_score);
                }
                display.flush(Rectangle::new(Point::zero(), Size::new(W as u32, H as u32)));
                if won {
                    // Fireworks until tap inside face
                    let mut parts = [BLANK_P; 80];
                    let mut spawn_cd: u8 = 0;
                    'fw: loop {
                        if let Some((tx, ty)) = touch.read() {
                            let dx = tx as f32 - CX;
                            let dy = ty as f32 - CY;
                            if dx * dx + dy * dy <= 90.0 * 90.0 {
                                while touch.read().is_some() {}
                                break 'fw;
                            }
                        }
                        let buf = framebuf_mut();
                        let mut bx0 = W as i32; let mut by0 = H as i32;
                        let mut bx1 = 0i32;     let mut by1 = 0i32;

                        // Erase old positions
                        for p in parts.iter() {
                            if !p.active { continue; }
                            let px = p.x as i32; let py = p.y as i32;
                            fill_circle(buf, px, py, PARTICLE_R, RAW_BG);
                            bx0 = bx0.min(px - PARTICLE_R); by0 = by0.min(py - PARTICLE_R);
                            bx1 = bx1.max(px + PARTICLE_R); by1 = by1.max(py + PARTICLE_R);
                        }

                        // Spawn a new burst every 25 frames
                        spawn_cd = spawn_cd.wrapping_add(1);
                        if spawn_cd >= 25 {
                            spawn_cd = 0;
                            let ang = rng.range(0, 628) as f32 / 100.0;
                            let rad = rng.range(20, 170) as f32;
                            let burst_x = CX + rad * ang.cos();
                            let burst_y = CY + rad * ang.sin();
                            let col = FW_COLORS[rng.range(0, 6) as usize];
                            let mut n = 0u8;
                            for p in parts.iter_mut() {
                                if !p.active && n < 18 {
                                    let dir = rng.range(0, 628) as f32 / 100.0;
                                    let spd = rng.range(15, 40) as f32 / 10.0;
                                    let life = (35 + rng.range(0, 20)) as u8;
                                    *p = Particle {
                                        x: burst_x, y: burst_y,
                                        vx: spd * dir.cos(), vy: spd * dir.sin(),
                                        life, max_life: life,
                                        color: col, active: true,
                                    };
                                    n += 1;
                                }
                            }
                        }

                        // Update physics and draw new positions
                        for p in parts.iter_mut() {
                            if !p.active { continue; }
                            p.x += p.vx; p.y += p.vy;
                            p.vy += 0.08;
                            p.vx *= 0.96; p.vy *= 0.96;
                            p.life -= 1;
                            if p.life == 0 { p.active = false; continue; }
                            let px = p.x as i32; let py = p.y as i32;
                            let draw_color = if p.life + PARTICLE_HOLD >= p.max_life {
                                p.color // hold period — full brightness
                            } else {
                                let fade = p.life as u32;
                                let fade_max = p.max_life.saturating_sub(PARTICLE_HOLD).max(1) as u32;
                                let r = (((p.color as u32 >> 11) & 0x1F) * fade / fade_max) as u16;
                                let g = (((p.color as u32 >>  5) & 0x3F) * fade / fade_max) as u16;
                                let b = (( p.color as u32        & 0x1F) * fade / fade_max) as u16;
                                (r << 11) | (g << 5) | b
                            };
                            fill_circle(buf, px, py, PARTICLE_R, draw_color);
                            bx0 = bx0.min(px - PARTICLE_R); by0 = by0.min(py - PARTICLE_R);
                            bx1 = bx1.max(px + PARTICLE_R); by1 = by1.max(py + PARTICLE_R);
                        }

                        if bx1 >= bx0 && by1 >= by0 {
                            display.flush(Rectangle::new(
                                Point::new(bx0, by0),
                                Size::new((bx1 - bx0 + 1) as u32, (by1 - by0 + 1) as u32),
                            ));
                        }
                        timer.delay_ms(33u32);
                    }
                } else {
                    // Lose: wait for tap inside face, or sleep after 30 s
                    let mut idle_ms: u32 = 0;
                    loop {
                        if let Some((tx, ty)) = touch.read() {
                            let dx = tx as f32 - CX;
                            let dy = ty as f32 - CY;
                            if dx * dx + dy * dy <= 90.0 * 90.0 { break; }
                        }
                        timer.delay_ms(1u32);
                        idle_ms += 1;
                        if idle_ms >= 30_000 { enter_dormant(&mut display); }
                    }
                    loop { if touch.read().is_none() { break; } }
                }
                continue 'game;
            }
        }
    }
}

fn battery_rect() -> Rectangle {
    // Matches the body+nub bounding box in draw_battery (BX=215, BY=422, BW+NW=36, BH=16)
    Rectangle::new(Point::new(215, 422), Size::new(36, 16))
}

fn draw_battery(buf: &mut [u16], pct: u8) {
    const BW: i32 = 32;
    const BH: i32 = 16;
    // Centre the body+nub (total width 36) at x=233, y=422.
    const BX: i32 = 233 - (BW + 4) / 2; // = 215
    const BY: i32 = 422;
    const NW: i32 = 4;
    const NH: i32 = 8;

    let color: u16 = if pct >= 50 { 0x07E0 }       // green
                     else if pct >= 20 { 0xFFE0 }   // yellow
                     else { 0xF800 };               // red

    // Terminal nub (right side, vertically centred)
    let ny = BY + (BH - NH) / 2;
    fill_rect(buf, BX + BW, ny, BX + BW + NW, ny + NH, 0xFFFF);

    // Body outline (1-px white border)
    fill_rect(buf, BX,          BY,          BX + BW,     BY + 1,      0xFFFF); // top
    fill_rect(buf, BX,          BY + BH - 1, BX + BW,     BY + BH,     0xFFFF); // bottom
    fill_rect(buf, BX,          BY,          BX + 1,      BY + BH,     0xFFFF); // left
    fill_rect(buf, BX + BW - 1, BY,          BX + BW,     BY + BH,     0xFFFF); // right

    // Coloured fill proportional to charge
    let fill_w = (pct as i32 * (BW - 2) + 50) / 100; // round
    if fill_w > 0 {
        fill_rect(buf, BX + 1, BY + 1, BX + 1 + fill_w, BY + BH - 1, color);
    }
    // Black out unfilled interior
    if fill_w < BW - 2 {
        fill_rect(buf, BX + 1 + fill_w, BY + 1, BX + BW - 1, BY + BH - 1, 0x0000);
    }
}

/// Turn off the display and enter RP2350 DORMANT mode.
/// Only a GPIO 15 falling edge (BOOT button → CS line) or RESET can wake the chip.
/// On wake the chip reboots cleanly, restarting at the splash screen.
fn enter_dormant<CS: OutputPin, RST: OutputPin, PWR: OutputPin>(
    display: &mut Display<CS, RST, PWR>,
) -> ! {
    use hal::{
        pac,
        reboot::{reboot, RebootArch, RebootKind},
        rosc::RingOscillator,
    };

    display.power_off();

    // Safety: PAC is stolen only here, after init() has finished.
    let p = unsafe { pac::Peripherals::steal() };

    // ── Configure dormant wake on GPIO 15 falling edge ────────────────────────
    // DORMANT_WAKE_INTE register 1 covers GPIOs 8-15.
    // GPIO 15 = gpio7 slot within that register.
    // Clear any latched edge first, then enable the interrupt.
    // INTR is W1C; GPIO 15 EDGE_LOW = bit 30 of register 1 ((15-8)*4 + 2).
    p.IO_BANK0.intr(1).write(|w| unsafe { w.bits(1 << 30) });
    p.IO_BANK0
        .dormant_wake_inte(1)
        .modify(|_, w| w.gpio7_edge_low().set_bit());

    // ── Switch clocks off PLL onto ROSC ───────────────────────────────────────
    // clk_sys → clk_ref (glitchless mux; bit 0 clear = CLK_REF)
    p.CLOCKS.clk_sys_ctrl().modify(|_, w| w.src().clk_ref());
    while p.CLOCKS.clk_sys_selected().read().bits() != 1 {}

    // clk_ref → ROSC phase-shifted output
    p.CLOCKS
        .clk_ref_ctrl()
        .modify(|_, w| w.src().rosc_clksrc_ph());
    while p.CLOCKS.clk_ref_selected().read().bits() != 1 {}

    // Power down both PLLs (all powerdown bits set)
    p.PLL_SYS
        .pwr()
        .write(|w| w.pd().set_bit().vcopd().set_bit().postdivpd().set_bit().dsmpd().set_bit());
    p.PLL_USB
        .pwr()
        .write(|w| w.pd().set_bit().vcopd().set_bit().postdivpd().set_bit().dsmpd().set_bit());

    // ── Enter ROSC dormant ────────────────────────────────────────────────────
    // The chip halts here; execution resumes after a GPIO 15 edge wake event.
    let rosc = RingOscillator::new(p.ROSC);
    let rosc = rosc.initialize();
    unsafe { rosc.dormant() };

    // Woke up — reboot so firmware restarts from the top (splash screen).
    reboot(RebootKind::Normal, RebootArch::Normal)
}
