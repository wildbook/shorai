use shorai::{
    geometry::Line,
    missile::{Missile, MissileSet},
    pathfind,
    pos::Pos,
    FxIndexMap,
};
use std::{iter::once, time::Instant};
use structopt::StructOpt;

use image::{GenericImage, Rgba, RgbaImage};
use palette::{Hue, IntoColor, Lch, Srgb};
use rand::{prelude::StdRng, SeedableRng};
use rayon::iter::{ParallelBridge, ParallelIterator};
use ultraviolet::Vec2;

static SIZE: Pos = Pos::new(2000.0, 2000.0, 0.0);

#[derive(Debug, StructOpt)]
#[structopt(name = "shorai")]
struct Args {
    #[structopt(long)]
    seed: Option<u64>,

    #[structopt(long)]
    missiles: u32,

    #[structopt(long, default_value = "0.4")]
    render_scale: f32,

    #[structopt(long, default_value = "0.05")]
    render_step: f32,

    #[structopt(long)]
    render_smear: Option<f32>,

    #[structopt(long, default_value = "10.0")]
    max_time: f32,

    #[structopt(long, default_value = "10000", help = "Custom limit to prevent deadlocks")]
    max_steps: usize,

    #[structopt(long, default_value = "50.0", help = "Size of each 'grid cell'")]
    step_size: f32,

    #[structopt(long, default_value = "55.0", help = "Player radius")]
    pawn_size: f32,

    #[structopt(long, default_value = "325.0", help = "Player movement speed")]
    move_speed: f32,
}

fn main() {
    let args = Args::from_args();
    let curr_time = 0.0; // Current time

    let Args { render_scale, render_step, move_speed, pawn_size, step_size, max_time, max_steps, .. } = args;
    let render_smear = args.render_smear.unwrap_or(render_step);

    let origin = Pos::new(0.0, 0.0, curr_time);
    let target = Pos::new(SIZE.x.0, SIZE.y.0, max_time);

    let seed = args.seed.unwrap_or_else(rand::random);

    println!("seed: {}", seed);
    let random = &mut StdRng::seed_from_u64(seed);

    let step_time = step_size / move_speed;

    println!("step size: {} units", step_size);
    println!("step time: {} s / unit", step_time);

    let mut missiles = FxIndexMap::default();
    for i in 0..args.missiles {
        let missile = Missile::random(
            random,
            -100.0..SIZE.x(),
            -100.0..SIZE.y(),
            60.0..120.0,
            300.0..1000.0,
            curr_time..max_time,
        );

        missiles.insert(i, missile);
    }

    let mis = MissileSet(missiles);

    println!("searching");
    let mut steps = max_steps;

    let now = Instant::now();

    let result = pathfind::find(
        origin,
        |pos| pos.successors(&mis, step_time, step_size, pawn_size),
        // LoS checks are actually "can I get hit by a missile on the way here" checks for us, so they're pretty expensive
        |beg, end| mis.collides::<false>(beg, end, move_speed, pawn_size).is_none(),
        |beg, end| (beg.dist(end) / step_size).into(),
        // If a jump is taken we need to fix up the time in the cell we jump to by recalculating when we'll be there
        |beg, end| end.t = beg.t + end.dist(beg) / move_speed,
        |pos| pos.manhattan_distance(&target),
        |pos| {
            steps -= 1;
            steps == 0 || pos.is_same_pos(&target, step_size) || max_time <= pos.time()
        },
    );

    let elapsed = now.elapsed();

    let (steps, time, score) = match &result {
        Some((v, p)) => (max_steps - steps, v.last().unwrap().time() - curr_time, p.0),
        None => (steps, 0.0, 0.0),
    };

    #[rustfmt::skip]
    println!("search took {:?} - score: {}, steps: {}/{}, seconds: {}/{}", elapsed, score, steps, max_steps, time, max_time - curr_time);

    match result {
        Some((path, _cost)) => render_path(render_smear, render_scale, &mis, &path, move_speed, pawn_size, render_step),
        None => println!("No path found!"),
    }
}

fn render_path(smear: f32, scale: f32, mis: &MissileSet, path: &[Pos], move_speed: f32, pawn_size: f32, step: f32) {
    let pawn_size_sq = pawn_size.powi(2);
    let zero = Pos::new(0.0, 0.0, 0.0);

    // Get min/max boundaries, and then size from that

    #[rustfmt::skip]
    let (min_x, min_y) = (
        path.iter().chain(once(&zero)).map(|p| p.x).map(|x| x - 10.0).min().unwrap().0,
        path.iter().chain(once(&zero)).map(|p| p.y).map(|x| x - 10.0).min().unwrap().0,
    );

    #[rustfmt::skip]
    let (max_x, max_y) = (
        path.iter().chain(once(&SIZE)).map(|p| p.x).map(|x| x + 10.0).max().unwrap().0,
        path.iter().chain(once(&SIZE)).map(|p| p.y).map(|x| x + 10.0).max().unwrap().0,
    );

    let (size_x, size_y) = (max_x - min_x, max_y - min_y);

    let (px_size_x, px_size_y) = ((size_x as f32 * scale) as u32, (size_y as f32 * scale) as u32);

    let _ = std::fs::create_dir("out");
    for entry in std::fs::read_dir("out").unwrap().flatten() {
        let _ = std::fs::remove_file(entry.path());
    }

    // Create base image we can clone later
    let mut base_img = RgbaImage::new(px_size_x as u32, px_size_y as u32);

    for px in 0..px_size_x {
        for py in 0..px_size_y {
            // Color the background
            base_img.put_pixel(px as u32, py as u32, Rgba([0, 0, 0, 255]));

            // Scale coordinates
            let (x, y) = ((px as f32 / scale) + min_x, (py as f32 / scale) + min_y);
            let point = Vec2::new(x, y);

            // Draw player path
            for window in path.windows(2) {
                let (from, into) = (window[0].vec(), window[1].vec());
                if Line(from, into).dist_to_point_sq(point) < pawn_size_sq {
                    base_img.blend_pixel(px as u32, py as u32, Rgba([255, 255, 255, 30]));
                }
            }

            // Draw missile paths
            for mis in mis.0.values() {
                let (from, into) = (mis.origin, mis.target);
                if Line(from, into).dist_to_point_sq(point) < (5.0_f32).powi(2) {
                    base_img.blend_pixel(px as u32, py as u32, Rgba([255, 255, 255, 20]));
                }
            }
        }
    }

    base_img.save("out/step_000.png").unwrap();

    let end_time = path.last().unwrap().time();

    (0..).map(|x| x as f32 * step).take_while(|&x| x <= end_time).enumerate().par_bridge().for_each(|(i, t)| {
        // Clone image
        let mut image = base_img.clone();

        let t_beg = t;
        let t_end = t + smear;

        // Render missiles to image
        for px in 0..px_size_x {
            for py in 0..px_size_y {
                // Scale coordinates
                let (x, y) = ((px as f32 / scale) + min_x, (py as f32 / scale) + min_y);

                // Create position to check
                let pixel = Pos { x: x.into(), y: y.into(), t: t_end.into() };

                // Draw missiles
                if let Some(i) = mis.overlaps(t_beg, pixel, 0.0) {
                    let col: Lch = Srgb::<u8>::new(0, 0, 255).into_format::<f32>().into_linear().into_color();

                    let col = col.shift_hue(i as f32 * 50.0);
                    let rgb: palette::rgb::Rgb = col.into_color();

                    let (r, g, b) = rgb.into_format::<u8>().into_components();
                    image.put_pixel(px, py, Rgba([r, g, b, 220]));
                };

                path.windows(2).for_each(|window| {
                    let time = t_beg..t_end;
                    let (from, into) = (window[0], window[1]);

                    if from.time() <= time.end && into.time() >= time.start {
                        let dir = (into.vec() - from.vec()).normalized() * move_speed;

                        let max_beg = time.start.max(from.time());
                        let min_end = time.end.min(into.time());

                        // Construct a line from the overlapping segment
                        let line = Line(
                            from.vec() + dir * (time.start - from.time()).max(0.0),
                            into.vec() - dir * (into.time() - time.end).max(0.0),
                        );

                        // Check if line intersects with pixel
                        if line.dist_to_point_sq(pixel.vec()) < pawn_size_sq {
                            let beg = Pos::from_vec(line.0, max_beg);
                            let end = Pos::from_vec(line.1, min_end);

                            let collides = mis.collides::<true>(&beg, &end, move_speed, 0.0).is_some();
                            let overlaps = mis.overlaps(t_beg, pixel, 0.0).is_some();

                            let color = match (collides, overlaps) {
                                (true, true) => Rgba([255, 100, 100, 200]),

                                (true, false) => Rgba([100, 100, 255, 255]), // overlaps, no collision
                                (false, true) => Rgba([100, 255, 100, 255]), // collides, no overlap

                                (false, false) => Rgba([255, 255, 255, 200]),
                            };

                            image.blend_pixel(px, py, color);
                        }
                    }
                });
            }
        }

        // Save image
        image.save(format!("out/step_{:03}.png", i)).unwrap();
        //
    });
}
