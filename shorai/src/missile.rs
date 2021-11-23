use std::ops::Range;

use ultraviolet::Vec2;

use crate::{geometry::Line, math::collides_within_time, pos::Pos, FxIndexMap};

#[derive(Copy, Clone, Debug)]
pub struct Missile {
    pub time_beg: f32,
    pub time_end: f32,

    pub radius: f32,
    pub origin: Vec2,
    pub target: Vec2,

    pub time_offset: Vec2,
}

impl Missile {
    #[must_use]
    pub fn new(spawn_time: f32, origin: Vec2, target: Vec2, radius: f32, speed: f32) -> Missile {
        let offset = target - origin;
        let distance = offset.mag();

        let time_moving = distance / speed;
        let time_offset = offset / time_moving;

        Missile { origin, target, radius, time_offset, time_beg: spawn_time, time_end: spawn_time + time_moving }
    }

    #[must_use]
    pub fn get_pos_range(&self, time: Range<f32>) -> Option<(Pos, Pos)> {
        let is_alive = self.time_beg <= time.end && time.start <= self.time_end;

        // Note:
        //   Moving the time_beg and time_end outside of this if statement
        //   and then checking `if time_beg < time_end` will cause LLVM to
        //   optimize away the branch entirely, resulting in assembly that
        //   is branchless. This sounds good in theory but adds around 30%
        //   to time taken to run the benchmarks.

        is_alive.then(|| {
            let time_beg = self.time_beg.max(time.start);
            let time_end = self.time_end.min(time.end);

            let off_to_beg = time_beg - self.time_beg;
            let off_to_end = time_end - time_beg;

            let beg_pos = self.origin + self.time_offset * off_to_beg;
            let end_pos = beg_pos + self.time_offset * off_to_end;

            let beg = Pos::from_vec(beg_pos, time_beg);
            let end = Pos::from_vec(end_pos, time_end);

            (beg, end)
        })
    }

    #[must_use]
    pub fn overlaps(&self, smear_from: f32, pos: Pos, pawn_size: f32) -> bool {
        self.get_pos_range(smear_from..pos.time()).map_or(false, |(beg, end)| {
            Line(beg.vec(), end.vec()).dist_to_point_sq(pos.vec()) < (self.radius + pawn_size).powi(2)
        })
    }

    #[must_use]
    pub fn collides(&self, pos: Pos, pos_velocity: Vec2, time: Range<f32>, pawn_size: f32) -> bool {
        // Slice off the ends to only keep the overlapping part
        let t_beg = self.time_beg.max(time.start).max(pos.time());
        let t_end = self.time_end.min(time.end);

        // If the missile's lifetime doesn't overlap with our path's lifetime, we're not colliding
        if t_end < t_beg {
            return false;
        }

        // Offset from current time to target start time
        let off_to_beg_mis = t_beg - self.time_beg;
        let off_to_beg_pos = t_beg - pos.time();

        // Offset positions to their positions at the target start time
        let target_pos_beg = pos.vec() + pos_velocity * off_to_beg_pos;
        let target_mis_beg = self.origin + self.time_offset * off_to_beg_mis;

        let t_dlt = t_end - t_beg;

        let radius_sq = (self.radius + pawn_size).powi(2);
        collides_within_time(target_pos_beg, target_mis_beg, pos_velocity, self.time_offset, radius_sq, t_dlt)
    }

    #[cfg(feature = "rand")]
    pub fn random(
        rand: &mut impl rand::Rng,
        x: Range<f32>,
        y: Range<f32>,
        radius: Range<f32>,
        speed: Range<f32>,
        spawn_time: Range<f32>,
    ) -> Missile {
        let origin = Vec2::new(rand.gen_range(x.clone()), rand.gen_range(y.clone()));
        let target = Vec2::new(rand.gen_range(x), rand.gen_range(y));

        Missile::new(rand.gen_range(spawn_time), origin, target, rand.gen_range(radius), rand.gen_range(speed))
    }
}

#[derive(Clone)]
pub struct MissileSet(pub FxIndexMap<u32, Missile>);

impl MissileSet {
    #[must_use]
    pub fn overlaps(&self, smear_from: f32, pos: &Pos, pawn_size: f32) -> Option<u32> {
        self.0.iter().find(|(_, missile)| missile.overlaps(smear_from, *pos, pawn_size)).map(|(&i, _)| i)
    }

    #[must_use]
    pub fn collides_velocity(&self, pos: &Pos, velocity: Vec2, end_time: f32, pawn_size: f32) -> Option<u32> {
        self.0
            .iter()
            .find(|(_, missile)| missile.collides(*pos, velocity, pos.time()..end_time, pawn_size))
            .map(|(&i, _)| i)
    }

    #[must_use]
    pub fn collides_points(&self, pos_beg: &Pos, pos_end: &Pos, move_speed: f32, pawn_size: f32) -> Option<u32> {
        let velocity = pos_beg.direction(pos_end) * move_speed;

        let dist = (pos_end.vec() - pos_beg.vec()).mag();
        let move_time = dist / move_speed;

        let move_beg_time = pos_beg.time();
        let move_end_time = move_beg_time + move_time;

        self.collides_velocity(pos_beg, velocity, move_end_time, pawn_size)
            .or_else(|| self.overlaps(move_end_time, pos_end, pawn_size))
    }
}

#[test]
fn missile_overlaps() {
    let missile = Missile::new(0.0, Vec2::new(-50.0, 0.0), Vec2::new(50.0, 0.0), 1.0, 10.0);

    // Collides within valid time ranges
    assert!(missile.overlaps(4.8, Pos::new(0.0, 0.0, 5.2), 0.0));

    // Does not collide outside of valid time ranges
    assert!(!missile.overlaps(4.8, Pos::new(0.0, 0.0, 4.9), 0.0));
    assert!(!missile.overlaps(5.1, Pos::new(0.0, 0.0, 5.2), 0.0));
}

#[test]
fn missile_collides_or_overlaps() {
    let mut missiles = MissileSet(FxIndexMap::default());
    missiles.0.insert(0, Missile::new(0.0, Vec2::new(-50.0, 0.0), Vec2::new(50.0, 0.0), 1.0, 10.0));

    let source = Pos::new(0.0, -10.0, 5.0);

    let move_speed = 100.0;

    // Does not collide outside of valid time ranges
    let target = Pos::new(0.0, 0.0, 10.0);
    assert!(missiles.collides_points(&source, &target, move_speed, 0.0).is_some());

    let target = Pos::new(0.0, 0.0, 10.0);
    assert!(missiles.collides_points(&source, &target, move_speed, 0.0).is_some());
}

#[test]
fn missile_collides_basic() {
    let missile = Missile::new(0.0, Vec2::new(-100.0, 0.0), Vec2::new(0.0, 0.0), 1.0, 10.0);

    let pos = Pos::from_vec(Vec2::new(100.0, 0.0), 0.0);
    let pos_v = Vec2::new(-10.0, 0.0);

    // Collides within valid time ranges
    assert!(missile.collides(pos, pos_v, 0.0..10.0, 0.0));
    assert!(missile.collides(pos, pos_v, 8.0..12.0, 0.0));

    // Does not collide outside of valid time ranges
    assert!(!missile.collides(pos, pos_v, 00.0..8.0, 0.0));
    assert!(!missile.collides(pos, pos_v, 12.0..20.0, 0.0));
}

#[test]
fn missile_collides_with_same_spawn_time() {
    let spawn_time = 5.0;

    let missile = Missile::new(spawn_time, Vec2::new(-100.0, 0.0), Vec2::new(0.0, 0.0), 1.0, 10.0);

    let pos = Pos::from_vec(Vec2::new(100.0, 0.0), spawn_time);
    let pos_v = Vec2::new(-10.0, 0.0);

    // Collides within valid time ranges
    assert!(missile.collides(pos, pos_v, spawn_time + 0.0..spawn_time + 10.0, 0.0));
    assert!(missile.collides(pos, pos_v, spawn_time + 8.0..spawn_time + 12.0, 0.0));

    // Does not collide outside of valid time ranges
    assert!(!missile.collides(pos, pos_v, spawn_time + 00.0..spawn_time + 8.0, 0.0));
    assert!(!missile.collides(pos, pos_v, spawn_time + 12.0..spawn_time + 20.0, 0.0));
}

#[test]
fn missile_collides_with_missile_spawn_time() {
    let missile = Missile::new(10.0, Vec2::new(-100.0, 0.0), Vec2::new(0.0, 0.0), 1.0, 10.0);

    let pos = Pos::new(200.0, 0.0, 0.0);
    let pos_v = Vec2::new(-10.0, 0.0);

    // Collides within valid time ranges
    assert!(missile.collides(pos, pos_v, 10.0..20.0, 0.0));
    assert!(missile.collides(pos, pos_v, 18.0..22.0, 0.0));

    // Does not collide outside of valid time ranges
    assert!(!missile.collides(pos, pos_v, 10.0..18.0, 0.0));
    assert!(!missile.collides(pos, pos_v, 22.0..30.0, 0.0));
}

#[test]
fn missile_collides_with_pos_spawn_time() {
    let missile = Missile::new(0.0, Vec2::new(-200.0, 0.0), Vec2::new(0.0, 0.0), 1.0, 10.0);

    let pos = Pos::new(100.0, 0.0, 10.0);
    let pos_v = Vec2::new(-10.0, 0.0);

    // Collides within valid time ranges
    assert!(missile.collides(pos, pos_v, 10.0..20.0, 0.0));
    assert!(missile.collides(pos, pos_v, 18.0..22.0, 0.0));

    // Does not collide outside of valid time ranges
    assert!(!missile.collides(pos, pos_v, 10.0..18.0, 0.0));
    assert!(!missile.collides(pos, pos_v, 22.0..30.0, 0.0));
}

#[test]
fn missile_collides_with_different_spawn_time() {
    let missile = Missile::new(10.0, Vec2::new(-300.0, 0.0), Vec2::new(0.0, 0.0), 1.0, 10.0);

    let pos = Pos::new(200.0, 0.0, 20.0);
    let pos_v = Vec2::new(-10.0, 0.0);

    // Collides within valid time ranges
    assert!(missile.collides(pos, pos_v, 30.0..40.0, 0.0));
    assert!(missile.collides(pos, pos_v, 38.0..42.0, 0.0));

    // Does not collide outside of valid time ranges
    assert!(!missile.collides(pos, pos_v, 30.0..38.0, 0.0));
    assert!(!missile.collides(pos, pos_v, 42.0..50.0, 0.0));
}

#[test]
fn missile_collides_with_static_object() {
    let missile = Missile::new(0.0, Vec2::new(-300.0, 0.0), Vec2::new(0.0, 0.0), 1.0, 10.0);

    let pos = Pos::new(0.0, 0.0, 0.0);
    let pos_v = Vec2::new(0.0, 0.0);

    // Collides within valid time ranges
    assert!(missile.collides(pos, pos_v, 30.0..31.0, 0.0));

    // Does not collide outside of valid time ranges
    assert!(!missile.collides(pos, pos_v, 28.0..29.0, 0.0));
    assert!(!missile.collides(pos, pos_v, 31.0..32.0, 0.0));
}
