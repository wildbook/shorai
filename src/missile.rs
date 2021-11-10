use std::ops::Range;

use ultraviolet::Vec2;

use crate::{geometry::Line, math::predict_collision_time, pos::Pos, FxIndexMap};

#[derive(Copy, Clone, Debug)]
pub struct Missile {
    pub origin: Vec2,
    pub target: Vec2,

    pub radius_sq: f32,

    pub time_beg: f32, // Timestamp at missile start
    pub time_end: f32, // Timestamp at missile end

    pub time_offset: Vec2,
}

impl Missile {
    fn new(spawn_time: f32, origin: Vec2, target: Vec2, radius: f32, speed: f32) -> Missile {
        let offset = target - origin;
        let distance = offset.mag();
        let radius_sq = radius * radius;

        let time_moving = distance / speed;
        let time_offset = offset / time_moving;

        Missile { origin, target, radius_sq, time_offset, time_beg: spawn_time, time_end: spawn_time + time_moving }
    }

    #[inline]
    fn get_pos_range(&self, time: Range<f32>) -> Option<(Pos, Pos)> {
        let is_alive = self.time_beg <= time.end && time.start <= self.time_end;

        is_alive.then(|| {
            let time_beg = self.time_beg.max(time.start);
            let time_end = self.time_end.min(time.end);

            let off_to_beg = (time.start - self.time_beg).max(0.0);
            let off_to_end = time_end - time_beg;

            let beg_pos = self.origin + self.time_offset * off_to_beg;
            let end_pos = beg_pos + self.time_offset * off_to_end;

            let beg = Pos::from_vec(beg_pos, time_beg);
            let end = Pos::from_vec(end_pos, time_end);

            (beg, end)
        })
    }

    fn overlaps(&self, smear_from: f32, pos: Pos, pawn_size: f32) -> bool {
        self.get_pos_range(smear_from..pos.t.0).map_or(false, |(beg, end)| {
            let radius_sq = (self.radius_sq.sqrt() - pawn_size).powi(2);
            Line(beg.vec(), end.vec()).dist_to_point_sq(pos.vec()) < radius_sq
        })
    }

    fn collides(&self, pos: Pos, pos_velocity: Vec2, time: Range<f32>) -> bool {
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

        predict_collision_time(target_pos_beg, target_mis_beg, pos_velocity, self.time_offset, self.radius_sq)
            .map_or(false, |t| t <= t_dlt)
    }
}

//

#[test]
fn predict_collision_time_is_correct() {
    let lhs = Vec2::new(-100.0, 0.0);
    let rhs = Vec2::new(100.0, 0.0);

    let v_lhs = Vec2::new(10.0, 0.0);
    let v_rhs = Vec2::new(-10.0, 0.0);

    let radius_sq = 10.0_f32.powi(2);

    let time = predict_collision_time(lhs, rhs, v_lhs, v_rhs, radius_sq);

    assert_eq!(time, Some(9.5));
}

#[test]
fn missile_collides_basic() {
    let missile = Missile::new(0.0, Vec2::new(-100.0, 0.0), Vec2::new(0.0, 0.0), 1.0, 10.0);

    let pos = Pos::from_vec(Vec2::new(100.0, 0.0), 0.0);
    let pos_v = Vec2::new(-10.0, 0.0);

    // Collides within valid time ranges
    assert!(missile.collides(pos, pos_v, 0.0..10.0));
    assert!(missile.collides(pos, pos_v, 8.0..12.0));

    // Does not collide outside of valid time ranges
    assert!(!missile.collides(pos, pos_v, 00.0..8.0));
    assert!(!missile.collides(pos, pos_v, 12.0..20.0));
}

#[test]
fn missile_collides_with_same_spawn_time() {
    let spawn_time = 5.0;

    let missile = Missile::new(spawn_time, Vec2::new(-100.0, 0.0), Vec2::new(0.0, 0.0), 1.0, 10.0);

    let pos = Pos::from_vec(Vec2::new(100.0, 0.0), spawn_time);
    let pos_v = Vec2::new(-10.0, 0.0);

    // Collides within valid time ranges
    assert!(missile.collides(pos, pos_v, spawn_time + 0.0..spawn_time + 10.0));
    assert!(missile.collides(pos, pos_v, spawn_time + 8.0..spawn_time + 12.0));

    // Does not collide outside of valid time ranges
    assert!(!missile.collides(pos, pos_v, spawn_time + 00.0..spawn_time + 8.0));
    assert!(!missile.collides(pos, pos_v, spawn_time + 12.0..spawn_time + 20.0));
}

#[test]
fn missile_collides_with_missile_spawn_time() {
    let missile = Missile::new(10.0, Vec2::new(-100.0, 0.0), Vec2::new(0.0, 0.0), 1.0, 10.0);

    let pos = Pos::new(200.0, 0.0, 0.0);
    let pos_v = Vec2::new(-10.0, 0.0);

    // Collides within valid time ranges
    assert!(missile.collides(pos, pos_v, 10.0..20.0));
    assert!(missile.collides(pos, pos_v, 18.0..22.0));

    // Does not collide outside of valid time ranges
    assert!(!missile.collides(pos, pos_v, 10.0..18.0));
    assert!(!missile.collides(pos, pos_v, 22.0..30.0));
}

#[test]
fn missile_collides_with_pos_spawn_time() {
    let missile = Missile::new(0.0, Vec2::new(-200.0, 0.0), Vec2::new(0.0, 0.0), 1.0, 10.0);

    let pos = Pos::new(100.0, 0.0, 10.0);
    let pos_v = Vec2::new(-10.0, 0.0);

    // Collides within valid time ranges
    assert!(missile.collides(pos, pos_v, 10.0..20.0));
    assert!(missile.collides(pos, pos_v, 18.0..22.0));

    // Does not collide outside of valid time ranges
    assert!(!missile.collides(pos, pos_v, 10.0..18.0));
    assert!(!missile.collides(pos, pos_v, 22.0..30.0));
}

#[test]
fn missile_collides_with_different_spawn_time() {
    let missile = Missile::new(10.0, Vec2::new(-300.0, 0.0), Vec2::new(0.0, 0.0), 1.0, 10.0);

    let pos = Pos::new(200.0, 0.0, 20.0);
    let pos_v = Vec2::new(-10.0, 0.0);

    // Collides within valid time ranges
    assert!(missile.collides(pos, pos_v, 30.0..40.0));
    assert!(missile.collides(pos, pos_v, 38.0..42.0));

    // Does not collide outside of valid time ranges
    assert!(!missile.collides(pos, pos_v, 30.0..38.0));
    assert!(!missile.collides(pos, pos_v, 42.0..50.0));
}

//

#[derive(Clone)]
pub struct MissileSet(pub FxIndexMap<u32, Missile>);

impl MissileSet {
    pub fn overlaps(&self, smear_from: f32, pos: Pos, pawn_size: f32) -> Option<u32> {
        self.0.iter().find(|(_, missile)| missile.overlaps(smear_from, pos, pawn_size)).map(|(&i, _)| i)
    }

    /// If `TRUST_END_TIME` is set to `true`, `end.time()` will be used.
    /// Otherwise it will be recalculated from the supplied movement speed.
    pub fn collides<const TRUST_END_TIME: bool>(&self, pos_beg: &Pos, pos_end: &Pos, move_speed: f32) -> Option<u32> {
        let pos_delta = pos_end.vec() - pos_beg.vec();
        let pos_velocity = pos_delta.normalized() * move_speed;

        let time_beg = pos_beg.time();
        let time_end = match TRUST_END_TIME {
            true => pos_end.time(),
            false => time_beg + pos_delta.mag() / move_speed,
        };

        self.0.iter().find(|(_, missile)| missile.collides(*pos_beg, pos_velocity, time_beg..time_end)).map(|(&i, _)| i)
    }
}

//

use rand::Rng;

impl Missile {
    pub fn random(
        rand: &mut impl Rng,
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