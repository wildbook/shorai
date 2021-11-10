use ordered_float::OrderedFloat;
use pathfinding::prelude::absdiff;
use ultraviolet::Vec2;

use crate::{missile::MissileSet, Cost};

#[derive(Copy, Clone, Eq)]
pub struct Pos {
    // Position
    pub x: OrderedFloat<f32>,
    pub y: OrderedFloat<f32>,

    // Tick
    pub t: OrderedFloat<f32>,
}

impl std::hash::Hash for Pos {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.x.hash(state);
        self.y.hash(state);
        self.t.hash(state);
    }
}

impl PartialEq for Pos {
    fn eq(&self, other: &Self) -> bool {
        self.x == other.x && self.y == other.y && self.t == other.t
    }
}

impl std::fmt::Debug for Pos {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Pos").field("x", &self.x.0).field("y", &self.y.0).field("t", &self.t.0).finish()
    }
}

impl Pos {
    #[inline(always)]
    pub const fn new(x: f32, y: f32, t: f32) -> Pos {
        Pos { x: OrderedFloat(x), y: OrderedFloat(y), t: OrderedFloat(t) }
    }

    #[inline(always)]
    pub const fn from_vec(v: Vec2, t: f32) -> Pos {
        Pos::new(v.x, v.y, t)
    }

    #[inline(always)]
    pub const fn time(&self) -> f32 {
        self.t.0
    }

    #[inline(always)]
    pub const fn x(&self) -> f32 {
        self.x.0
    }

    #[inline(always)]
    pub const fn y(&self) -> f32 {
        self.y.0
    }

    #[inline(always)]
    pub const fn vec(&self) -> Vec2 {
        Vec2::new(self.x(), self.y())
    }

    pub fn next(&self, diff_x: f32, diff_y: f32, diff_t: f32) -> Pos {
        Pos { x: self.x + diff_x, y: self.y + diff_y, t: self.t + diff_t }
    }

    pub fn is_same_pos(&self, other: &Pos, step_size: f32) -> bool {
        (self.x - other.x).abs() < step_size && (self.y - other.y).abs() < step_size
    }

    pub fn manhattan_distance(&self, other: &Pos) -> Cost {
        Cost::from(absdiff(self.x(), other.x()) + absdiff(self.y(), other.y()))
    }

    pub fn successors<'a>(
        &self,
        // All missiles that are currently active / relevant
        missiles: &'a MissileSet,
        // The time it takes to move `step_size` units
        step_time: f32,
        // The amount of grid "cells" moved in a single movement
        step_size: f32,
    ) -> impl IntoIterator<Item = (Pos, Cost)> + 'a {
        const DIR: f32 = 1.0;
        const DIA: f32 = std::f32::consts::SQRT_2;

        let s = step_size;

        // Time spent moving
        let dir_diff_t = DIR * step_time;
        let dia_diff_t = DIA * step_time;

        let opts = [
            (self.next(s, 0.0, dir_diff_t), DIR.into()),
            (self.next(0.0, s, dir_diff_t), DIR.into()),
            (self.next(0.0, -s, dir_diff_t), DIR.into()),
            (self.next(-s, 0.0, dir_diff_t), DIR.into()),
            // Diagonals
            (self.next(s, s, dia_diff_t), DIA.into()),
            (self.next(-s, s, dia_diff_t), DIA.into()),
            (self.next(-s, -s, dia_diff_t), DIA.into()),
            (self.next(s, -s, dia_diff_t), DIA.into()),
        ];

        // Filter out moves that would put us in a missile
        let smear_from = self.time();
        opts.into_iter().filter(move |&(pos, _)| missiles.overlaps(smear_from, pos, 0.0).is_none())
    }

    pub fn dist_sqr(&self, other: &Pos) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).0
    }

    pub fn dist(&self, other: &Pos) -> f32 {
        self.dist_sqr(other).sqrt()
    }
}
