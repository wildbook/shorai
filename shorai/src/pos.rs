use ordered_float::OrderedFloat;
use ultraviolet::Vec2;

use crate::{math::absdiff, Cost};

#[derive(Copy, Clone, Eq, PartialEq, Hash)]
pub struct Pos {
    pub x: OrderedFloat<f32>,
    pub y: OrderedFloat<f32>,
    pub t: OrderedFloat<f32>,
}

impl std::fmt::Debug for Pos {
    #[rustfmt::skip]
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Pos")
            .field("x", &self.x.0)
            .field("y", &self.y.0)
            .field("t", &self.t.0)
            .finish()
    }
}

impl Pos {
    #[must_use]
    #[inline(always)]
    pub const fn new(x: f32, y: f32, t: f32) -> Pos {
        Pos { x: OrderedFloat(x), y: OrderedFloat(y), t: OrderedFloat(t) }
    }

    #[must_use]
    #[inline(always)]
    pub const fn from_vec(v: Vec2, t: f32) -> Pos {
        Pos::new(v.x, v.y, t)
    }

    #[must_use]
    #[inline(always)]
    pub const fn time(&self) -> f32 {
        self.t.0
    }

    #[must_use]
    #[inline(always)]
    pub const fn x(&self) -> f32 {
        self.x.0
    }

    #[must_use]
    #[inline(always)]
    pub const fn y(&self) -> f32 {
        self.y.0
    }

    #[must_use]
    #[inline(always)]
    pub const fn vec(&self) -> Vec2 {
        Vec2::new(self.x(), self.y())
    }

    #[must_use]
    #[inline(always)]
    pub fn next(&self, diff_x: f32, diff_y: f32, diff_t: f32) -> Pos {
        Pos { x: self.x + diff_x, y: self.y + diff_y, t: self.t + diff_t }
    }

    /// Returns a normalized direction vector between two positions.
    /// In cases where the two positions are the same, the returned vector is (0, 0).
    #[must_use]
    #[inline(always)]
    pub fn direction(&self, towards: &Pos) -> Vec2 {
        match towards.vec() - self.vec() {
            v if v.x == 0.0 && v.y == 0.0 => v,
            v => v.normalized(),
        }
    }

    #[must_use]
    #[inline(always)]
    pub fn is_same_pos(&self, other: &Pos, epsilon: f32) -> bool {
        (self.x - other.x).abs() < epsilon && (self.y - other.y).abs() < epsilon
    }

    #[must_use]
    #[inline]
    pub fn successors<'a>(
        &self,
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

        [
            // Staying still
            (self.next(0.0, 0.0, step_time), 0.0.into()),
            // X / Y movement
            (self.next(s, 0.0, dir_diff_t), DIR.into()),
            (self.next(0.0, s, dir_diff_t), DIR.into()),
            (self.next(0.0, -s, dir_diff_t), DIR.into()),
            (self.next(-s, 0.0, dir_diff_t), DIR.into()),
            // Diagonals
            (self.next(s, s, dia_diff_t), DIA.into()),
            (self.next(-s, s, dia_diff_t), DIA.into()),
            (self.next(-s, -s, dia_diff_t), DIA.into()),
            (self.next(s, -s, dia_diff_t), DIA.into()),
        ]
    }

    #[must_use]
    #[inline(always)]
    pub fn dist_manhattan(&self, other: &Pos) -> f32 {
        absdiff(self.x(), other.x()) + absdiff(self.y(), other.y())
    }

    #[must_use]
    #[inline(always)]
    pub fn dist_sq(&self, other: &Pos) -> f32 {
        let dx = (self.x - other.x).powi(2);
        let dy = (self.y - other.y).powi(2);
        dx + dy
    }

    #[must_use]
    #[inline(always)]
    pub fn dist(&self, other: &Pos) -> f32 {
        self.dist_sq(other).sqrt()
    }

    /// This is a reasonable default jump check for pathfinding.
    ///
    /// If all three points are different, the middle point will be removed from the path and `n1`
    /// will be returned as the parent. The time where `to_node` is reached will be recalculated.
    ///
    /// If all three points are the same, the middle point will be removed from the path and `n1`
    /// will be returned as the parent. The time where `to_node` is reached will NOT be updated
    /// since we're assuming that the timing info held is its sole purpose. This will however allow
    /// folding chains of nodes at the same point (waiting for an obstacle to pass) into two nodes,
    /// one where the position is initially reached and one with the purpose to delay movement.
    ///
    /// If two of the points are the same, they will all be kept.
    #[must_use]
    pub fn jump_calc(n1: &Pos, n2: &Pos, to_node: &Pos, move_speed: f32) -> Option<Pos> {
        let n1_n2_same = n1.dist_manhattan(n2) < 0.1;
        let n1_n3_same = n1.dist_manhattan(to_node) < 0.1;
        let n2_n3_same = n2.dist_manhattan(to_node) < 0.1;

        // A normal jump, recalculate the end time.
        let mut node = *to_node;

        // n1, n2, n3 are all different points
        if !n1_n2_same && !n2_n3_same && !n1_n3_same {
            node.t = n1.t + to_node.dist(n1) / move_speed;
            return Some(node);
        }

        // Since all three are identical we can strip n2, but we want to keep the time info or
        // we'll be optimizing away intentional delays.
        if n1_n2_same && n2_n3_same {
            return Some(node);
        }

        None
    }
}

#[test]
fn direction_does_not_return_nan() {
    let p1 = Pos::new(0.0, 0.0, 0.0);
    let p2 = Pos::new(0.0, 0.0, 0.0);
    assert_eq!(p1.direction(&p2), Vec2::new(0.0, 0.0));
}

#[test]
fn direction_returns_valid_values() {
    let p1 = Pos::new(0.0, 0.0, 0.0);
    let p2 = Pos::new(1.0, 0.0, 0.0);
    assert_eq!(p1.direction(&p2), Vec2::new(1.0, 0.0));

    let p1 = Pos::new(0.0, 0.0, 0.0);
    let p2 = Pos::new(0.0, 1.0, 0.0);
    assert_eq!(p1.direction(&p2), Vec2::new(0.0, 1.0));

    let p1 = Pos::new(0.0, 0.0, 0.0);
    let p2 = Pos::new(1.0, 1.0, 0.0);
    assert_eq!(p1.direction(&p2), Vec2::new(0.70710677, 0.70710677));
}
