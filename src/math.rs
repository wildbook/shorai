use ultraviolet::Vec2;

pub fn predict_collision_time(
    // Origin positions
    lhs: Vec2,
    rhs: Vec2,
    // Velocities
    v_lhs: Vec2,
    v_rhs: Vec2,
    // Combined radius (squared)
    radius_sq: f32,
) -> Option<f32> {
    // https://stackoverflow.com/a/11369989/6713695

    // Check if the initial states are already touching
    if (lhs - rhs).mag_sq() < radius_sq {
        return Some(0.0);
    }

    let c0 = v_lhs.dot(v_lhs) + v_rhs.dot(v_rhs) - 2.0 * (v_lhs.x * v_rhs.x + v_lhs.y * v_rhs.y);

    let c1x = (lhs.x * v_lhs.x) - (lhs.x * v_rhs.x) - (rhs.x * v_lhs.x) + (rhs.x * v_rhs.x);
    let c1y = (lhs.y * v_lhs.y) - (lhs.y * v_rhs.y) - (rhs.y * v_lhs.y) + (rhs.y * v_rhs.y);

    let c1 = c1x + c1y;
    let c2 = lhs.x.powi(2) + lhs.y.powi(2) + rhs.x.powi(2) + rhs.y.powi(2)
        - (2.0 * lhs.x * rhs.x)
        - (2.0 * lhs.y * rhs.y)
        - radius_sq;

    match solve_quadratic(c0, 2.0 * c1, c2) {
        QuadricSolution::One(t) if t > 0.0 => Some(t),
        QuadricSolution::Two(t, t2) => match (t > 0.0, t2 > 0.0) {
            (true, true) => Some(t.min(t2)),
            (true, false) => Some(t),
            (false, true) => Some(t2),
            (false, false) => None,
        },
        _ => None,
    }
}

pub enum QuadricSolution {
    None,
    One(f32),
    Two(f32, f32),
}

#[inline(always)]
fn solve_quadratic(c0: f32, c1: f32, c2: f32) -> QuadricSolution {
    /* normal form: x^2 + px + q = 0 */
    let p = c1 / (2.0 * c0);
    let q = c2 / c0;
    let d = p * p - q;

    // FIXME use approxeq
    if d == 0.0 {
        QuadricSolution::One(-p)
    } else if d < 0.0 {
        QuadricSolution::None
    } else {
        let sqrt = d.sqrt();
        QuadricSolution::Two(sqrt - p, -sqrt - p)
    }
}
