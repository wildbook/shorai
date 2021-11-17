use ultraviolet::Vec2;

pub fn solve_collision_time(
    // Origin positions
    p_lhs: Vec2,
    p_rhs: Vec2,
    // Velocities
    v_lhs: Vec2,
    v_rhs: Vec2,
    // Combined radius (squared)
    radius_sq: f32,
) -> Option<f32> {
    // https://stackoverflow.com/a/11369989/6713695

    // Check if the initial states are already touching
    if (p_lhs - p_rhs).mag_sq() < radius_sq {
        return Some(0.0);
    }

    let c0 = v_lhs.dot(v_lhs) + v_rhs.dot(v_rhs) - v_lhs.dot(v_rhs) - v_lhs.dot(v_rhs);
    let c1 = v_lhs.dot(p_lhs) + v_rhs.dot(p_rhs) - v_rhs.dot(p_lhs) - v_lhs.dot(p_rhs);
    let c2 = p_lhs.dot(p_lhs) + p_rhs.dot(p_rhs) - p_lhs.dot(p_rhs) - p_lhs.dot(p_rhs);

    match solve_quadratic(c0, c1 + c1, c2 - radius_sq) {
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

pub fn collides_within_time(
    // Origin positions
    p_lhs: Vec2,
    p_rhs: Vec2,
    // Velocities
    v_lhs: Vec2,
    v_rhs: Vec2,
    // Combined radius (squared)
    radius_sq: f32,
    time: f32,
) -> bool {
    // https://stackoverflow.com/a/11369989/6713695

    // Check if the initial states are already touching
    if (p_lhs - p_rhs).mag_sq() < radius_sq {
        return true;
    }

    // Calculate quadratic coefficients
    let c0 = v_lhs.dot(v_lhs) + v_rhs.dot(v_rhs) - v_lhs.dot(v_rhs) - v_lhs.dot(v_rhs);
    let c1 = v_lhs.dot(p_lhs) + v_rhs.dot(p_rhs) - v_rhs.dot(p_lhs) - v_lhs.dot(p_rhs);
    let c2 = p_lhs.dot(p_lhs) + p_rhs.dot(p_rhs) - p_lhs.dot(p_rhs) - p_lhs.dot(p_rhs);
    let c2 = c2 - radius_sq;

    // Solve the resulting quadratic formula
    let p = (c1 + c1) / (c0 + c0);
    let q = c2 / c0;
    let d = p * p - q;

    if d == 0.0 && 0.0 < p {
        p < time
    } else if 0.0 < d {
        let p_sq_signed = p * p.abs();
        let time_sq = time * time; // add .abs(); if it's free?
        let value = time_sq + p_sq_signed;

        (d > p_sq_signed) && (d < value) || (d < -p_sq_signed) && (-d < value)
    } else {
        false
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
    let p = c1 / (c0 + c0);
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

pub fn absdiff(x: f32, y: f32) -> f32 {
    if x < y {
        y - x
    } else {
        x - y
    }
}

#[test]
fn solve_collision_time_is_correct() {
    let lhs = Vec2::new(-100.0, 0.0);
    let rhs = Vec2::new(100.0, 0.0);

    let v_lhs = Vec2::new(10.0, 0.0);
    let v_rhs = Vec2::new(-10.0, 0.0);

    let radius_sq = 10.0_f32.powi(2);

    let time = solve_collision_time(lhs, rhs, v_lhs, v_rhs, radius_sq);

    assert_eq!(time, Some(9.5));
}
