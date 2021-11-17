use ultraviolet::Vec2;

#[inline]
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

#[inline(always)]
pub fn absdiff(x: f32, y: f32) -> f32 {
    if x < y {
        y - x
    } else {
        x - y
    }
}
