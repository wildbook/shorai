use ultraviolet::Vec2;

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Line(pub Vec2, pub Vec2);

impl Line {
    #[inline]
    pub fn dist_to_point_sq(&self, point: Vec2) -> f32 {
        let v = self.0;
        let w = self.1;
        let p = point;

        let d = w - v;

        // i.e. |w-v|^2 -  avoid a sqrt
        let l2 = d.mag_sq();

        // Consider the line extending the segment, parameterized as v + t (w - v).
        // We find projection of point p onto the line.
        // It falls where t = [(p-v) . (w-v)] / |w-v|^2
        let t = ((p.x - v.x) * (w.x - v.x) + (p.y - v.y) * (w.y - v.y)) / l2;

        // We clamp t from [0,1] to handle points outside the segment vw.
        let t = t.max(0.0).min(1.0);

        // Projection falls on the segment
        let pos = v + (t * d);
        (pos - p).mag_sq()
    }
}

#[test]
fn dist_to_point_sq_is_valid() {
    let line = Line(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0));
    let point = Vec2::new(0.5, 1.0);

    assert_eq!(line.dist_to_point_sq(point).sqrt(), 1.0);
}

#[test]
fn dist_to_point_sq_with_zero_mag_line_is_valid() {
    let line = Line(Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0));
    let point = Vec2::new(1.0, 0.0);

    assert_eq!(line.dist_to_point_sq(point).sqrt(), 1.0);
}
