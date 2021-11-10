use ultraviolet::Vec2;

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Line(pub Vec2, pub Vec2);

impl Line {
    #[inline]
    pub fn dist_to_point_sq(&self, point: Vec2) -> f32 {
        let v = self.0;
        let w = self.1;
        let p = point;

        // i.e. |w-v|^2 -  avoid a sqrt
        let l2 = (w - v).mag_sq();

        // v == w case
        if l2 == 0.0 {
            return (v - p).mag_sq();
        }

        // Consider the line extending the segment, parameterized as v + t (w - v).
        // We find projection of point p onto the line.
        // It falls where t = [(p-v) . (w-v)] / |w-v|^2
        let t = ((p.x - v.x) * (w.x - v.x) + (p.y - v.y) * (w.y - v.y)) / l2;

        // We clamp t from [0,1] to handle points outside the segment vw.
        let t = t.max(0.0).min(1.0);

        // Projection falls on the segment
        let pos = v + (t * (w - v));
        (pos - p).mag_sq()
    }
}
