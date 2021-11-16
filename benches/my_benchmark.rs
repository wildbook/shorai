use criterion::{criterion_group, criterion_main, Criterion};
use rand::{prelude::StdRng, SeedableRng};

use shorai::{
    missile::{Missile, MissileSet},
    pathfind,
    pos::Pos,
    FxIndexMap,
};

fn generic_path_benchmark(c: &mut Criterion) {
    let seed = 13870599315289853359;

    let curr_time = 0.0; // Current time

    let move_speed = 325.0;
    let pawn_size = 55.0;
    let step_size = 50.0;
    let missile_count = 100;

    let max_time = 20.0;

    let origin = Pos::new(0.0, 0.0, curr_time);
    let target = Pos::new(1000.0, 1000.0, max_time);

    let rand = &mut StdRng::seed_from_u64(seed);

    let step_time = step_size / move_speed;

    let mut missiles = FxIndexMap::default();
    for i in 0..missile_count {
        let missile = Missile::random(
            rand,
            -100.0..target.x(),
            -100.0..target.y(),
            60.0 + pawn_size..120.0 + pawn_size,
            300.0..1000.0,
            curr_time..max_time,
        );

        missiles.insert(i, missile);
    }

    let mis = MissileSet(missiles);

    c.bench_function("find_path", |b| {
        b.iter(|| {
            pathfind::find(
                origin,
                |pos| pos.successors(&mis, step_time, step_size),
                |beg, end| mis.collides::<false>(beg, end, move_speed).is_none(),
                |beg, end| (beg.dist(end) / step_size).into(),
                |beg, end| end.t = beg.t + beg.dist(end) / move_speed,
                |pos| pos.manhattan_distance(&target),
                |pos| pos.is_same_pos(&target, step_size) || max_time <= pos.time(),
            )
        });
    });
}

criterion_group!(benches, generic_path_benchmark);
criterion_main!(benches);
