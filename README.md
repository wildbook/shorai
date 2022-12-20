<div align="center">
</br>

# Shorai
### A simple yet flexible any-angle pathfinding library.
</br>
</div>

Uses a modified variant of Lazy Theta* to reach performance goals while still being accurate.

The algorithm itself is in [pathfind.rs](shorai/src/pathfind.rs), is relatively small, and is extensively commented. If you're curious how the crate or algorithm works I'd recommend reading through the comments in that file.

## Benchmarks

The benchmark relies on a fixed-seed random state.

To run the benchmarks, run `cargo bench --features rand` in `shorai/`

## Demo

The demo involves time as part of the pathfinding algorithm, just to show that it's possible to.

*All paths are precalculated and not modified on-the-fly. This means that the obstacle movements need to be known when the algorithm runs, which is the case for this demo.*

---

To run the demo, run `cargo run --release -- --help` in `demo/render/`. Running without `--release` takes a *long* time.

As an example command, try `cargo run --release -- --missiles 100 --seed 6424138677911309346`:

https://user-images.githubusercontent.com/1352092/208564293-8b1ef16f-df21-4085-ad08-c55e866c63f1.mp4

---

The output ends up as individual files in an `out/` folder to simplify debugging.

To merge them into a video using ffmpeg, run `ffmpeg -r 20 -i out/step_%03d.png -c:v libx264 -vf fps=25 -pix_fmt yuv420p out.mp4`.
