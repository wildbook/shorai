mod common;
mod math;

pub mod geometry;
pub mod missile;
pub mod pathfind;
pub mod pos;

pub type FxIndexMap<K, V> = indexmap::IndexMap<K, V, std::hash::BuildHasherDefault<rustc_hash::FxHasher>>;
pub type Cost = ordered_float::OrderedFloat<f32>;
