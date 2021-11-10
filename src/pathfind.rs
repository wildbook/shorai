// All credit for the base A* implementation goes to [the `pathfinding` crate](https://crates.io/crates/pathfinding).
// Theta* was then added on top of that, and further modifications were made for the purposes of this project.

use indexmap::map::Entry::{Occupied, Vacant};
use num_traits::Zero;
use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::hash::Hash;

use crate::common::reverse_path;
use crate::FxIndexMap;

pub fn find<N, C, FN, IN, FL, FC, FJ, FH, FS>(
    start: N,
    mut successors: FN,
    mut is_valid_move: FL,
    // Used to dynamially calculate cost for arbitrary jumps.
    // It is important that uses the same calculation as  `successors` does.
    mut movement_cost: FC,
    // Called when a jump is taken, allows recalculation of time etc.
    mut take_jump: FJ,
    // Used to estimate the distance from a specific point to the goal
    mut heuristic: FH,
    mut success: FS,
) -> Option<(Vec<N>, C)>
where
    N: Eq + Hash + Copy,
    C: Zero + Ord + Copy + std::fmt::Debug,
    FN: FnMut(&N) -> IN,
    IN: IntoIterator<Item = (N, C)>,
    FL: FnMut(&N, &N) -> bool,
    FJ: FnMut(&N, &mut N),
    FC: FnMut(&N, &N) -> C,
    FH: FnMut(&N) -> C,
    FS: FnMut(&N) -> bool,
{
    let mut pending = BinaryHeap::new();
    pending.push(PotentialNode { estimated_cost: Zero::zero(), cost: Zero::zero(), index: 0 });

    let mut visited: FxIndexMap<N, (usize, C)> = FxIndexMap::default();
    visited.insert(start, (usize::max_value(), Zero::zero()));

    // pX = parent X - p0 = current node, p1 = parent of p0, p2 = parent of p1, etc.
    while let Some(PotentialNode { cost, index: p0_index, .. }) = pending.pop() {
        let (p0_node, &(p1_index, p0_cost)) = visited.get_index(p0_index).unwrap();

        if success(p0_node) {
            // println!("Reached goal - entries: {}", visited.len());
            let path = reverse_path(&visited, |&(p, _)| p, p0_index);
            return Some((path, cost));
        }

        // We may have inserted a node several time into the binary heap if we found
        // a better way to access it. Ensure that we are currently dealing with the
        // best path and discard the others.
        if p0_cost < cost {
            continue;
        }

        let p0_copy = &p0_node.clone();
        for (mut next_node, move_cost) in successors(p0_node) {
            let (next_parent_idx, next_cost) = match visited.get_index(p1_index) {
                // Check if we're able to do IVM jumps.
                Some((p1_node, &(_, p1_cost))) if is_valid_move(p1_node, &next_node) => {
                    // We'll want to fix up the time in next_node.
                    take_jump(p1_node, &mut next_node);

                    // If we could, return p1 and skip over the p0 node entirely.
                    (p1_index, p1_cost + movement_cost(p1_node, &next_node))
                }

                // If we're not able to do jumps, we're just going to do normal A* movements.
                //
                // We still need to IVM check those too though, since A* by default does not
                // handle intermediate states, so we'd potentially run into collisions during
                // transitions otherwise.
                _ if is_valid_move(p0_copy, &next_node) => (p0_index, cost + move_cost),

                // And if we can't do either due to IVM checks failing, we give up on this node.
                _ => continue,
            };

            let (next_heuristic, next_node_idx) = match visited.entry(next_node) {
                Vacant(entry) => {
                    let out = (heuristic(entry.key()), entry.index());
                    entry.insert((next_parent_idx, next_cost));
                    out
                }
                Occupied(mut entry) if next_cost < entry.get().1 => {
                    let out = (heuristic(entry.key()), entry.index());
                    entry.insert((next_parent_idx, next_cost));
                    out
                }
                Occupied(_) => continue,
            };

            pending.push(PotentialNode {
                estimated_cost: next_cost + next_heuristic,
                cost: next_cost,
                index: next_node_idx,
            });
        }
    }
    None
}

struct PotentialNode<K> {
    estimated_cost: K,
    cost: K,
    index: usize,
}

impl<K: PartialEq> PartialEq for PotentialNode<K> {
    fn eq(&self, other: &Self) -> bool {
        self.estimated_cost == other.estimated_cost && self.cost == other.cost
    }
}

impl<K: PartialEq> Eq for PotentialNode<K> {}

impl<K: Ord> PartialOrd for PotentialNode<K> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<K: Ord> Ord for PotentialNode<K> {
    fn cmp(&self, other: &Self) -> Ordering {
        match other.estimated_cost.cmp(&self.estimated_cost) {
            Ordering::Equal => self.cost.cmp(&other.cost),
            s => s,
        }
    }
}
