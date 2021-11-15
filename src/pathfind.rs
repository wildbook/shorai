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
    // Used to estimate the distance from a specific point to the goal.
    heuristic: FH,
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
    FH: Fn(&N) -> C,
    FS: FnMut(&N) -> bool,
{
    let mut pending = BinaryHeap::new();
    let mut visited: FxIndexMap<N, (usize, C)> = FxIndexMap::default();

    // Used below to register available nodes as they're discovered.
    let add_pending_node = |visited: &mut _, pending: &mut _, n_parent_idx, n_cost, node, fallback| {
        let visited: &mut FxIndexMap<N, (usize, C)> = visited;
        let pending: &mut BinaryHeap<PotentialNode<C, N>> = pending;

        let (n_heuristic, n_node_idx) = match visited.entry(node) {
            Vacant(entry) => {
                let out = (heuristic(entry.key()), entry.index());
                entry.insert((n_parent_idx, n_cost));
                out
            }
            Occupied(mut entry) if n_cost < entry.get().1 => {
                let out = (heuristic(entry.key()), entry.index());
                entry.insert((n_parent_idx, n_cost));
                out
            }
            Occupied(_) => return,
        };

        pending.push(PotentialNode { estimated_cost: n_cost + n_heuristic, cost: n_cost, index: n_node_idx, fallback });
    };

    // Add the start node to the visited map, and a reference to it in the pending heap.
    visited.insert(start, (usize::max_value(), Zero::zero()));
    pending.push(PotentialNode { estimated_cost: Zero::zero(), cost: Zero::zero(), index: 0, fallback: None });

    // pX = parent X - p0 = current node, p1 = parent of p0, p2 = parent of p1, etc.
    while let Some(PotentialNode { cost, index: p0_index, fallback, .. }) = pending.pop() {
        let (p0_node, &(p1_index, p0_cost)) = visited.get_index(p0_index).unwrap();

        // We may have inserted a node several time into the binary heap if we found a better way
        // to access it since. If that's the case and the existing node is better than the current
        // one, we're not interested in evaluating this one.
        if p0_cost < cost {
            continue;
        }

        // This is only ever *not* hit for the first node, since it's the only one without a parent.
        if let Some((p1_node, _)) = visited.get_index(p1_index) {
            // Ensure that the move from the parent node to the node we're at is actually valid.
            //
            // This is done here and not where the node is added since that allows us to pretend
            // that all moves are valid until we're actually considering moving to them. That saves
            // us from having to check whether we're actually able to move to nodes we never end up
            // considering / visiting.
            if !is_valid_move(p1_node, p0_node) {
                // If this node has a fallback node defined we want to register that fallback node
                // as a potential node. We could've also registered this node as pending already
                // when we first found it, but since the node we're currently on is objectively
                // better if it can be taken we can defer it until now and avoid pushing more nodes
                // than necessary to the pending heap.
                if let Some(fb) = fallback {
                    add_pending_node(&mut visited, &mut pending, fb.parent, fb.cost, fb.node, None);
                }

                // Since the move wasn't valid we're done with this node.
                continue;
            }
        }

        if success(p0_node) {
            let path = reverse_path(&visited, |&(p, _)| p, p0_index);
            return Some((path, cost));
        }

        for (mut node, move_cost) in successors(p0_node) {
            match visited.get_index(p1_index) {
                // If the parent has a parent, assume we can jump from that directly to this node
                // and set up a fallback for if we can't.
                Some((p1_node, &(_, p1_cost))) => {
                    let fallback = FallbackNode { parent: p0_index, cost: cost + move_cost, node };

                    // We'll want to fix up the time in next_node.
                    take_jump(p1_node, &mut node);

                    let move_cost = movement_cost(p1_node, &node);

                    // Use p1 as parent and skip over the p0 node entirely.
                    add_pending_node(&mut visited, &mut pending, p1_index, p1_cost + move_cost, node, Some(fallback))
                }

                // If the parent doesn't have a parent there's nothing special to be done so we just add the node.
                _ => add_pending_node(&mut visited, &mut pending, p0_index, cost + move_cost, node, None),
            };
        }
    }
    None
}

struct PotentialNode<K, N> {
    estimated_cost: K,
    cost: K,
    index: usize,

    fallback: Option<FallbackNode<K, N>>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct FallbackNode<K, N> {
    parent: usize,
    cost: K,
    node: N,
}

impl<K: PartialEq, N: PartialEq> PartialEq for PotentialNode<K, N> {
    fn eq(&self, other: &Self) -> bool {
        self.estimated_cost == other.estimated_cost && self.cost == other.cost && self.fallback == other.fallback
    }
}

impl<K: PartialEq, N: Eq> Eq for PotentialNode<K, N> {}

impl<K: Ord, N: Eq> PartialOrd for PotentialNode<K, N> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<K: Ord, N: Eq> Ord for PotentialNode<K, N> {
    fn cmp(&self, other: &Self) -> Ordering {
        match other.estimated_cost.cmp(&self.estimated_cost) {
            Ordering::Equal => self.cost.cmp(&other.cost),
            s => s,
        }
    }
}
