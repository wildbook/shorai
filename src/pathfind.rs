// This implementation was originally based on [the `pathfinding` crate](https://crates.io/crates/pathfinding).
// It was heavily modified to fit the needs of this project.

use indexmap::map::Entry::{Occupied, Vacant};
use num_traits::Zero;
use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::hash::Hash;
use std::iter;

use crate::FxIndexMap;

pub fn find<N, C, IN>(
    start: N,
    mut successors: impl FnMut(&N) -> IN,
    mut is_valid_move: impl FnMut(&N, &N) -> bool,
    // Used to dynamially calculate cost for arbitrary jumps.
    // It is important that uses the same calculation as `successors` does.
    mut movement_cost: impl FnMut(&N, &N) -> C,
    // Called when a jump is taken, allows making modifications to `N` before cost is calculated.
    mut take_jump: impl FnMut(&N, &mut N),
    mut heuristic: impl FnMut(&N) -> C,
    mut success: impl FnMut(&N) -> bool,
) -> Option<(Vec<N>, C)>
where
    N: Eq + Hash + Copy,
    C: Zero + Ord + Copy,
    IN: IntoIterator<Item = (N, C)>,
{
    // Set up the two main collections we'll be using.
    let mut pending = BinaryHeap::new();
    let mut visited = FxIndexMap::default();

    // Add the start node to the visited map, and a reference to it in the pending heap.
    visited.insert(start, (usize::max_value(), Zero::zero()));
    pending.push(Pending { estimated_cost: Zero::zero(), cost: Zero::zero(), index: 0, fallback: None });

    // pX = parent X - p0 = current node, p1 = parent of p0, p2 = parent of p1, etc.
    while let Some(Pending { cost, index: p0_index, fallback, .. }) = pending.pop() {
        // This isn't strictly required to be unchecked, but it helps quite a bit with performance.
        let (p0_node, &(p1_index, p0_cost)) = unsafe { visited.get_index(p0_index).unwrap_unchecked() };

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
                    add_pending(&mut visited, &mut pending, &mut heuristic, fb.parent, fb.cost, fb.node, None);
                }

                // Since the move wasn't valid we're done with this iteration.
                continue;
            }
        }

        // If the node we're currently on is considered a valid goal, we're done.
        if success(p0_node) {
            // Since we're holding the end piece we need to rebuild the path by walking the trail
            // of parent indices.

            // We'll start by building the path from the end node to the start node.
            let to_out = |(&n, _)| n;
            let parent = |&(_, &(p, _)): &_| visited.get_index(p);
            let mut path = iter::successors(visited.get_index(p0_index), parent).map(to_out).collect::<Vec<_>>();

            // We then need to reverse the path to get the path from the start node to the end node.
            path.reverse();

            // And finally, return success with the finished path and the cost of taking it.
            return Some((path, cost));
        }

        // Since our current node isn't the goal, we expand it by retrieving and registering all
        // nodes that we can get to from it.
        for (mut node, move_cost) in successors(p0_node) {
            // If our p0 is the starting node there's no p1 to jump from, so we default to a
            // pending normal move from the starting node to the successor.
            let (mut idx, mut cost, mut fallback) = (p0_index, cost + move_cost, None);

            if let Some((p1_node, &(_, p1_cost))) = visited.get_index(p1_index) {
                // Create a fallback node so we can expand into an equivalent of the second
                // branch in this match if this jump ends up being considered and is invalid.
                let backup = Fallback { parent: p0_index, cost: cost + move_cost, node };

                // We'll want to fix up the time in next_node.
                take_jump(p1_node, &mut node);

                // Calculate the actual cost of moving to there.
                let move_cost = movement_cost(p1_node, &node);

                // Use p1 as parent and skip over the p0 node entirely.
                idx = p1_index;
                cost = p1_cost + move_cost;
                fallback = Some(backup);
            }

            add_pending(&mut visited, &mut pending, &mut heuristic, idx, cost, node, fallback);
        }
    }

    // We only end up here if there's no more elements to pop and explore.
    None
}

fn add_pending<N: Eq + Hash + Copy, C: Zero + Ord + Copy>(
    visited: &mut FxIndexMap<N, (usize, C)>,
    pending: &mut BinaryHeap<Pending<C, N>>,
    mut heuristic: impl FnMut(&N) -> C,
    n_parent_idx: usize,
    cost: C,
    node: N,
    fallback: Option<Fallback<C, N>>,
) {
    let (heuristic_value, index) = match visited.entry(node) {
        Vacant(entry) => {
            let out = (heuristic(entry.key()), entry.index());
            entry.insert((n_parent_idx, cost));
            out
        }
        Occupied(mut entry) if cost < entry.get().1 => {
            let out = (heuristic(entry.key()), entry.index());
            entry.insert((n_parent_idx, cost));
            out
        }

        // If the entry is occupied with a lower cost (or same-cost) alternative we'll just
        // keep that one.
        Occupied(_) => return,
    };

    pending.push(Pending { estimated_cost: cost + heuristic_value, cost, index, fallback });
}

struct Pending<K, N> {
    estimated_cost: K,
    cost: K,
    index: usize,

    // If the pending node is not a valid move, we're going to insert a fallback node into the
    // heap of pending nodes. This is done so that we can "take" a jump without knowing if it's
    // valid and then still consider a normal move to the same node if the jump turns out to be
    // invalid.
    //
    // If you imagine that we have a path such as this:
    // +-------+
    // | P → A |
    // |     ↓ |
    // |     B |
    // +-------+
    //
    // If we're currently at A we can do a normal move to B, but we might also be able to cut out A
    // entirely and instead move from its parent node P to B directly:
    //
    // +-------+
    // | P   A |
    // |   \   |
    // |     B |
    // +-------+
    //
    // The easiest way to handle this is to simply do a "can I move P → B" check right then, and
    // if the move is valid we'll add B to the list of visited nodes with P as its parent. If the
    // move is invalid we'll instead add B with A as its parent, which is a normal A → B move.
    //
    // The above way of doing it has a major downside though. If we're currently at A and we're
    // not sure where we're going, we'll have to check if we can jump to all of the nodes that we
    // might want to jump to. Not just the ones we actually end up considering.
    //
    // If you imagine a grid where we've moved from P to A and we're now expanding A to get all
    // potential future moves, we'll have to check if we can jump to all of the nodes that we
    // might want to jump to. This ends up being a lot of extra checks. For example, if we'd have
    // a situation where A expands to B and C we'd now have to check whether or not we can jump
    // both P → B and P → C, even if we'd only ever consider one of them.
    //
    // +-----------+
    // | P → A → C |
    // |     ↓     |
    // |     B     |
    // +-----------+
    //
    // The way we solve this is that we instead register the jumps as if they're valid, but we
    // store the normal moves as fallbacks on the jumps. That results in our potential A → B and
    // A → C moves being stored as moves from P → B and P → C instead.
    //
    // Then our grid of potential paths instead looks like this:
    // +-----------+
    // | P → → → C |
    // |   \       |
    // |     B     |
    // +-----------+
    //
    // Where the P → C move contains A → C as a fallback, and the P → B move contains A → B as a
    // fallback.
    //
    // If we end up considering P → B we'll check whether or not that jump is invalid then, and if
    // it turns out that P → B is invalid, we'll read the fallback (A → B) move and then push that
    // to the heap of considered moves before we continue. That way we only have to check whether
    // or not we can take a jump if that jump is actually considered, and we don't have to pay the
    // cost of checking whether or not P → C is valid if we never end up interested in actually
    // taking that jump. We can simply pretend that it's valid, see that it's not a jump we're
    // interested in taking, and ignore whether or not we could've taken it if we wanted to.
    //
    // Since P → B is objectively better than A → B, there's no reason to push both alternatives
    // early on and rely on P → B being picked over A → B. We could do it that way too, in which
    // case we wouldn't need to store any fallback data, but there is simply no reason to consider
    // A → B while P → B is assumed to be valid.
    fallback: Option<Fallback<K, N>>,
}

#[derive(PartialEq, Eq)]
struct Fallback<K, N> {
    // Since our fallbacks always involve swapping out the parent node, we're storing the index of
    // the new parent here. This is how we keep track of what the actual fallback move is. If we
    // need to consider any other fallback cases later on it should be easy enough to modify this
    // to store the information required for that instead.
    parent: usize,

    // We don't technically need to store the cost as part of the fallback data since we're able
    // to recalculate it later, but since we're already holding the cost of this move when we're
    // constructing the fallback we might as well store it and save ourselves the cost of doing
    // the same calculation again. This also has the benefit that if the cost of the original move
    // doesn't need to be calculated, or is cheaper to calculate than a cost from any point A to
    // any point B is (like how if all successors are horizontal or diagonal the cost is static),
    // we don't have to pay the cost of the more expensive cost calculation if the fallback is
    // eventually considered.
    cost: K,

    // We're saving the node since taking a jump might modify the node before the jump it taken.
    // If that happens we'll need to restore it to the original state before taking the fallback,
    // and the easiest way to do that is simply to store the original state as part of the fallback
    // data.
    node: N,
}

impl<K: PartialEq, N: Eq> Eq for Pending<K, N> {}
impl<K: PartialEq, N: PartialEq> PartialEq for Pending<K, N> {
    fn eq(&self, other: &Self) -> bool {
        // A node also has to have the same fallback case to be considered to be the same node for
        // now. This is suboptimal, but the alternative is that we might otherwise abandon fallback
        // cases that we're not intending to abandon. This is something that we might want to look
        // at further to see if we can simplify, since that would reduce the number of considered
        // nodes at any given point in time, potentially saving us some memory.
        self.estimated_cost == other.estimated_cost && self.cost == other.cost && self.fallback == other.fallback
    }
}

impl<K: Ord, N: Eq> PartialOrd for Pending<K, N> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<K: Ord, N: Eq> Ord for Pending<K, N> {
    fn cmp(&self, other: &Self) -> Ordering {
        match other.estimated_cost.cmp(&self.estimated_cost) {
            Ordering::Equal => self.cost.cmp(&other.cost),
            s => s,
        }
    }
}
