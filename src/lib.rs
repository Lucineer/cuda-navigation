/*!
# cuda-navigation

Pathfinding and route planning.

Getting from A to B sounds simple. Add obstacles, energy budgets,
traffic, and uncertainty and it becomes a real optimization problem.
This crate handles the "where do I go and how do I get there" question.

- A* pathfinding on grid
- Waypoint management
- Obstacle avoidance
- Route cost estimation
- Path smoothing
- Replanning on obstacle discovery
*/

use serde::{Deserialize, Serialize};
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::cmp::Ordering;

/// 2D position
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Pos { pub x: i32, pub y: i32 }
impl Pos {
    pub fn new(x: i32, y: i32) -> Self { Pos { x, y } }
    pub fn manhattan(&self, other: &Pos) -> f64 { (self.x - other.x).abs() as f64 + (self.y - other.y).abs() as f64 }
    pub fn euclidean(&self, other: &Pos) -> f64 { (((self.x - other.x).pow(2) + (self.y - other.y).pow(2)) as f64).sqrt() }
    pub fn neighbors4(&self) -> Vec<Pos> { vec![Pos::new(self.x-1, self.y), Pos::new(self.x+1, self.y), Pos::new(self.x, self.y-1), Pos::new(self.x, self.y+1)] }
}

/// A node in the pathfinding graph
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NavNode {
    pub pos: Pos,
    pub walkable: bool,
    pub cost: f64,        // base traversal cost
    pub terrain_type: String,
}

/// A waypoint
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Waypoint {
    pub pos: Pos,
    pub required: bool,   // must visit
    pub name: String,
    pub estimated_cost: f64,
}

/// A planned path
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Path {
    pub waypoints: Vec<Pos>,
    pub total_cost: f64,
    pub estimated_duration_ms: u64,
    pub confidence: f64,
}

impl Path {
    pub fn new() -> Self { Path { waypoints: vec![], total_cost: 0.0, estimated_duration_ms: 0, confidence: 1.0 } }
    pub fn length(&self) -> usize { self.waypoints.len() }
    pub fn is_empty(&self) -> bool { self.waypoints.is_empty() }
}

/// Priority queue node for A*
#[derive(Clone, Debug)]
struct AStarNode { pos: Pos, g: f64, h: f64 }
impl PartialEq for AStarNode { fn eq(&self, other: &Self) -> bool { (self.g + self.h).eq(&(other.g + other.h)) } }
impl Eq for AStarNode {}
impl PartialOrd for AStarNode { fn partial_cmp(&self, other: &Self) -> Option<Ordering> { (self.g + self.h).partial_cmp(&(other.g + other.h)) } }
impl Ord for AStarNode { fn cmp(&self, other: &Self) -> Ordering { (self.g + self.h).partial_cmp(&(other.g + other.h)).unwrap_or(Ordering::Equal) } }

/// The navigation system
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Navigator {
    pub grid: HashMap<Pos, NavNode>,
    pub waypoints: Vec<Waypoint>,
    pub current_path: Option<Path>,
    pub current_index: usize,
    pub speed: f64,           // cells per second
    pub replan_count: u32,
}

impl Navigator {
    pub fn new() -> Self { Navigator { grid: HashMap::new(), waypoints: vec![], current_path: None, current_index: 0, speed: 2.0, replan_count: 0 } }

    /// Set a grid cell
    pub fn set_cell(&mut self, pos: Pos, walkable: bool, cost: f64, terrain: &str) {
        self.grid.insert(pos, NavNode { pos, walkable, cost, terrain_type: terrain.to_string() });
    }

    /// Add waypoint
    pub fn add_waypoint(&mut self, pos: Pos, name: &str, required: bool) {
        self.waypoints.push(Waypoint { pos, required, name: name.to_string(), estimated_cost: 0.0 });
    }

    /// Is a position walkable?
    fn walkable(&self, pos: &Pos) -> bool {
        self.grid.get(pos).map(|n| n.walkable).unwrap_or(true) // default walkable
    }

    /// Cost to traverse a cell
    fn cost(&self, pos: &Pos) -> f64 {
        self.grid.get(pos).map(|n| n.cost).unwrap_or(1.0)
    }

    /// A* pathfinding
    pub fn find_path(&self, start: Pos, goal: Pos) -> Path {
        if start == goal { return Path { waypoints: vec![start, goal], total_cost: 0.0, estimated_duration_ms: 0, confidence: 1.0 }; }

        let mut open: BinaryHeap<AStarNode> = BinaryHeap::new();
        let mut came_from: HashMap<Pos, Pos> = HashMap::new();
        let mut g_score: HashMap<Pos, f64> = HashMap::new();
        let mut closed: HashSet<Pos> = HashSet::new();

        g_score.insert(start, 0.0);
        open.push(AStarNode { pos: start, g: 0.0, h: start.euclidean(&goal) });

        while let Some(current) = open.pop() {
            if current.pos == goal { return self.reconstruct_path(&came_from, goal, start); }

            if closed.contains(&current.pos) { continue; }
            closed.insert(current.pos);

            for neighbor in current.pos.neighbors4() {
                if !self.walkable(&neighbor) { continue; }
                if closed.contains(&neighbor) { continue; }

                let tentative_g = *g_score.get(&current.pos).unwrap_or(&f64::MAX) + self.cost(&neighbor);
                if tentative_g < *g_score.get(&neighbor).unwrap_or(&f64::MAX) {
                    g_score.insert(neighbor, tentative_g);
                    came_from.insert(neighbor, current.pos);
                    let f = tentative_g + neighbor.euclidean(&goal);
                    open.push(AStarNode { pos: neighbor, g: tentative_g, h: f - tentative_g });
                }
            }
        }

        // No path found
        Path { waypoints: vec![], total_cost: f64::MAX, estimated_duration_ms: 0, confidence: 0.0 }
    }

    fn reconstruct_path(&self, came_from: &HashMap<Pos, Pos>, goal: Pos, start: Pos) -> Path {
        let mut path = vec![goal];
        let mut current = goal;
        while let Some(&prev) = came_from.get(&current) {
            path.push(prev);
            current = prev;
            if current == start { break; }
        }
        path.reverse();

        let total_cost: f64 = path.windows(2).map(|w| self.cost(&w[1])).sum();
        let cells = path.len().max(1) - 1;
        Path { total_cost, estimated_duration_ms: ((cells as f64 / self.speed) * 1000.0) as u64, confidence: 1.0, waypoints: path }
    }

    /// Set current path to follow
    pub fn set_path(&mut self, path: Path) { self.current_path = Some(path); self.current_index = 0; }

    /// Get next waypoint on current path
    pub fn next_waypoint(&mut self) -> Option<Pos> {
        if let Some(ref path) = self.current_path {
            if self.current_index < path.waypoints.len() {
                let wp = path.waypoints[self.current_index];
                self.current_index += 1;
                return Some(wp);
            }
        }
        None
    }

    /// Obstacle detected — replan from current position
    pub fn replan(&mut self, obstacle: Pos) {
        let current = self.current_path.as_ref().and_then(|p| p.waypoints.get(self.current_index)).copied().unwrap_or(Pos::new(0, 0));
        let goal = self.current_path.as_ref().and_then(|p| p.waypoints.last()).copied().unwrap_or(Pos::new(0, 0));

        // Mark obstacle
        if let Some(node) = self.grid.get_mut(&obstacle) { node.walkable = false; }

        let new_path = self.find_path(current, goal);
        if !new_path.is_empty() { self.set_path(new_path); }
        self.replan_count += 1;
    }

    /// Smooth path (remove unnecessary waypoints)
    pub fn smooth_path(&mut self) {
        if let Some(ref mut path) = self.current_path {
            if path.waypoints.len() <= 2 { return; }
            let mut smoothed = vec![path.waypoints[0]];
            for i in 1..path.waypoints.len() - 1 {
                let prev = smoothed.last().unwrap();
                let next = &path.waypoints[i + 1];
                // Skip if we can go directly
                if !self.walkable(&Pos::new((prev.x + next.x) / 2, (prev.y + next.y) / 2)) {
                    smoothed.push(path.waypoints[i]);
                }
            }
            smoothed.push(*path.waypoints.last().unwrap());
            path.waypoints = smoothed;
        }
    }

    /// Path following progress
    pub fn progress(&self) -> f64 {
        if let Some(ref path) = self.current_path {
            if path.waypoints.is_empty() { return 1.0; }
            self.current_index as f64 / path.waypoints.len() as f64
        } else { 0.0 }
    }

    /// Summary
    pub fn summary(&self) -> String {
        let path_len = self.current_path.as_ref().map(|p| p.waypoints.len()).unwrap_or(0);
        format!("Navigator: {} grid cells, {} waypoints, path_len={}, replans={}", self.grid.len(), self.waypoints.len(), path_len, self.replan_count)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simple_path() {
        let mut nav = Navigator::new();
        let path = nav.find_path(Pos::new(0, 0), Pos::new(3, 0));
        assert!(!path.is_empty());
        assert!(path.total_cost < f64::MAX);
    }

    #[test]
    fn test_obstacle_avoidance() {
        let mut nav = Navigator::new();
        nav.set_cell(Pos::new(2, 0), false, 0.0, "wall");
        let path = nav.find_path(Pos::new(0, 0), Pos::new(3, 0));
        assert!(!path.is_empty());
        // Should go around (2,0)
        assert!(!path.waypoints.contains(&Pos::new(2, 0)));
    }

    #[test]
    fn test_blocked_path() {
        let mut nav = Navigator::new();
        nav.set_cell(Pos::new(1, 0), false, 0.0, "wall");
        nav.set_cell(Pos::new(1, 1), false, 0.0, "wall");
        nav.set_cell(Pos::new(1, -1), false, 0.0, "wall");
        let path = nav.find_path(Pos::new(0, 0), Pos::new(2, 0));
        assert!(path.is_empty()); // no path possible
    }

    #[test]
    fn test_same_position() {
        let nav = Navigator::new();
        let path = nav.find_path(Pos::new(5, 5), Pos::new(5, 5));
        assert_eq!(path.waypoints.len(), 2);
    }

    #[test]
    fn test_path_following() {
        let mut nav = Navigator::new();
        let path = nav.find_path(Pos::new(0, 0), Pos::new(2, 0));
        nav.set_path(path);
        let w1 = nav.next_waypoint();
        assert!(w1.is_some());
        let w2 = nav.next_waypoint();
        assert!(w2.is_some());
    }

    #[test]
    fn test_replan() {
        let mut nav = Navigator::new();
        nav.set_cell(Pos::new(1, 0), true, 1.0, "floor");
        let path = nav.find_path(Pos::new(0, 0), Pos::new(3, 0));
        nav.set_path(path);
        nav.current_index = 1;
        nav.replan(Pos::new(1, 0)); // block ahead
        assert_eq!(nav.replan_count, 1);
    }

    #[test]
    fn test_terrain_cost() {
        let mut nav = Navigator::new();
        nav.set_cell(Pos::new(1, 0), true, 3.0, "mud"); // expensive
        let path = nav.find_path(Pos::new(0, 0), Pos::new(2, 0));
        assert!(path.total_cost > 2.0); // mud costs extra
    }

    #[test]
    fn test_waypoints() {
        let mut nav = Navigator::new();
        nav.add_waypoint(Pos::new(5, 5), "home", true);
        nav.add_waypoint(Pos::new(10, 10), "work", true);
        assert_eq!(nav.waypoints.len(), 2);
    }

    #[test]
    fn test_progress() {
        let mut nav = Navigator::new();
        nav.set_path(nav.find_path(Pos::new(0, 0), Pos::new(2, 0)));
        assert!(nav.progress() < 0.1);
        nav.next_waypoint(); nav.next_waypoint();
        assert!(nav.progress() > 0.5);
    }

    #[test]
    fn test_summary() {
        let nav = Navigator::new();
        let s = nav.summary();
        assert!(s.contains("0 grid cells"));
    }
}
