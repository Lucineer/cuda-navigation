# cuda-navigation

Pathfinding — A* on grid, waypoint management, obstacle avoidance, replanning (Rust)

Part of the Cocapn fleet — a Lucineer vessel component.

## What It Does

### Key Types

- `Pos` — core data structure
- `NavNode` — core data structure
- `Waypoint` — core data structure
- `Path` — core data structure
- `Navigator` — core data structure

## Quick Start

```bash
# Clone
git clone https://github.com/Lucineer/cuda-navigation.git
cd cuda-navigation

# Build
cargo build

# Run tests
cargo test
```

## Usage

```rust
use cuda_navigation::*;

// See src/lib.rs for full API
// 10 unit tests included
```

### Available Implementations

- `Pos` — see source for methods
- `Path` — see source for methods
- `PartialEq for AStarNode` — see source for methods
- `Eq for AStarNode` — see source for methods
- `PartialOrd for AStarNode` — see source for methods
- `Ord for AStarNode` — see source for methods

## Testing

```bash
cargo test
```

10 unit tests covering core functionality.

## Architecture

This crate is part of the **Cocapn Fleet** — a git-native multi-agent ecosystem.

- **Category**: other
- **Language**: Rust
- **Dependencies**: See `Cargo.toml`
- **Status**: Active development

## Related Crates


## Fleet Position

```
Casey (Captain)
├── JetsonClaw1 (Lucineer realm — hardware, low-level systems, fleet infrastructure)
├── Oracle1 (SuperInstance — lighthouse, architecture, consensus)
└── Babel (SuperInstance — multilingual scout)
```

## Contributing

This is a fleet vessel component. Fork it, improve it, push a bottle to `message-in-a-bottle/for-jetsonclaw1/`.

## License

MIT

---

*Built by JetsonClaw1 — part of the Cocapn fleet*
*See [cocapn-fleet-readme](https://github.com/Lucineer/cocapn-fleet-readme) for the full fleet roadmap*
