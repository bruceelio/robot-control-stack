## Robot Control Stack

A modular, **simulation-first robot control architecture** designed to scale  
from development and testing to real hardware.

This repository focuses on:

- Clear separation of concerns
- Safe iteration via simulation
- Predictable, testable robot behavior
- Explicit, auditable configuration

---

## Documentation

Start here depending on what you want to understand or change:

- 🧠 **Architecture overview**  
  System structure, layers, and design rules  
  → [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)

- 🔁 **Runtime flow**  
  What happens from startup to execution  
  → [docs/RUNTIME_FLOW.md](docs/RUNTIME_FLOW.md)

- 🧩 **Configuration system**  
  How robot, arena, and strategy inputs resolve into a single config  
  → [config/README.md](config/README.md)

- 🏷️ **Versioning & release policy**  
  How versions are defined, tagged, and used  
  → [docs/VERSIONING.md](docs/VERSIONING.md)

- 🧪 **Testing & diagnostics**  
  How tests and diagnostics fit into the system safely  
  → [docs/TESTING_AND_DIAGNOSTICS.md](docs/TESTING_AND_DIAGNOSTICS.md)

- 🛠️ **Development workflow**  
  Setup, simulation, and iteration workflow  
  → [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md)

---

## Quick Start (Simulation)

1. Install dependencies
2. Run the simulator
3. Observe logs and behavior

Detailed setup and workflow instructions live in:  
→ [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md)

---

## Project Status

This project is under active development.

- APIs and configuration may evolve
- Breaking changes are expected prior to `v1.0.0`
- Tags mark known-good milestones

Until `v1.0.0`, stability is indicated by **tags**, not the `main` branch.
