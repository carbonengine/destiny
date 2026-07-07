# ECS Conversion Execution Plan for Destiny

**Branch:** `ecs-conversion-plan`
**Goal:** Introduce a high-performance internal data-oriented / ECS-style core while preserving 100% of external interfaces.
**Primary Objective:** Reduce TIDI by significantly improving simulation throughput under high entity load.

## 1. Executive Summary

This plan outlines a low-risk, high-reward migration path to replace the internal implementation of Destiny's simulation engine with a modern, cache-efficient, data-oriented architecture (inspired by ECS principles) without changing any interfaces used by other Carbon modules (Scheduler, IO, Networking, Database, Python server logic, etc.).

The new core will live behind the existing `IEveBallpark` and `Ballpark` facade. All Python bindings, state serialization, client interpolation, and scheduling interactions remain unchanged.

**Key Deliverable:** A branch-ready implementation plan with defined benchmark comparisons that demonstrate clear superiority in tick time, memory efficiency, and entity scalability.

## 2. Current Problems (Why Change?)

- `Ball` is a massive god-object (hundreds of members, mixed concerns).
- Poor cache locality during evolve and collision phases.
- Expensive entity creation/destruction (especially Mini* primitives for drones/missiles).
- Difficult to parallelize safely.
- These factors contribute directly to TIDI under heavy fleet fights and high entity counts.

## 3. Proposed Architecture

**Core Principle:** Stable Facade + High-Performance Internal Core

- `Ballpark` / `IEveBallpark` remains the public interface (no changes to method signatures or behavior).
- Internally, replace the heavy `Ball` + `Partition` implementation with:
  - Lightweight entities (IDs + generation).
  - Component data stored in contiguous arrays / archetype chunks.
  - Pure data-oriented Systems for evolution, collision, proximity, etc.
  - Data-oriented version of the existing hierarchical multi-level grid spatial system.

This keeps all external modules (Scheduler, networking, IO, database operations) completely unaffected.

## 4. Phased Execution Plan

### Phase 0: Foundation (1-2 weeks)
- Create this branch (`ecs-conversion-plan`).
- Add feature flag `DESTINY_USE_NEW_CORE`.
- Set up benchmarking harness (extend `tools/benchmark`).
- Duplicate current `Ballpark` implementation as baseline.
- Define component data structures extracted from `Ball.h`.

### Phase 1: Internal Core Skeleton (3-4 weeks)
- Introduce minimal `SimulationCore` using custom data-oriented design (or EnTT if accepted).
- Implement basic entity creation, Transform + PhysicsProperties components.
- Implement initial `EvolveSystem` (integration loop).
- Wire basic path through `Ballpark` facade behind the feature flag.

### Phase 2: Spatial & Collision Systems (3-4 weeks)
- Port/adapt the hierarchical grid (`Partition` / `Box`) to operate on component data.
- Implement data-oriented broad-phase and narrow-phase collision.
- Add ProximitySystem.

### Phase 3: Full Behavioral Migration (4-6 weeks)
- Migrate mode handling (Goto, Follow, Orbit, Missile, Stop, etc.).
- Migrate Mini-shape / compound collision handling (using child entities or specialized components).
- Migrate formation, proximity sensors, ownership, etc.

### Phase 4: Python & State Compatibility (2-3 weeks)
- Ensure all existing Python methods and state serialization produce identical output.
- Add any necessary bulk query helpers (additive only).
- Validate client interpolation path.

### Phase 5: Benchmarking & Validation (ongoing + dedicated 2 weeks)
- Run head-to-head comparisons (see Benchmark Strategy below).
- Determinism verification between old and new paths.
- Performance profiling and tuning.

### Phase 6: Cutover & Cleanup (2-3 weeks)
- Make new core the default.
- Deprecate/remove old heavy `Ball` / `Partitionable` code.
- Clean up adapter layer.

**Total Estimated Timeline:** 15-22 weeks for a production-ready branch (depending on team size and parallelization).

## 5. Interface Stability Guarantees

**Zero breaking changes** to any interface used by other modules:

- `IEveBallpark` method signatures and semantics unchanged.
- All Python bindings (`Py*` methods, Blue exposure) remain identical.
- State serialization / delta format unchanged (clients see no difference).
- `ClientBall` interpolation continues to work without modification.
- Scheduler integration points untouched.
- Networking, IO, and database layers see the same `Ballpark` behavior.
- Existing Python server code (`python/destiny/net/server/`) requires no changes.

The only visible difference will be improved performance under load.

## 6. Benchmark Strategy (Proof of Superiority)

All benchmarks will be run on both the old and new implementation using the existing `tools/benchmark` infrastructure, extended as needed.

### Core Metrics
- **Tick time vs Entity Count** (primary TIDI indicator)
- **Memory usage** (peak and steady-state)
- **Entity creation/destruction cost** (especially drone/missile spam scenarios)
- **Collision query performance** (broad + narrow phase)
- **Cache miss rates** (where measurable via profiling tools)
- **Determinism hash** (full state comparison between old and new paths)

### Key Scenarios
1. Large fleet fight simulation (thousands of ships + projectiles)
2. High drone/missile spam (massive transient entity creation)
3. Dense collision scenarios (many overlapping objects)
4. Mixed load (players + NPCs + wrecks + effects)
5. Long-running stability test (hours of simulation)

### Comparison Deliverables
- Graphs: Tick time vs entity count (old vs new)
- Tables: Memory usage and allocation counts
- Latency distributions for critical operations
- Determinism verification report
- Profiling flame graphs (hot paths)

**Success Criteria:**
- New core shows **at least 2-3x improvement** in tick time at high entity counts.
- Memory usage per entity significantly lower.
- No regression in determinism or correctness.
- All existing tests pass identically.

## 7. Risks and Mitigations

- **Determinism regression** → Side-by-side execution with hash comparison on every evolve.
- **Interface drift** → Strict adapter layer + comprehensive Python test suite as contract.
- **Performance regression during migration** → Feature flag + ability to instantly switch back.
- **Complexity** → Incremental system-by-system migration.
- **Team familiarity** → Start with focused custom data-oriented core rather than full general ECS.

## 8. Success Criteria

- New internal core delivers measurable reduction in simulation tick time under heavy load.
- All external interfaces and behaviors remain 100% compatible.
- Existing test suite passes without modification.
- Clear benchmark data showing superiority over current architecture.
- Branch is production-ready for internal evaluation by Fenris Creations.

## 9. Next Steps

1. Review and approve this plan.
2. Begin Phase 0 implementation on this branch.
3. Extend benchmark harness with the defined scenarios.
4. Implement minimal `SimulationCore` skeleton.
5. Schedule regular progress reviews with benchmark results.

---

**Document Version:** 1.0
**Created for:** sfwhite/destiny-mcp fork
**Purpose:** Provide Fenris Creations / CCP with a clear, low-risk path to a significantly more performant Destiny simulation engine.