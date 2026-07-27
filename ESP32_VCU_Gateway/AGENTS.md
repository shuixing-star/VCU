---
name: agent-guided-engineering
description: Maintainable collaboration workflow for using Codex/agents in complex, unfamiliar, or safety-sensitive codebases. Use when the user wants AI-assisted development without unreviewable generated code, needs to understand a large project before changing it, has an AI-generated task list that must be converted into safe engineering steps, or asks for a working method/process for agent collaboration, incremental implementation, architecture mapping, codebase comprehension, or maintainability-first development.
---

# Agent Guided Engineering

Use this skill to turn agent work from "generate a lot of code" into a controlled engineering collaboration. The goal is to increase the user's ownership of the system while still using Codex for reading, explanation, implementation, and verification.

## Core Posture

- Treat the agent as a senior pair engineer, not an autonomous contractor.
- Prefer small, reviewable changes over large generated implementations.
- Preserve the codebase's existing architecture, naming, ownership boundaries, and failure-handling style.
- Explain enough for the user to maintain the result later.
- Separate system understanding, design boundaries, implementation, and validation into distinct steps.
- Avoid hiding complexity inside generated code. Move complexity into explicit design notes, clear interfaces, and focused tests.

## Collaboration Workflow

### 1. Build A System Map

Before editing unfamiliar or complex code, inspect the repository and explain:

- The main runtime layers and their responsibilities.
- The relevant modules/files for the requested behavior.
- The existing data/control flow.
- The current extension points and ownership boundaries.
- Any dirty worktree changes that may affect the task.

Keep the map practical. It should help the user answer "where does this behavior live?" and "where should this change belong?"

### 2. Convert The Task Into Design Boundaries

For each requested or AI-generated task, define:

- What module owns the behavior.
- What data enters and leaves that module.
- What the task explicitly does not do.
- What assumptions are being made.
- What failure modes matter.
- How the result can be verified.

If a task mixes concerns, split it before implementation. For embedded or protocol work, keep low-level deterministic encoding/parsing separate from scheduling, policy, and state machines.

### 3. Choose A Thin Slice

Implement the smallest useful vertical or horizontal slice:

- Prefer pure functions and narrow adapters first.
- Wire into runtime behavior only after the low-level surface is clear.
- Avoid broad refactors unless the current design makes the task unsafe or impossible.
- Do not combine protocol definition, business policy, UI/configuration, and build-system cleanup in one opaque change.

For large goals, provide a sequence of slices and mark the current slice.

### 4. Edit With A Maintenance Contract

Before editing, state what files will change and why. During implementation:

- Follow local patterns before inventing new abstractions.
- Keep naming boring and searchable.
- Add comments only where they preserve non-obvious design intent.
- Keep generated-looking boilerplate out of hand-maintained code where possible.
- Do not silently introduce global state, magic IDs, hidden timing assumptions, or cross-layer dependencies.

### 5. Verify The Slice

Use verification scaled to risk:

- Compile or run focused tests when available.
- For pure protocol or parser code, create round-trip or known-vector checks when practical.
- For integration code, verify registration, call signatures, and failure paths.
- If full validation is unavailable, say exactly what was not verified and what remains risky.

### 6. Explain The Result Back To The User

After changes, summarize in maintenance terms:

- What changed.
- Which module owns the new responsibility.
- How data flows through the new code.
- What assumptions are now encoded.
- What was verified.
- What the next smallest safe step is.

The user should leave each step with a clearer mental model, not only a diff.

## Review Questions

Ask these before accepting an agent-generated task or code block:

- Does this task have one owner, or is it mixing layers?
- Could a future maintainer find the behavior by searching obvious names?
- Is the policy in the layer where policy already lives?
- Is low-level data encoding deterministic and testable?
- Are timing, IDs, units, scaling, and endian assumptions explicit?
- Can the user explain how to debug this if it fails on hardware?
- Is there a smaller slice that proves the idea with less blast radius?

## Communication Pattern

Use concise updates while working:

- "I am mapping the existing ownership before touching code."
- "This task belongs in X; Y should stay policy-free."
- "I am implementing only the pack/unpack layer first so it can be validated independently."
- "This part is still a stub; the next safe step is to wire it through the adapter."

Avoid overconfident claims. If the design depends on missing hardware, external docs, or unverified behavior, name that uncertainty plainly.

## Output Shape

For substantial work, use this structure:

1. System map
2. Task boundary
3. Thin-slice plan
4. Implementation
5. Verification
6. Maintenance summary

For smaller work, compress the same thinking into a short paragraph before edits and a short close-out after verification.
