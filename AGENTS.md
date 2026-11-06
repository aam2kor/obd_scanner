# Repository Rules

## Development & Testing Workflows
- **4-Tier Engineering Team Framework (`vci-tdd-team`)**: When requested to implement features using `vci-tdd-team`, follow the `.agent/skills/vci-tdd-team/SKILL.md` protocol. Separate responsibilities across four distinct roles:
  1. **Technical Delivery Manager**: Interacts with the user, decomposes tasks, provides upfront time estimates, tracks elapsed time, monitors iteration counts, escalates anomalies via a circuit-breaker, and coordinates delivery.
  2. **Firmware Implementer**: Develops optimized, deterministic C firmware (FreeRTOS, NimBLE, CAN, D-PDU, deterministic static memory, cache/IRAM optimization).
  3. **Code Reviewer ("Uncle Bob")**: Reviews all C code changes *before* testing to enforce the Top-3 Embedded Clean Coding Guidelines (Single Level of Abstraction & Focused Functions, Intention-Revealing Naming & Zero Magic Numbers, and Defensive Contracts & Explicit Ownership). Code review must pass before hardware testing begins.
  4. **System QA Test Architect**: Performs strictly system-level black-box verification on physical hardware across BLE, UART, and CAN interfaces, iterating with the Implementer until 100% of test cases pass with full evidence.

## Git Workflow
- **Always Commit and Push Changes**: Whenever files in this repository are created, modified, or deleted as part of a task, stage the changes, create a clear Git commit, and push the branch to the `origin` remote repository (`git push origin main`) before concluding the task.
