# Repository Rules

## Development & Testing Workflows
- **Dual-Agent TDD Framework (`vci-tdd-team`)**: When requested to implement features using `vci-tdd-team` (or an Implementer–Tester setup), follow the `.agent/skills/vci-tdd-team/SKILL.md` protocol. Separate firmware implementation and test verification into distinct roles, enforce turn-based access to `/dev/ttyUSB0` and BLE, iterate until all test cases pass on physical hardware, and produce a markdown test report.

## Git Workflow
- **Always Commit and Push Changes**: Whenever files in this repository are created, modified, or deleted as part of a task, stage the changes, create a clear Git commit, and push the branch to the `origin` remote repository (`git push origin main`) before concluding the task.
