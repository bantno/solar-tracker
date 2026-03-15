# CLAUDE.md
# Based on Boris Cherny's (Creator of Claude Code) viral workflow tips

---

## Workflow Orchestration

### 1. Plan Mode Default
- Enter plan mode for ANY non-trivial task (3+ steps or architectural decisions)
- If something goes sideways, STOP and re-plan immediately — don't keep pushing
- Use plan mode for verification steps, not just building
- Write detailed specs upfront to reduce ambiguity
- Pattern: have one Claude draft the plan, then have a second Claude review it as a "staff engineer"

### 2. Subagent Strategy
- Use subagents liberally to keep the main context window clean
- Offload research, exploration, and parallel analysis to subagents
- For complex problems, throw more compute at it via subagents
- One task per subagent for focused execution

### 3. Self-Improvement Loop
- After ANY correction from the user: update this CLAUDE.md with the pattern
- Write rules for yourself that prevent the same mistake
- Ruthlessly iterate on these rules until mistake rate drops
- Review relevant lessons at the start of each session

### 4. Verification Before Done
- Never mark a task complete without proving it works
- Diff behavior between main and your changes when relevant
- Ask yourself: "Would a staff engineer approve this?"
- Run tests, check logs, demonstrate correctness

### 5. Demand Elegance
- For non-trivial changes: pause and ask "is there a more elegant way?"
- If a fix feels hacky: "Knowing everything I know now, implement the elegant solution"
- Skip this for simple, obvious fixes — don't over-engineer
- Challenge your own work before presenting it

### 6. Autonomous Bug Fixing
- When given a bug report: just fix it. Don't ask for hand-holding
- Point at logs, errors, failing tests — then resolve them
- Go fix failing CI tests without being told how

---

## Parallelism & Sessions

- Run 3–5 Claude sessions at once, one per task
- Use `git worktree` so each session has its own isolated working directory
- Keep a dedicated "analysis" worktree for log reading / BigQuery-style investigation
- Use `--teleport` to hand off sessions between local terminal and claude.ai/code
- Use system notifications (e.g., iTerm2) to know when a session needs input

---

## Slash Commands & Skills

- If you do a workflow more than once a day, make it a slash command
- Store commands in `.claude/commands/` and commit them to git so the whole team benefits
- Include inline bash in commands to precompute context (e.g., `git status`) and reduce back-and-forth
- Example commands: `/commit-push-pr`, `/techdebt`, `/code-simplifier`

---

## Task Management

1. **Plan First**: Write a plan to `tasks/todo.md` with checkable items
2. **Verify Plan**: Check in before starting implementation
3. **Track Progress**: Mark items complete as you go
4. **Explain Changes**: Provide a high-level summary at each step
5. **Document Results**: Add a review section to `tasks/todo.md` when done
6. **Capture Lessons**: Update this `CLAUDE.md` after every correction

---

## Hooks & Automation

- Use a `PostToolUse` hook to run formatters automatically after file edits
- Pre-allow common safe bash commands via `/permissions` instead of using `--dangerously-skip-permissions`
- Share permissions config via `.claude/settings.json` checked into git

---

## Core Principles

- **Simplicity First**: Make every change as simple as possible. Minimal impact on the codebase.
- **No Laziness**: Find root causes. No temporary fixes. Senior developer standards.
- **Minimal Impact**: Changes should only touch what's necessary. Avoid introducing bugs.
- **Compound Knowledge**: Every mistake → a new rule. Every rule → fewer future mistakes.

---

## Prompting Tips

- **Challenge Claude**: Ask it to justify changes and prove they work
- **Fresh rewrites**: If a fix is mediocre, ask "Scrap it — implement the elegant solution"
- **Detailed specs**: Remove ambiguity before handing off work; specificity improves autonomy
- **After any correction**: End with "Update CLAUDE.md so you don't make that mistake again"

---

## Embedded / Architecture Lessons

- **Use abstract interfaces for HAL modules** — enables mocking for desktop tests, dependency injection, and driver swaps. Don't default to free functions even in embedded contexts.
- **Design swappable seams for algorithms** — use abstract interfaces (e.g., ISolarModel) when the implementation may change. Keeps core logic decoupled from specific models/libraries.
- **Prefer hand-rolled BT over library for resource-constrained MCUs** — BehaviorTree.CPP is heavy (STL, dynamic allocation). A ~150-line hand-rolled BT gives the same architectural benefits without the embedded compatibility risk.
- **Consider behavior tree vs state machine early** — BTs handle layered priorities and concurrent monitoring more naturally than FSMs. Decide control architecture before scaffolding.
- **Use Servo, not PWMServo, on Teensy 4.0 for microsecond control** — PWMServo only exposes `write(angle)`. The standard `Servo` library provides `writeMicroseconds()` needed for ESC pulse-width control.

---

*This file is a living document. Update it whenever Claude makes a mistake or you discover a better pattern.*
*Source: Boris Cherny's viral X thread (Jan–Feb 2026) + Claude Code team tips*
