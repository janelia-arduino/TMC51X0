# Project Codex guidance

This file is the canonical project guidance for Codex in this repository.

## Session start

Read this file first.

If `.codex/handoff.local.md` exists, read it before planning or editing. Treat it as local, current-session context rather than permanent project policy.

If the task is multi-step, risky, or likely to span more than one session, read `.codex/PLANS.md` before making changes and keep it updated as work progresses.

## Working style

Understand the relevant code paths before editing.

Prefer the smallest correct change that satisfies the request.

Preserve existing architecture and style unless the task explicitly asks for a redesign.

When you discover assumptions or constraints that materially affect the work, record them in `.codex/PLANS.md` if the task is long-running.

## Validation

Run the smallest useful verification commands after changes.

Replace the placeholder commands below with real project commands.

- Build: `REPLACE_WITH_BUILD_COMMAND`
- Test: `REPLACE_WITH_TEST_COMMAND`
- Lint: `REPLACE_WITH_LINT_COMMAND`
- Type check: `REPLACE_WITH_TYPECHECK_COMMAND`

If a command is slow or unavailable, say what you skipped and why.

## Editing boundaries

Do not modify secrets, deployment credentials, or unrelated configuration unless the task requires it.

Avoid destructive operations unless the user explicitly asked for them.

## Plans

For substantial work, maintain `.codex/PLANS.md` as a living document:

- keep the goal, milestones, assumptions, and verification steps current
- record meaningful design decisions
- update progress before stopping
