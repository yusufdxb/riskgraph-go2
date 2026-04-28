# Codex Audit Prompt — Final (pre-handoff)

**Run with:** `codex exec`.

**Repo state:** v0.1.0. All offline validation green; ROS-side end-to-end smoke green on workstation; hardware proof boundary explicit in docs.

## Task

This is the last review before this repo is presented to a thesis advisor and/or robotics reviewer. Look for the things that get the project rejected: overclaims, missing rigor, silent failures, presentation issues.

## Specific checks

1. **Truthful README.md.** Walk every paragraph; flag any sentence a reviewer would call out as overclaim. Particularly: validation status, hardware proof boundary, novelty.
2. **Reproducibility.** `git clone` + `./scripts/run_offline_demo.sh` from a fresh shell. Does it work end-to-end without prerequisites that aren't documented? What does fail look like (missing pyyaml, missing python3, etc.)?
3. **Documentation rot.** Any code-doc divergence introduced after architecture audit was addressed? Especially in `docs/architecture.md` flow diagrams — do all referenced node names and topics still match `setup.py` entry points and launch files?
4. **Audit log completeness.** `docs/audits/audit_log.md` should record findings from architecture audit, midpoint audit, and at minimum any internal pre-final review. Anything outstanding without a justification for not fixing?
5. **Bus-factor.** Could a new contributor pick this up cold? Is there a clear "where to start" path?
6. **Edge cases in published artefacts.** `demo_results.json`, `*.sqlite` — any of these accidentally checked in despite `.gitignore`?
7. **Final novelty stance.** Does the README's novelty paragraph still match `docs/prior_art.md`? Is anything more conservative needed in light of the architecture / midpoint audits?

## Output format

For each finding:

```
- severity: [HIGH|MED|LOW]
- file: <path>:<line range>
- finding: <one sentence>
- recommendation: <one sentence>
```

End with a single "ship/no-ship" verdict line and the top 3 reasons.
