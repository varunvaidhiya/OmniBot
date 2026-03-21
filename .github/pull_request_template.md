## Summary

<!-- What does this PR do and why? 2–4 sentences. Link the issue it resolves. -->

Closes #

---

## Type of change

<!-- Check all that apply -->

- [ ] Bug fix
- [ ] New feature
- [ ] Benchmark / research result
- [ ] Documentation
- [ ] Refactor (no behaviour change)
- [ ] Tests only
- [ ] CI / infra

## Contribution track

- [ ] Engineering
- [ ] Research / benchmarking
- [ ] Both

---

## Engineering checklist

_Skip sections that don't apply to your change._

- [ ] Tests added or updated — new code has test coverage
- [ ] CI passes locally (`colcon test` + `pytest` in affected packages)
- [ ] `CLAUDE.md` updated if I added/changed/removed a ROS 2 parameter or node
- [ ] Launch files updated if parameter names changed (watch out for the `wheel_separation_x/y` vs `wheel_separation_length/width` mismatch)
- [ ] No new `# noqa` suppressions without an inline explanation
- [ ] No hardcoded paths or usernames (e.g. `/home/varunvaidhiya/...`)

---

## Benchmark / research checklist

_Fill in if this PR includes or references experimental results._

- [ ] Experiment was proposed and discussed in an issue before running (link above)
- [ ] Dataset is in LeRobot format and follows naming convention `<user>/omnibot-<task>-<n>demos`
- [ ] Results are from actual experiments (not extrapolated or fabricated) — see [Code of Conduct](../CODE_OF_CONDUCT.md)
- [ ] `data_engine/schema/constants.py` specs followed for state/action dimensions

### Results table

<!-- Delete this section if not a benchmark PR -->

| Metric | Value |
|---|---|
| Task success rate | |
| Grasp success rate | |
| Mean inference latency | |
| Number of eval episodes | |
| Hardware config | |
| Model checkpoint | |
| Dataset repo ID | |

---

## Screenshots / recordings

<!-- Attach GIFs, plots, or RViz screenshots if relevant. -->

---

## Notes for reviewer

<!-- Anything the reviewer should pay particular attention to. -->
