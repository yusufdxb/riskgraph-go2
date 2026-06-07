# RiskGraph-Go2 v0.1.0: Hardware Verification Checklist

Single source of truth for the CaresLab session that flips v0.1.0 from
`hw-unverified` to `hw-verified`. Cross-references:
`docs/hardware_integration.md` (wiring), `docs/validation.md` (what is
already proven offline), `tests/hw/` (the runnable harness).

This is a checklist, not a tutorial. If you have not read
`docs/hardware_integration.md` first, stop and read it.

---

## 0. What v0.1.0 actually claims, by category

Pulled from `docs/validation.md` for cross-reference. Anything in the
**hardware-dependent** column is what this checklist exists to verify.

| Claim                                                             | Category            | Verifies in this session? |
|-------------------------------------------------------------------|---------------------|---------------------------|
| Pure-Python core (events, segments, store, scoring, explainer)    | offline / verified  | no, already green         |
| Soft-import adapters (HAVE_GO2_MSGS true/false arms)              | offline / verified  | no, already green         |
| Cross-run mock-pose memory in unit tests                          | offline / verified  | no, already green         |
| `colcon build` of all 7 packages                                  | offline / verified  | no, already green         |
| ROS in-process smoke (`scripts/ros_end_to_end_check.py`)          | offline / verified  | re-run as sanity (Step 2) |
| Adapters subscribe to live `/go2/safety_alert` etc. with right QoS | hardware-dependent | yes (Step 4)              |
| Live `RiskEvent` flow → SQLite write                              | hardware-dependent  | yes (Step 5)              |
| `/riskgraph/score_routes` answers correctly under live data       | hardware-dependent  | yes (Step 5)              |
| Cross-run memory on Jetson NVMe across a process restart          | hardware-dependent  | yes (Step 6)              |
| Frame_id / TF semantics for live events                           | hardware-dependent  | partial - documented gap (Step 7) |
| `length_m` field decoration is harmless on the wire               | hardware-dependent  | yes (passive, in bag)     |

Everything else (closed-loop motion, weight tuning, user-study evidence)
is explicitly OUT of scope for this session.

---

## 1. What to bring

- Unitree Go2 EDU with its onboard Jetson Orin NX 16 GB powered and on the
  lab subnet.
- A workstation that can `ssh unitree@<jetson>` and reach the same
  ROS_DOMAIN_ID. (You can also run everything on the Jetson via a tmux
  session if SSH X-forwarding is unavailable.)
- A clear floor zone for the dog. The scenario does NOT require closed-loop
  motion; the dog can stand on a stand.
- The repos sourced into one workspace: this repo + `GO2-seeing-eye-dog`
  (for `go2_msgs`) + `helix` (for `helix_msgs`). Either or both upstream
  packages can be absent, the affected adapter just no-ops, which is
  itself part of what we want to confirm.

---

## 2. Pre-flight (workstation, before the dog is even on)

```bash
cd ~/Projects/personal/riskgraph-go2
./scripts/run_tests.sh
./scripts/run_offline_demo.sh
python3 scripts/ros_end_to_end_check.py    # needs ROS sourced
```

Expected: 71 pytest tests pass, offline demo writes `demo_results.json`
with `chosen=LONG`, in-process smoke prints `VERDICT: PASS`.

If any of these are red on the workstation, **do not** proceed to lab. The
hardware session is not the place to debug pure-software regressions.

---

## 3. Build on the Jetson

```bash
ssh unitree@<jetson>
cd ~/riskgraph-go2          # or wherever the repo is checked out

source /opt/ros/humble/setup.bash
source ~/workspace/GO2-seeing-eye-dog/install/setup.bash    # if present
source ~/workspace/helix/install/setup.bash                  # if present

colcon build --symlink-install
source install/setup.bash

ros2 interface list | grep riskgraph
```

Expected output: 7 messages + 2 services listed (`RiskEvent`, `RiskFactor`,
`Route`, `RouteSegment`, `RouteScore`, `RouteScoreArray`, `RouteExplanation`,
`ScoreRoutes.srv`, `QuerySegmentRisk.srv`).

Capture the build log to `tests/hw/runs/jetson_build.log`.

---

## 4. Decide the SQLite store path and seed it

The default config path (`/tmp/riskgraph_store.sqlite`) does NOT survive
reboot. For the cross-run claim to mean anything, you must point the launch
at an NVMe-backed file. Recommended:

```bash
mkdir -p ~/.local/share/riskgraph
export STORE=~/.local/share/riskgraph/hw_scenario.sqlite
rm -f "$STORE"          # start hermetic for this session
```

Override the launch params via a small YAML:

```yaml
# /tmp/hw_overrides.yaml
riskgraph_memory:
  ros__parameters:
    store_path: "/home/unitree/.local/share/riskgraph/hw_scenario.sqlite"
    decay_half_life_s: 7200.0
riskgraph_planner:
  ros__parameters:
    store_path: "/home/unitree/.local/share/riskgraph/hw_scenario.sqlite"
    weight_geometry: 1.0
    weight_semantic: 1.0
    weight_risk: 4.0
    decay_half_life_s: 7200.0
riskgraph_explainer:
  ros__parameters:
    store_path: "/home/unitree/.local/share/riskgraph/hw_scenario.sqlite"
```

(See `docs/hardware_integration.md` "Persistence on Jetson" for rationale.)

---

## 5. Phase 1 - live event ingestion + score

**Terminal A (Jetson):** start the integration launch with all adapters on.

```bash
ros2 launch riskgraph_bringup integration.launch.py \
    enable_safety_adapter:=true \
    enable_helix_adapter:=true \
    enable_tactile_adapter:=true \
    --params-file /tmp/hw_overrides.yaml
```

Watch for: each node prints `ready, store_path=...`. The two missing-msgs
adapters (if any) print their no-op stderr line and exit cleanly, that's
the expected behavior and is not a failure.

**Terminal B (Jetson):** sanity-check the topic graph.

```bash
ros2 topic list | grep -E "riskgraph|go2/safety_alert|helix/faults|tactile/slip_state"
ros2 service list | grep riskgraph
```

You should see the three RiskGraph topics
(`/riskgraph/risk_events`, `/riskgraph/route_scores`, `/riskgraph/explanations`)
and the two services. Whether the upstream topics show up depends on what
else is running on the dog; capture the list either way.

**Terminal C (Jetson):** run the harness.

```bash
cd ~/riskgraph-go2
./tests/hw/run_scenario.sh --phase one
```

This script:

1. Starts a `ros2 bag record` of all relevant topics into
   `tests/hw/runs/<ts>/bag/run/`.
2. Publishes 3 synthetic `RiskEvent` messages (slip, severity 0.9) at
   map-frame `(2, 0)`.
3. Calls `/riskgraph/score_routes` with two candidate routes:
   `short_glossy` (passes through `(0,0) → (4,0)`) and
   `long_safe` (passes through `(0,2) → (4,2)`).
4. Asserts `chosen_route_id == "long_safe"` and the explanation cites
   `>=1` evidence event id.

**Pass criteria:**
- `verdict.json.phases[0].pass == true`
- `verdict.json.phases[0].chosen_route_id == "long_safe"`
- `verdict.json.phases[0].evidence_event_ids` includes at least one of
  `hw_slip_0..2`.
- The bag contains traffic on `/riskgraph/risk_events` and
  `/riskgraph/route_scores` (sanity-check with `ros2 bag info bag/run`).

**Fail behaviors and what they mean:**
- `error: service did not appear` → planner not running, or the launch
  failed silently. Check terminal A.
- `chosen=short_glossy` → events did not land on segment `hw_glossy`.
  Likely the memory node has no segments registered, so the spatial join
  returns nothing. KNOWN GAP for v0.1.0: the integration launch does not
  yet seed segments. See `tests/hw/scenario_glossy_loop.py` GLOSSY/SAFE
  constants. **In v0.1.0 the segments are pre-tagged via `RiskEvent.segment_id`
  in the synthetic events**, NOT through TF. Confirm the harness is using
  the explicit `segment_id` route, TODO for Yusuf below.
- `pass=false; evidence empty` → planner answered, but the explainer found
  no events for `hw_glossy`. Check `sqlite3 $STORE "SELECT * FROM risk_event;"`
  on the Jetson.

---

## 6. Phase 2 - cross-run memory across a restart

With phase 1 green and the bag still recording, the harness will pause
and prompt:

```
[hw] >>> RESTART the launch now (kill + relaunch with the SAME store_path)
[hw] >>> Press <Enter> here when the planner service is back up.
```

**Operator action:** in Terminal A, `Ctrl-C` the launch, wait for clean
shutdown, then re-run the EXACT SAME launch command (same params file, same
store path). Wait until the planner prints its `ready` line, then return to
terminal C and press Enter.

The harness then re-calls `/riskgraph/score_routes` WITHOUT republishing any
events. The verdict in phase 2 is:

**Pass criteria:**
- `verdict.json.phases[1].pass == true`
- `chosen_route_id == "long_safe"` again, this time from persisted history
  alone.
- Evidence event ids match phase 1 (the same `hw_slip_*`).

This is the cross-run claim. If phase 2 fails while phase 1 passed, the
likely cause is `store_path` defaulting back to `/tmp/...` on restart;
double-check the params file was sourced both times.

---

## 7. Known limitations to capture in the bag, not assert against

These are documented gaps in `docs/hardware_integration.md`. The session
should record evidence of their current state without failing the run on
them:

- **Adapter pose is `(0, 0, 0)`.** Real `/go2/safety_alert` events arriving
  through the safety_adapter will be stored with position 0, so the memory
  node's spatial join will tie-break to whichever segment is nearest to
  the origin. *This is why phase 1 uses synthetic events with explicit
  positions, to pin the join.*
- **No TF transform in adapters.** If you `ros2 topic pub` an alert with
  `frame_id=base_link`, the planner will treat it as `map`. Note in the
  bag.
- **No segment registration topic.** The integration launch does not yet
  publish a known segment list to `riskgraph_memory`. The pre-tag path
  (`RiskEvent.segment_id` set explicitly) is the only way segments get
  associated in this session.

---

## 8. Exit conditions

- **Session passes** if both phases of `tests/hw/run_scenario.sh` are green
  AND the bag is non-empty AND `colcon build` on the Jetson was clean.
  At that point: archive `tests/hw/runs/<ts>/` to the vault, append a
  "verified on hardware" line to `docs/validation.md`, and only then is it
  honest to update the project memory entry to `hw-verified`.
- **Session fails** otherwise. Do NOT update version, tag, or memory. Open
  one issue per failed assertion citing the bag path.

---

## 9. Post-session archival

```bash
cd ~/riskgraph-go2
ls tests/hw/runs/                 # find your <ts> dir
tar czf riskgraph-hw-<ts>.tar.gz tests/hw/runs/<ts>
# move tarball to vault: ~/Documents/Obsidian Vault/04 - Robotics/RiskGraph-Go2/
```

Then update `docs/validation.md`:
- Move "End-to-end behaviour with the live Go2 stack" from
  hardware-dependent to verified offline IFF both phases passed.
- Otherwise, add a "session N: failed at step X" line to the bottom of
  the hardware-dependent section.

Do NOT silently widen any other claim. Each unmoved bullet stays where it
is until separately verified.
