# antoniq_navigation

Nav2 configuration package for the Antoniq robot.

## Tuning notes: reducing unnecessary "flip" turns on `FollowPath` / `FollowBackPath`

**Symptom:** the robot would occasionally get stuck when only partially
overlapping the local costmap's inflated region, and while turning near the
edge of the costmap it would touch the inflation and abruptly whip around to
face the opposite direction ("flip").

Both `FollowPath` and `FollowBackPath` use `nav2_mppi_controller::MPPIController`,
which samples a batch of candidate trajectories every cycle and scores them with
the configured critics. The flipping behavior is a classic symptom of MPPI
oscillating between two similarly-scored candidate turns (one left, one right)
once the trajectory brushes the inflated costmap — small perturbations tip the
decision from one cycle to the next, so the chosen heading swings hard between
cycles instead of committing smoothly to one turn.

The following parameters were adjusted for both `FollowPath` and
`FollowBackPath` in [`config/nav2_params.yaml`](config/nav2_params.yaml):

| Parameter | Before | After | Why |
|---|---|---|---|
| `wz_std` | `0.4` | `0.3` | This is the standard deviation used when sampling candidate angular velocities. A high value makes MPPI sample a wide spread of turn directions/magnitudes each cycle, so once one side of the footprint brushes cost, the next-best sample can be a hard turn the *other* way. Narrowing the sampling distribution keeps consecutive cycles' chosen turns closer together, reducing sudden reversals. |
| `prune_distance` | `1.0` | `1.7` | This is how far ahead along the global path the controller keeps for local planning. A short distance means the controller only "sees" a turn very late and has to react abruptly. Restoring the longer lookahead (the value was already noted as an alternative in a comment) gives it more path horizon to commit to a turn smoothly instead of last-second correcting. |
| `CostCritic.cost_weight` | `8.0` | `6.0` | Controls how strongly proximity to inflated/occupied cost penalizes a trajectory. At `8.0`, a trajectory that only grazes the inflation with a small part of the footprint was penalized almost as heavily as one running through the middle of an obstacle, causing the controller to reject otherwise-fine forward trajectories (the "stuck" symptom) and instead jump to a very different (flipped) heading. Lowering the weight makes partial/brief inflation contact less catastrophic to a trajectory's score. |
| `CostCritic.critical_cost` | `300.0` | `350.0` | This is the cost threshold above which a trajectory is treated as critically bad. Raising it slightly gives a bit more headroom before a partially-overlapping trajectory is treated as equivalent to a real collision. |
| `TwirlingCritic` | disabled (commented out) | enabled, `twirling_cost_power: 1`, `twirling_cost_weight: 8.0` | This critic directly penalizes unnecessary rotation in a candidate trajectory. It was previously commented out. Enabling it discourages the controller from choosing a trajectory that spins/whips the robot around when a smoother, less rotational option is available — this is the most direct counter to the "flip" behavior. |

### Suggested follow-up (not applied)

- `robot_radius` (`local_costmap`/`global_costmap`, currently `0.2`) drives the
  circular collision footprint used by `CostCritic` (`consider_footprint: false`
  means the real footprint polygon isn't used, per the comment already in the
  file). If the robot's true footprint is smaller than a 0.2 m radius circle,
  this circle is inflating how much of the costmap counts as "inside the
  robot," contributing to the partial-overlap "stuck" symptom. This wasn't
  changed here because it requires confirming the robot's actual physical
  dimensions first — shrinking it without knowing the true footprint risks
  under-estimating the collision envelope.
- `wz_max` (`1.82` rad/s) could be lowered further (e.g. ~1.2-1.5) if flips are
  still too violent after the above changes — it caps how fast a turn can
  physically be, regardless of which one MPPI selects.

### Re-tuning

If flipping still occurs, tighten `wz_std` further and/or raise
`twirling_cost_weight`. If the robot instead becomes too hesitant/sluggish
around obstacles (won't commit to any turn), ease `CostCritic.cost_weight`
back up or `critical_cost` back down in small steps and re-test.
