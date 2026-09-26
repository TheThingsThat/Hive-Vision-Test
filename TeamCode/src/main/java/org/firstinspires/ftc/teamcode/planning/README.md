# Collect-four planner

Given every ball the robot has seen, choose four, an order, and an approach direction for each so the
whole pickup run is as fast as possible, then hand Pedro one continuous path.

| Class | Role |
|-------|------|
| `BallMap` | Accumulates Limelight detections into field coordinates across frames. Merges repeat sightings, forgets stale balls, removes collected ones. |
| `FieldBall` | One tracked ball: color, position, sighting count, confidence. |
| `PickupPlanner` | Selection + ordering + approach-heading optimizer. Returns a `PickupPlan`. |
| `PickupPlan` | Chosen balls, pickup poses, headings, estimated time, and the Pedro `Path`. |
| `PathTimeModel` | Bezier / line sampling and the speed model used to score candidates. |
| `PlannerConstants` | Every knob. |

## Pickup geometry

A ball counts as collected when it reaches the intake at the front bumper with the robot pointed at it.
The planner enforces that by construction rather than by penalty:

- The waypoint for a ball is the ball position pulled back `PICKUP_POINT_OFFSET_IN` (default 8.5 in,
  half the 17 in robot) along the chosen approach direction. When the robot center is there, the ball
  is dead ahead at the bumper.
- The path arrives at that waypoint tangent to the approach direction, with the last
  `STRAIGHT_APPROACH_IN` inches straight, and Pedro's tangent heading interpolation keeps the robot
  pointed along the path. So the ball is on the centerline as it enters.
- Between balls the robot keeps driving; it never stops at a pickup. A short `FINAL_OVERRUN_IN` is
  added after the last ball.

Set `INTAKE_AT_REAR = true` if the intake is on the back; the robot then drives in reverse and Pedro
uses reverse-tangent heading.

## How the optimizer works

Selection and path shape are coupled: which four balls are fastest depends on how you can chain the
approaches, and the best approach direction at a ball depends on where you came from and where you go
next. The planner searches both together.

1. **Segment cost tables.** For every ordered pair of balls and every pair of approach headings from
   `HEADING_BINS` options (default 12, i.e. 30 deg), build the connecting Bezier + straight run-in and
   time it with a curvature-limited speed model (top speed, centripetal limit, turn-rate limit). The
   robot's 17 x 17 footprint, rotated by the path heading, must stay inside the field: a small intrusion
   costs time, more than `MAX_WALL_INTRUSION_IN` makes the segment infeasible. Also table the cost from
   the robot's current pose to each ball, where the initial direction of travel is free and the heading
   change from the current heading is costed as a turn.
2. **Sequence search.** Depth-first enumeration of every ordered choice of four balls. Along each branch
   a Viterbi pass keeps the best cost per approach heading at the current ball, so prefixes are shared and
   the heading assignment for a sequence is optimal for the additive model. Branches whose best possible
   cost already exceeds the worst of the top `TOP_CANDIDATES` solutions are pruned. A ball that cannot be
   reached without crossing a wall (for example one wedged in a corner) never appears in a finite-cost
   sequence; if no four-ball sequence is feasible the planner falls back to three, then two.
3. **Coupled re-scoring.** The top candidates are re-timed with acceleration and braking limits applied
   across the whole path (forward/backward speed-profile passes), including coming to rest at the end.
4. **Continuous refinement.** Each candidate's headings and initial direction are polished with a few
   rounds of coordinate descent on the coupled time. The best becomes the plan.
5. **Pedro path.** The winning geometry becomes `Paths.path(curve(...).tangent(), line(...).tangent(), ...)`.
   Segments that depart tangentially keep tangent heading; the first segment and any free departure use
   `linear` heading interpolation between the two pickup headings, so the robot rotates while moving.

**Departures.** Leaving a pickup the robot normally keeps following the tangent, which is fastest. When
that would run the footprint into a wall (a ball picked up facing a wall) or means a sharp reversal, the
planner also tries departing along the chord to the next ball, 45 deg either side of it, and straight
backwards, with the heading rotating linearly to the next approach heading. The mecanum drivetrain is
holonomic, so Pedro follows such a segment with motion and heading decoupled. Direction reversals inside a
curve (cusps) are detected and charged a full stop-and-restart.

With 12 candidate balls and 12 heading bins planning takes about 100 ms on a laptop; expect up to a second
on a Control Hub. It runs once, before the robot moves. `MAX_CANDIDATE_BALLS` caps the search if the map is
crowded (nearest balls are kept).

## Running it

- **Limelight: Collect Four Test** (TeleOp): push the robot into the origin corner and press Start to set
  the pose, drive around so the map fills, press A to plan (the plan is printed in telemetry with order,
  positions, approach angles and estimated time), Y to follow, B to cancel. D-pad toggles which colors are
  allowed.
- **Collect Four (Limelight + Pedro)** (Autonomous): the robot starts pushed into a field corner
  (`START_CORNER`, `START_HEADING_DEG`; `StartPoses` computes the exact center from the robot size), so
  odometry is seeded exactly and no AprilTags are needed. Balls visible during init are mapped, the robot
  drives `SCAN_CLEARANCE_IN` out of the corner, spins for `SCAN_SECONDS`, plans once, and drives the path.
  Set `ALLOWED` to restrict colors.

Both OpModes use `Constants.create` (odometry only). AprilTag fusion (`Constants.createWithVision`) remains
available for longer runs where drift matters; see the vision README.

## What to tune first

1. `MAX_VELOCITY_IN_S`, `MAX_ACCEL_IN_S2`, `MAX_DECEL_IN_S2` from AutoTune's numbers (the OpModes copy the
   velocity automatically).
2. `MAX_LATERAL_ACCEL_IN_S2` and `MAX_TURN_RATE_RAD_S`: lower them if Pedro cuts corners or the robot
   slides at the speeds the planner assumes.
3. `PICKUP_POINT_OFFSET_IN` and `STRAIGHT_APPROACH_IN` to match where the intake actually grabs.
4. `MERGE_DISTANCE_IN` / `MIN_OBSERVATIONS` if the map shows duplicate balls or misses real ones.

Detections are projected using the robot pose at the moment the frame is processed, not the exact capture
time, so at high speed the map will lag by the Limelight latency (tens of milliseconds). Scanning slowly
or from rest gives the cleanest map.
