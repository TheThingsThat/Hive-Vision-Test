# Testing guide: Limelight 3A + Hive-Vision + Pedro Pathing 3 + collect-four planner

Work through the phases in order. Each test adds one dependency, so a failure points at the piece that
was just added. OpModes are in the "Limelight" group on the Driver Station.

Pedro coordinates used throughout: origin is the corner on the red alliance side, on the right when you
stand at the red wall looking across the field. +x runs away from the red wall (toward blue), +y runs to
the left, heading 0 faces the blue wall, counter-clockwise is positive, units are inches. Field center
facing the blue wall is (72, 72, 0).

## Phase A. Before the robot moves

1. **Laptop.** Install Android Studio (Narwhal 3 Feature Drop or newer). Clone
   `https://github.com/TheThingsThat/Hive-Vision-Test.git`, open it, let Gradle sync, then Build > Make
   Project. It must build clean before you change anything.
2. **Model files.** Clone `https://github.com/sidhuharjas/Hive-Vision`. You need
   `yolo/weights/best_limelight3a_ssd_mobilenetv2_300x300.tflite` and `yolo/weights/labels.txt`.
3. **Limelight.** Plug the Limelight into the laptop with USB-C and open `http://limelight.local:5801`.
   Pipeline 0: type Neural Detector, upload the model and labels, confidence 0.40. Set the team number in
   Settings. Download the pipeline and keep the file. Skip what is already done.
   *Optional:* Pipeline 1, type AprilTag, full 3D on, the season's field map loaded, camera pose entered
   (LL Forward / Right / Up in meters, roll / pitch / yaw in degrees). Only needed for Tests 2 and 5; the
   collect-four OpModes start from a fixed corner pose and do not use tags.
4. **Wiring.** Limelight USB-C to the Control Hub's USB 3.0 port. Four drive motors. goBILDA Pinpoint on
   an I2C port with both odometry pods.
5. **Robot configuration** (Driver Station > Configure Robot > scan): rename the "Ethernet Device" to
   `limelight`; motors `leftFront`, `rightFront`, `leftBack`, `rightBack`; Pinpoint `pinpoint`. If you
   prefer other names, change them in `pedro/Constants.java` and `vision/LimelightConstants.java`.
   Save and activate the configuration.
6. **Geometry constants.** Measure with a tape and enter in `vision/LimelightConstants.java`:
   `CAMERA_FORWARD_OFFSET_IN`, `CAMERA_LATERAL_OFFSET_IN` (left positive), `CAMERA_HEIGHT_IN` (lens center
   to floor), `CAMERA_PITCH_DOWN_DEG`, `BALL_DIAMETER_IN`. In `planning/PlannerConstants.java`:
   `ROBOT_LENGTH_IN`, `ROBOT_WIDTH_IN`, `PICKUP_POINT_OFFSET_IN` (center to where the intake grabs),
   `INTAKE_AT_REAR`.
7. **Slow Pedro down for first runs** (optional, recommended): in `Constants.foresightConfig` add
   `c.maxPathSpeed.set(0.4);`. Remove it once everything works.
8. **Deploy.** Join the Control Hub's WiFi (or use USB), press Run in Android Studio with TeamCode
   selected. The Driver Station should now list five "Limelight:" TeleOps and "Collect Four".

## Phase B. Bench tests (nothing drives)

**Test 1: Limelight: Ball Detector Test.** Robot on the floor, no drivetrain needed.

1. Put one ball straight ahead, 24 in from the robot's tracking center (tape measure).
2. Init the OpMode. Telemetry shows each detection with confidence, tx, ty, area, dist and bearing.
3. Expect dist within 2 in of the tape and bearing near 0. If dist is off, adjust
   `CAMERA_PITCH_DOWN_DEG` first (it is the most sensitive), then `CAMERA_HEIGHT_IN`.
4. Repeat at 12, 36 and 60 in. The error should stay roughly proportional. Then move the ball 12 in to the
   left at 36 in: bearing should be about +18 deg (left is positive).
5. Watch the streak: "CONFIRMED" appears after 3 consecutive frames. A and B move the confidence threshold;
   write the value you like into `MIN_CONFIDENCE`. Y saves a snapshot you can view in the web UI.
6. Pass when all three colors are found at 36 in with confidence above 0.5 and stable range numbers.

**Test 2 (optional): Limelight: AprilTag Test.** Only if you set up pipeline 1. Needs at least one
AprilTag from the field map.

1. Place the robot at a known spot, for example field center facing the blue wall.
2. Init. Read the MT1 lines: FTC inches should be near (0, 0, 90 deg) and Pedro near (72, 72, 0 deg).
3. Move the robot 24 in toward the blue wall: Pedro x should read about 96.
4. If the Pinpoint initialized (telemetry says so) press A to seed its heading from MegaTag1. MT2 lines
   then appear and should agree with MT1 to within an inch or two.
5. Pass when both frames match the tape within 2 in and 3 deg at two positions.

**Test 3: AutoTune (Pedro).** Needs motors and Pinpoint; robot on the floor with room to drive.

1. With the Robot Controller running and the Driver Station connected, join the robot's WiFi on the laptop
   and open `http://192.168.43.1:10158`. The page is served by the Robot Controller app itself; it starts
   its own "Pedro Tuning" OpMode when you run a procedure.
2. Run "Mecanum Tuner". Paste its output over `drivetrainConfig` in `Constants.java`.
3. Run "Pinpoint Tuner" (push forward, push left, rotate 180 deg as instructed). Paste over
   `localizerConfig`.
4. Redeploy, then run "Foresight Tuner" (it drives the robot; give it 4 ft clear). Paste over
   `foresightConfig`.
5. Set `Constants.TUNED = true`. Copy `naturalForwardDeceleration` into `PlannerConstants.MAX_DECEL_IN_S2`
   and something a little lower into `MAX_ACCEL_IN_S2`; the planner copies the max velocity itself.
6. Redeploy and run "Tests" from the AutoTune page: hold, line, curve. Pass when the robot holds position
   and follows the line test without oscillating.

## Phase C. Driving tests

Clear the area, keep a hand on the Driver Station stop, and start every test at low speed.

**Test 4: Limelight: Align To Ball Test.** Motors only.

1. Ball 36 in ahead and about 12 in to one side. Press X until the target color matches.
2. Hold the right bumper. The robot should turn toward the ball, drive at it, and stop with the ball about
   12 in from the tracking center (`STANDOFF_IN`).
3. If it turns away from the ball, the drive motor directions in `Constants` are wrong: fix them there, do
   not flip the gain. If it oscillates, halve `TURN_KP`. If it stops short or long, adjust `DRIVE_KP`.

**Test 5 (optional): Limelight: Pedro Fusion Test.** Only if you set up pipeline 1. Motors, Pinpoint,
AprilTags in view.

1. Put the robot at (72, 72, 0), press Start to reset the pose if needed.
2. Drive around slowly with tags visible. "Fused - odometry" starts near zero and grows as odometry
   drifts; "accepted" counts up while tags are in view.
3. Press B to disable fusion and watch the odometry-only pose drift on its own. Press A to snap to the
   last tag estimate. Pass when the fused pose lands within 2 in of a taped position after a lap.

**Test 6: Limelight: Collect Four Test.** This is the full stack. Localization is odometry only, seeded
from the start pose, so the start position must be exact.

1. Place five or six balls in the open, at least 18 in from any wall and 12 in apart. Tape-measure each
   one's Pedro (x, y) and write them down.
2. Put the robot in the middle of the red alliance wall with its back flush against the wall, facing
   the blue wall. Center it: 72 in from either side wall to the robot's centerline. Init, then press Start
   on the gamepad to set the pose. Telemetry should read (8.5, 72, 0) for a 17 in robot. Drive straight
   forward a few inches before turning so the back corners clear the wall.
3. Drive slowly so every ball is seen for a second or more. Watch the ball map in telemetry: each ball
   needs two sightings. Positions should match your tape within 3 in. A ball listed twice means
   `MERGE_DISTANCE_IN` is too small; positions all shifted the same way means a camera constant is off.
4. Stop, press A. The plan prints: pickup order, each ball's position, approach angle, estimated time,
   planning time. Check it against your map: nearby balls, no absurd detours.
5. Press Y. The robot drives the plan without stopping between balls. Watch each pickup: the ball should
   enter centered. "Collected" counts up as the capture point passes each ball; "plan finished" appears at
   the end. Press B at any time to stop.
6. Diagnose by symptom:
   - Ball hits the bumper to one side: `CAMERA_LATERAL_OFFSET_IN` sign or value, or the camera pose in
     the Limelight (Test 1 bearing check).
   - Ball is reached but not grabbed, or grabbed too early: `PICKUP_POINT_OFFSET_IN`.
   - Robot cuts corners or slides: lower `MAX_LATERAL_ACCEL_IN_S2` and `MAX_TURN_RATE_RAD_S`, or keep
     `maxPathSpeed` low a while longer.
   - Robot brushes a wall on a wall ball: lower `MAX_WALL_INTRUSION_IN` to 0 and confirm
     `ROBOT_LENGTH_IN` / `ROBOT_WIDTH_IN` include bumpers.
   - Planning takes more than a second: set `HEADING_BINS` to 8 or `MAX_CANDIDATE_BALLS` to 8.

**Test 7: Collect Four (Autonomous).**

1. In `CollectFourAuto` set `START_WALL` (red by default) and `START_ALONG_IN` (72 = middle of the wall)
   to where the robot really starts, and `ALLOWED` to the colors this autonomous may collect. The robot
   faces straight away from the wall and does not move to look around: it plans from the balls the camera
   can see from there.
2. Place the balls inside the camera's view. Put the robot's back flush against the wall at the marked
   spot. Press Init: telemetry shows the computed start pose and the ball map filling while it waits. Wait
   until every ball you expect is listed.
3. Press Play. It watches for half a second more, plans once, drives, and stops after the last pickup with
   the count in telemetry. Leaving the wall, it holds its heading until the footprint is clear, then
   rotates toward the first ball.

## Pass criteria for "ready"

- Test 1 range within 2 in at 12 to 60 in, all three colors.
- Test 3 Tests pass and `TUNED = true`. Odometry drift matters now: after Test 3's line test, put the
  robot back against the wall at the start mark and check that the pose telemetry still reads the start
  pose within an inch.
- Test 6 collects four of four on three consecutive runs from different start spots.
