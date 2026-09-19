# Limelight 3A + Hive-Vision + Pedro Pathing 3

This package integrates a Limelight 3A into the Pedro Pathing 3 starter so it can be tested on the robot.

| Piece | What it does |
|-------|--------------|
| `HiveLimelight` | Wraps `Limelight3A`. Reads Hive-Vision ball detections (`bestOf(BallColor)`, multi-frame confirmation, floor-position estimate) and AprilTag botpose (MegaTag1/2) converted to Pedro coordinates. |
| `LimelightConstants` | Every tunable: hardware name, pipeline indexes, confidence threshold, camera mount geometry, tag gating, Kalman variances. |
| `FieldCoordinates` | FTC field frame (Limelight botpose, meters, center origin) to Pedro frame (inches, bottom-left origin) and back. |
| `CameraGeometry` | tx/ty angles to a floor point in the robot frame using the mount height and pitch. |
| `pedro/VisionFusedLocalizer` | A Pedro `Localizer` that wraps Pinpoint odometry in Pedro's `FusionLocalizer` Kalman filter and feeds it Limelight poses every loop. |
| `pedro/Constants.createWithVision(...)` | Builds a `Follower` on the fused localizer. |
| `opmodes/Limelight*Test` | Four test OpModes, ordered from "no drivetrain needed" to "full stack". |

## 1. Limelight setup (web UI at `http://limelight.local:5801`)

1. Update the Limelight to the latest firmware.
2. **Pipeline 0, Neural Detector.** Upload `yolo/weights/best_limelight3a_ssd_mobilenetv2_300x300.tflite`
   and `yolo/weights/labels.txt` from the Hive-Vision repo. Class order must stay
   `yellow_pollen, red_nectar, blue_nectar`. Set the confidence threshold in the 0.35 to 0.45 range
   (Hive-Vision's guidance; 0.25 shows false positives). Do not upload the ONNX or float32/int8 YOLO files;
   the 3A only loads this full-INT8 SSD model.
3. **Pipeline 1, AprilTag.** Enable 3D / full 3D, load the current season's FTC field map, and enter the
   camera pose in robot space (forward / right / up offsets and pitch). Enable MegaTag2 if the firmware offers
   the option. The botpose is only as good as this camera pose.
4. Give the Limelight a fixed IP or keep the default; the Control Hub finds it over USB either way.

## 2. Robot Controller setup

1. Plug the Limelight into a USB port on the Control Hub. It shows up as an Ethernet device.
2. In the robot configuration, add it and name it `limelight` (or change `LimelightConstants.HARDWARE_NAME`).
3. Measure and enter the camera mount in `LimelightConstants`: forward and lateral offset from the robot's
   tracking center, lens height, downward pitch, and the ball diameter. These drive the distance estimate.
4. Fill in `pedro/Constants.java` with the real motor and Pinpoint names, then run the **Tuning** OpMode and
   paste AutoTune's output over the three config blocks. Set `Constants.TUNED = true` when done.

## 3. Test plan

Run the OpModes in this order. Each one adds a dependency.

1. **Limelight: Ball Detector Test** (no drivetrain, no odometry). Confirms the USB link, the neural pipeline,
   and the confidence threshold. Put a ball at a tape-measured distance straight ahead and check the
   `dist` / `bearing` telemetry; adjust the camera constants until they agree. Y saves a snapshot you can
   inspect in the web UI.
2. **Limelight: AprilTag Test** (no drivetrain; odometry optional). Place the robot at a known field
   position and compare the MT1 / MT2 numbers in both frames. If the Pinpoint initializes, press A to seed
   its heading from MegaTag1 so MT2 becomes available.
3. **Limelight: Align To Ball Test** (drive motors only). Hold the right bumper to turn toward and approach
   the confirmed ball of the selected color. Start with `TURN_KP`/`DRIVE_KP` low and raise them.
4. **Limelight: Pedro Fusion Test** (drive motors + Pinpoint). Drive around; the fused pose should follow
   odometry between tags and snap back onto the tags when they are visible. B toggles fusion so you can see
   the odometry drift; A hard-snaps to the last tag estimate.

## 4. Using it in your own OpModes

```java
HiveLimelight limelight = new HiveLimelight(hardwareMap);
limelight.start(LimelightConstants.PIPELINE_APRILTAG);          // or PIPELINE_BALL_DETECTOR
Follower follower = Constants.createWithVision(hardwareMap, limelight);
follower.setPose(startPose);

// every loop
follower.update();                                             // odometry + vision fusion
BallDetection ball = limelight.confirmedBestOf(BallColor.RED); // null until seen CONFIRM_FRAMES in a row
if (ball != null && ball.hasRangeEstimate()) { /* ball.forwardIn, ball.lateralIn, ball.bearingRad() */ }

// on stop
limelight.stop();
```

Only one Limelight pipeline runs at a time, so an autonomous typically starts on the AprilTag pipeline to
localize, switches to the ball pipeline with `limelight.setPipeline(...)` while hunting balls, and switches
back when it needs a relocalization.

## 5. Conventions worth knowing

- Pedro `DrivePowers` and poses are (forward, **left**, counter-clockwise). A ball at positive `tx`
  (to the right) needs a negative turn power.
- Limelight botpose is the FTC frame in meters; `FieldCoordinates` applies Pedro's published conversion
  `pedro = (y + 72, 72 - x, heading - 90 deg)`.
- The SDK's `LLResult` timestamp is wall-clock based, while Pedro's fusion history uses `System.nanoTime()`.
  `HiveLimelight` therefore builds the capture time from staleness plus capture/targeting latency.
- MegaTag2 heading is just the heading we push in, so it is sent to the filter as "not measured" (NaN).
