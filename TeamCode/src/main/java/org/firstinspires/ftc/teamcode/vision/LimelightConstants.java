package org.firstinspires.ftc.teamcode.vision;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Every tunable number for the Limelight 3A integration lives here.
 * <p>
 * Fields are {@code public static} and deliberately not {@code final} so they can be edited from a
 * dashboard or nudged from a test OpMode without recompiling.
 */
@Configurable
public final class LimelightConstants {
    private LimelightConstants() {}

    // ------------------------------------------------------------------ hardware

    /** Name of the Limelight 3A in the Robot Controller hardware configuration. */
    public static String HARDWARE_NAME = "limelight";

    /** How often the SDK polls the Limelight for results. 100 Hz is the Limelight-recommended maximum. */
    public static int POLL_RATE_HZ = 100;

    // ------------------------------------------------------------------ pipelines
    // Pipelines are configured in the Limelight web UI (http://limelight.local:5801).

    /**
     * Neural Detector pipeline running Hive-Vision's
     * {@code best_limelight3a_ssd_mobilenetv2_300x300.tflite} with its {@code labels.txt}.
     */
    public static int PIPELINE_BALL_DETECTOR = 0;

    /** AprilTag (fiducial) pipeline with the FTC field map loaded; used for MegaTag robot localization. */
    public static int PIPELINE_APRILTAG = 1;

    // ------------------------------------------------------------------ ball detection

    /** Hive-Vision recommends 0.35 to 0.45 on the 3A. 0.25 produces visible false positives. */
    public static double MIN_CONFIDENCE = 0.40;

    /** Consecutive frames a color must be seen before it is "confirmed" (Hive-Vision guidance: 3 to 5). */
    public static int CONFIRM_FRAMES = 3;

    /** Results older than this (ms since the Control Hub received them) are treated as "no data". */
    public static long MAX_RESULT_AGE_MS = 250;

    // ------------------------------------------------------------------ camera mount geometry
    // Robot frame: +x forward, +y left, +z up. Inches and degrees. Measure these on the real robot.

    /** Lens position ahead of the robot's tracking center. */
    public static double CAMERA_FORWARD_OFFSET_IN = 2.5;

    /** Lens position to the LEFT of the robot's tracking center (negative = right). */
    public static double CAMERA_LATERAL_OFFSET_IN = -1.0;

    /** Lens height above the floor. */
    public static double CAMERA_HEIGHT_IN = 9.7;

    /** Positive = lens tilted DOWN toward the floor. 0 = level with the floor. */
    public static double CAMERA_PITCH_DOWN_DEG = 20.0;

    /** Diameter of a game ball. The camera ray is intersected with the plane through the ball's center. */
    public static double BALL_DIAMETER_IN = 3.0;

    // ------------------------------------------------------------------ AprilTag localization

    /** Use MegaTag2 (IMU-fused) botpose when a fresh robot heading has been pushed; otherwise MegaTag1. */
    public static boolean USE_MEGATAG2 = true;

    /** Minimum number of tags in a botpose solution before it is used. */
    public static int MIN_TAG_COUNT = 1;

    /** Reject botpose solutions whose average tag distance exceeds this (meters, as reported by the Limelight). */
    public static double MAX_TAG_AVG_DISTANCE_M = 3.0;

    /** Reject botpose solutions that land outside the field by more than this margin (inches). */
    public static double FIELD_MARGIN_IN = 6.0;

    /** Reject a single-tag measurement that disagrees with the current estimate by more than this (inches). 0 disables. */
    public static double MAX_SINGLE_TAG_JUMP_IN = 36.0;

    /** MegaTag2 needs a fresh heading. If none was pushed within this window, fall back to MegaTag1. */
    public static long MAX_HEADING_AGE_MS = 250;

    // ------------------------------------------------------------------ Kalman fusion (Pedro FusionLocalizer)
    // Variances are standard deviation squared. x/y in inches squared, heading in radians squared.

    public static double INITIAL_VARIANCE_XY = 1.0;
    public static double INITIAL_VARIANCE_HEADING = 0.01;

    /** Odometry drift accumulated per inch travelled / radian turned, in the body frame. */
    public static double PROCESS_VARIANCE_XY = 0.02;
    public static double PROCESS_VARIANCE_HEADING = 0.001;

    /** Vision measurement variance used when the Limelight does not report standard deviations. */
    public static double VISION_VARIANCE_XY = 4.0;       // 2 in standard deviation
    public static double VISION_VARIANCE_HEADING = 0.03; // about 10 deg standard deviation (MegaTag1 only)

    /** Number of localizer updates kept for latency compensation. At 50 Hz, 500 is about 10 s. */
    public static int FUSION_HISTORY_SIZE = 500;
}
