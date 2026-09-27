package org.firstinspires.ftc.teamcode.planning;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Tunables for ball selection and pickup-path planning. Public static (not final) so they can be
 * edited from a dashboard or test OpMode.
 */
@Configurable
public final class PlannerConstants {
    private PlannerConstants() {}

    // ------------------------------------------------------------------ robot & intake geometry

    /** Robot footprint, inches. Used for wall clearance. */
    public static double ROBOT_LENGTH_IN = 17.5;
    public static double ROBOT_WIDTH_IN = 17.5;

    /**
     * Distance from the robot's tracking center to the point where a ball is captured, measured along the
     * direction of travel. Half the robot length puts that point exactly at the front bumper.
     */
    public static double PICKUP_POINT_OFFSET_IN = ROBOT_LENGTH_IN / 2.0;

    /** True if the intake is on the back of the robot (robot drives intake-first, i.e. in reverse). */
    public static boolean INTAKE_AT_REAR = false;

    /** Straight run-in before each pickup so the ball is centered when it reaches the intake, inches. */
    public static double STRAIGHT_APPROACH_IN = 6.0;

    /** Extra straight travel after the last pickup so the final ball is fully ingested, inches. */
    public static double FINAL_OVERRUN_IN = 4.0;

    // ------------------------------------------------------------------ selection

    /** How many balls a plan collects (fewer if fewer are known). */
    public static int BALLS_TO_COLLECT = 4;

    /** Only the nearest this-many known balls are considered, to bound planning time. */
    public static int MAX_CANDIDATE_BALLS = 12;

    // ------------------------------------------------------------------ time model (inches, seconds)

    /** Robot top speed. Copy from AutoTune's maxAchievableForwardVelocity. */
    public static double MAX_VELOCITY_IN_S = 50.0;
    public static double MAX_ACCEL_IN_S2 = 60.0;
    public static double MAX_DECEL_IN_S2 = 80.0;

    /** Centripetal limit: v^2 * curvature <= this. Lower it if the robot slides in corners. */
    public static double MAX_LATERAL_ACCEL_IN_S2 = 40.0;

    /** Max heading rate while following tangentially: v * curvature <= this (rad/s). */
    public static double MAX_TURN_RATE_RAD_S = 3.5;

    /** Turn-in-place rate used to cost the initial heading change (rad/s). */
    public static double TURN_IN_PLACE_RATE_RAD_S = 3.0;

    // ------------------------------------------------------------------ path shape

    /** Bezier handle length as a fraction of the segment chord. */
    public static double HANDLE_FRACTION = 0.4;
    public static double MIN_HANDLE_IN = 6.0;

    /** Samples per curved segment for the time model. */
    public static int SAMPLES_PER_SEGMENT = 16;

    // ------------------------------------------------------------------ search

    /** Discrete approach headings per ball in the dynamic program (12 = 30 deg). */
    public static int HEADING_BINS = 12;

    /** How many discrete solutions are re-scored with the coupled speed profile and refined. */
    public static int TOP_CANDIDATES = 12;

    /** Coordinate-descent rounds for continuous heading refinement. */
    public static int REFINE_ROUNDS = 3;

    // ------------------------------------------------------------------ field

    public static double FIELD_SIZE_IN = 144.0;

    /** Seconds of penalty per inch the robot footprint would poke past a wall (soft, below the hard limit). */
    public static double OUT_OF_FIELD_PENALTY_S_PER_IN = 5.0;

    /**
     * A path whose heading-rotated footprint would cross a wall by more than this is infeasible and never
     * chosen. Balls that cannot be reached without doing so are skipped, inches.
     */
    public static double MAX_WALL_INTRUSION_IN = 0.5;

    // ------------------------------------------------------------------ ball map

    /** Detections closer than this to a known ball of the same color are merged into it, inches. */
    public static double MERGE_DISTANCE_IN = 5.0;

    /** Sightings needed before a ball is eligible for planning. */
    public static int MIN_OBSERVATIONS = 2;

    /** Ignore detections estimated beyond this range (angle errors blow up with distance), inches. */
    public static double MAX_DETECTION_RANGE_IN = 96.0;

    /** Forget balls not seen for this long, seconds. Balls can be out of view while still on the field, so keep it generous. */
    public static double BALL_MAX_AGE_S = 30.0;

    /** A ball is considered collected when the robot's capture point passes within this distance of it, inches. */
    public static double COLLECTED_RADIUS_IN = 4.0;
}
