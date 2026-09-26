package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.StartPoses;
import org.firstinspires.ftc.teamcode.planning.BallMap;
import org.firstinspires.ftc.teamcode.planning.FieldBall;
import org.firstinspires.ftc.teamcode.planning.PickupPlan;
import org.firstinspires.ftc.teamcode.planning.PickupPlanner;
import org.firstinspires.ftc.teamcode.planning.PlannerConstants;
import org.firstinspires.ftc.teamcode.vision.BallColor;
import org.firstinspires.ftc.teamcode.vision.HiveLimelight;
import org.firstinspires.ftc.teamcode.vision.LimelightConstants;

import java.util.EnumSet;

/**
 * Autonomous: start pressed into a known corner, look for balls, pick the best four, drive over them.
 * <p>
 * Localization is odometry only, seeded from the exact corner pose; no AprilTags are needed. Balls seen
 * while waiting for start are already in the map. After start the robot drives diagonally out of the
 * corner (a rotating square would hit both walls), spins for {@code SCAN_SECONDS} to see more of the
 * field, plans once and follows the plan.
 */
@Autonomous(name = "Collect Four (Limelight + Pedro)", group = "Limelight")
public class CollectFourAuto extends OpMode {
    /** Which corner the robot is pushed into, and which way it faces (0 = toward the blue wall). */
    public static StartPoses.Corner START_CORNER = StartPoses.Corner.NEAR_RIGHT;
    public static double START_HEADING_DEG = 0;

    /** How far to move diagonally toward the field center before spinning. 0 skips the move and the spin. */
    public static double SCAN_CLEARANCE_IN = 14.0;
    public static double SCAN_SECONDS = 2.5;
    public static double SCAN_TURN_POWER = 0.25;

    /** Which colors this autonomous is allowed to collect. */
    public static EnumSet<BallColor> ALLOWED = EnumSet.allOf(BallColor.class);

    private enum State { LEAVE_CORNER, SCAN, PLAN, COLLECT, DONE }

    private HiveLimelight limelight;
    private Follower follower;
    private Pose startPose;
    private Pose scanPose;
    private final BallMap map = new BallMap();
    private final PickupPlanner planner = new PickupPlanner();
    private final ElapsedTime timer = new ElapsedTime();

    private State state = State.LEAVE_CORNER;
    private PickupPlan plan;
    private int nextPickup;
    private String status = "";

    @Override
    public void init() {
        startPose = StartPoses.cornerDegrees(START_CORNER, START_HEADING_DEG);
        scanPose = StartPoses.towardCenter(startPose, SCAN_CLEARANCE_IN);

        limelight = new HiveLimelight(hardwareMap);
        limelight.start(LimelightConstants.PIPELINE_BALL_DETECTOR);
        PlannerConstants.MAX_VELOCITY_IN_S = Constants.foresightConfig.maxAchievableForwardVelocity.get();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        telemetry.setMsTransmissionInterval(50);
    }

    @Override
    public void init_loop() {
        // Robot is stationary at the corner, so anything seen now can be mapped already.
        limelight.update();
        map.ingest(limelight, startPose);
        telemetry.addData("Start", "%s facing %.0f deg -> (%.1f, %.1f)", START_CORNER, START_HEADING_DEG,
                startPose.x(), startPose.y());
        telemetry.addData("Ball map", map.summary());
        for (FieldBall b : map.all()) telemetry.addData("  ball", b.toString());
        telemetry.update();
    }

    @Override
    public void start() {
        timer.reset();
        if (SCAN_CLEARANCE_IN > 0) {
            follower.follow(Paths.line(startPose, scanPose).constant(startPose.heading()));
            state = State.LEAVE_CORNER;
        } else {
            state = State.PLAN;
        }
    }

    @Override
    public void loop() {
        switch (state) {
            case LEAVE_CORNER:
                if (!follower.following()) {
                    timer.reset();
                    state = SCAN_SECONDS > 0 ? State.SCAN : State.PLAN;
                }
                break;

            case SCAN:
                follower.manual(0, 0, SCAN_TURN_POWER);
                if (timer.seconds() >= SCAN_SECONDS) {
                    follower.stop();
                    state = State.PLAN;
                }
                break;

            case PLAN:
                plan = planner.plan(follower.pose(), speed(), map.plannable(ALLOWED));
                if (plan == null) {
                    status = "no balls to collect";
                    state = State.DONE;
                } else {
                    follower.follow(plan.path);
                    nextPickup = 0;
                    status = "collecting";
                    state = State.COLLECT;
                }
                break;

            case COLLECT:
                trackPickups();
                if (!follower.following()) {
                    status = "finished, " + nextPickup + "/" + plan.size() + " pickups reached";
                    follower.stop();
                    state = State.DONE;
                }
                break;

            case DONE:
                break;
        }

        follower.update();
        limelight.update();
        map.ingest(limelight, follower.pose());

        telemetry.addData("State", "%s %s", state, status);
        telemetry.addData("Pose", "x=%.1f y=%.1f h=%.0f", follower.pose().x(), follower.pose().y(),
                Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Ball map", map.summary());
        telemetry.addData("Plan", plan == null ? "-" : plan.summary());
        telemetry.update();
    }

    private void trackPickups() {
        double sign = PlannerConstants.INTAKE_AT_REAR ? -1 : 1;
        Pose capture = follower.pose().compose(new Pose(sign * PlannerConstants.PICKUP_POINT_OFFSET_IN, 0, 0));
        while (nextPickup < plan.size()
                && plan.order.get(nextPickup).distanceTo(capture.toVector2D()) <= PlannerConstants.COLLECTED_RADIUS_IN) {
            map.remove(plan.order.get(nextPickup));
            nextPickup++;
        }
    }

    private double speed() {
        Velocity v = follower.velocity();
        return Math.hypot(v.vx, v.vy);
    }

    @Override
    public void stop() {
        follower.drivetrain.stop();
        limelight.stop();
    }
}
