package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
 * Driver-controlled harness for the collect-four planner. Drive around to let the ball map fill up, plan,
 * inspect the plan in telemetry, then let Pedro drive it.
 * <ul>
 *   <li>Sticks: manual drive (when not following)</li>
 *   <li>A: plan from the current ball map (no motion)</li>
 *   <li>Y: follow the current plan</li>
 *   <li>B: cancel following</li>
 *   <li>X: clear the ball map and plan</li>
 *   <li>D-pad left / up / right: toggle yellow / red / blue as allowed targets</li>
 *   <li>Start: reset pose to {@code START_POSE} (push the robot into the start corner first)</li>
 * </ul>
 * Localization is odometry only, seeded from the corner pose; no AprilTags needed.
 */
@TeleOp(name = "Limelight: Collect Four Test", group = "Limelight")
public class LimelightCollectFourTest extends OpMode {
    /** Robot pushed into the origin corner (red side, right when standing at the red wall), facing the blue wall. */
    public static Pose START_POSE = StartPoses.cornerDegrees(StartPoses.Corner.NEAR_RIGHT, 0);

    private HiveLimelight limelight;
    private Follower follower;
    private String initError;

    private final BallMap map = new BallMap();
    private final PickupPlanner planner = new PickupPlanner();
    private final EnumSet<BallColor> allowed = EnumSet.allOf(BallColor.class);

    private PickupPlan plan;
    private boolean following;
    private int nextPickup;
    private int collected;
    private String status = "idle";

    @Override
    public void init() {
        limelight = new HiveLimelight(hardwareMap);
        limelight.start(LimelightConstants.PIPELINE_BALL_DETECTOR);
        PlannerConstants.MAX_VELOCITY_IN_S = Constants.foresightConfig.maxAchievableForwardVelocity.get();
        try {
            follower = Constants.create(hardwareMap);
            follower.setPose(START_POSE);
        } catch (Exception e) {
            initError = e.getClass().getSimpleName() + ": " + e.getMessage();
        }
        telemetry.setMsTransmissionInterval(50);
    }

    @Override
    public void loop() {
        if (follower == null) {
            limelight.update();
            telemetry.addData("Follower failed to init", initError);
            limelight.addTelemetry(telemetry);
            telemetry.update();
            return;
        }

        handleButtons();

        if (!following) {
            follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        }
        follower.update();
        limelight.update();
        map.ingest(limelight, follower.pose());

        if (following) trackPickups();

        telemetry.addData("Status", status);
        telemetry.addData("Pose", "x=%.1f y=%.1f h=%.0f", follower.pose().x(), follower.pose().y(),
                Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Allowed", allowed.toString());
        telemetry.addData("Ball map", "%s (frames %d)", map.summary(), map.framesIngested());
        for (FieldBall b : map.all()) telemetry.addData("  ball", b.toString());
        telemetry.addData("Plan", plan == null ? "-" : plan.summary());
        telemetry.addData("Collected", "%d (next %d)", collected, nextPickup);
        if (!Constants.TUNED) telemetry.addLine("Constants.TUNED is false: Pedro will not follow accurately yet");
        telemetry.update();
    }

    private void handleButtons() {
        if (gamepad1.dpadLeftWasPressed()) toggle(BallColor.YELLOW);
        if (gamepad1.dpadUpWasPressed()) toggle(BallColor.RED);
        if (gamepad1.dpadRightWasPressed()) toggle(BallColor.BLUE);
        if (gamepad1.startWasPressed()) follower.setPose(START_POSE);

        if (gamepad1.xWasPressed()) {
            map.clear();
            plan = null;
            status = "map cleared";
        }
        if (gamepad1.aWasPressed()) {
            plan = planner.plan(follower.pose(), speed(), map.plannable(allowed));
            status = plan == null ? "no plannable balls" : "planned";
        }
        if (gamepad1.yWasPressed() && plan != null && !following) {
            follower.follow(plan.path);
            following = true;
            nextPickup = 0;
            status = "following";
        }
        if (gamepad1.bWasPressed() && following) {
            follower.stop();
            following = false;
            status = "cancelled";
        }
    }

    private void toggle(BallColor c) {
        if (allowed.contains(c)) {
            if (allowed.size() > 1) allowed.remove(c);
        } else {
            allowed.add(c);
        }
    }

    private void trackPickups() {
        double sign = PlannerConstants.INTAKE_AT_REAR ? -1 : 1;
        Pose capture = follower.pose().compose(new Pose(sign * PlannerConstants.PICKUP_POINT_OFFSET_IN, 0, 0));
        while (nextPickup < plan.size()
                && plan.order.get(nextPickup).distanceTo(capture.toVector2D()) <= PlannerConstants.COLLECTED_RADIUS_IN) {
            map.remove(plan.order.get(nextPickup));
            nextPickup++;
            collected++;
        }
        if (!follower.following()) {
            following = false;
            status = "plan finished (" + nextPickup + "/" + plan.size() + " pickups reached)";
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
