package org.firstinspires.ftc.teamcode.opmodes;

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
 * Autonomous: start with the robot's back flush against the middle of a wall, plan from whatever balls the
 * camera can see from there, pick the best four, drive over them.
 * <p>
 * Localization is odometry only, seeded from the exact start pose; no AprilTags are needed. Balls seen
 * while waiting for start are mapped as they appear. After start the robot looks for a short moment more
 * (in case init was very brief), plans once and follows the plan. It does not move to look around.
 */
@Autonomous(name = "Collect Four (Limelight + Pedro)", group = "Limelight")
public class CollectFourAuto extends OpMode {
    /** Which wall the robot's back is against. It faces straight away from that wall. */
    public static StartPoses.Wall START_WALL = StartPoses.Wall.RED;

    /** Position along that wall in inches (measured in the +x or +y direction). 72 is the middle. */
    public static double START_ALONG_IN = PlannerConstants.FIELD_SIZE_IN / 2.0;

    /** Seconds to keep watching after start before planning, so the map has a few frames even after a quick init. */
    public static double LOOK_SECONDS = 0.5;

    /** Which colors this autonomous is allowed to collect. */
    public static EnumSet<BallColor> ALLOWED = EnumSet.allOf(BallColor.class);

    private enum State { LOOK, PLAN, COLLECT, DONE }

    private HiveLimelight limelight;
    private Follower follower;
    private Pose startPose;
    private final BallMap map = new BallMap();
    private final PickupPlanner planner = new PickupPlanner();
    private final ElapsedTime timer = new ElapsedTime();

    private State state = State.LOOK;
    private PickupPlan plan;
    private int nextPickup;
    private String status = "";

    @Override
    public void init() {
        startPose = StartPoses.againstWall(START_WALL, START_ALONG_IN);

        limelight = new HiveLimelight(hardwareMap);
        limelight.start(LimelightConstants.PIPELINE_BALL_DETECTOR);
        PlannerConstants.MAX_VELOCITY_IN_S = Constants.foresightConfig.maxAchievableForwardVelocity.get();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        telemetry.setMsTransmissionInterval(50);
    }

    @Override
    public void init_loop() {
        // Robot is stationary at the start pose, so anything seen now can be mapped already.
        limelight.update();
        map.ingest(limelight, startPose);
        telemetry.addData("Start", "back on %s wall at %.0f in -> (%.1f, %.1f, %.0f deg)", START_WALL, START_ALONG_IN,
                startPose.x(), startPose.y(), Math.toDegrees(startPose.heading()));
        telemetry.addData("Ball map", map.summary());
        for (FieldBall b : map.all()) telemetry.addData("  ball", b.toString());
        telemetry.update();
    }

    @Override
    public void start() {
        timer.reset();
        state = LOOK_SECONDS > 0 ? State.LOOK : State.PLAN;
    }

    @Override
    public void loop() {
        switch (state) {
            case LOOK:
                if (timer.seconds() >= LOOK_SECONDS) state = State.PLAN;
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
