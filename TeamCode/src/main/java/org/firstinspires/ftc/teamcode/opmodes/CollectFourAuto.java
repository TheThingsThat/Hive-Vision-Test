package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
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
 * Autonomous: look for balls, pick the best four, drive over them.
 * <p>
 * Balls seen while waiting for start are already in the map. After start the robot optionally rotates in
 * place for {@code SCAN_SECONDS} to see more of the field, then plans once and follows the plan.
 */
@Autonomous(name = "Collect Four (Limelight + Pedro)", group = "Limelight")
public class CollectFourAuto extends OpMode {
    /** Where the robot starts, Pedro coordinates. */
    public static Pose START_POSE = new Pose(72, 72, 0);
    public static double SCAN_SECONDS = 2.0;
    public static double SCAN_TURN_POWER = 0.25;
    /** Which colors this autonomous is allowed to collect. */
    public static EnumSet<BallColor> ALLOWED = EnumSet.allOf(BallColor.class);

    private enum State { SCAN, PLAN, COLLECT, DONE }

    private HiveLimelight limelight;
    private Follower follower;
    private final BallMap map = new BallMap();
    private final PickupPlanner planner = new PickupPlanner();
    private final ElapsedTime timer = new ElapsedTime();

    private State state = State.SCAN;
    private PickupPlan plan;
    private int nextPickup;
    private String status = "";

    @Override
    public void init() {
        limelight = new HiveLimelight(hardwareMap);
        limelight.start(LimelightConstants.PIPELINE_BALL_DETECTOR);
        PlannerConstants.MAX_VELOCITY_IN_S = Constants.foresightConfig.maxAchievableForwardVelocity.get();
        follower = Constants.createWithVision(hardwareMap, limelight);
        follower.setPose(START_POSE);
        telemetry.setMsTransmissionInterval(50);
    }

    @Override
    public void init_loop() {
        // Robot is stationary at START_POSE, so anything seen now can be mapped already.
        limelight.update();
        map.ingest(limelight, START_POSE);
        telemetry.addData("Ball map", map.summary());
        for (FieldBall b : map.all()) telemetry.addData("  ball", b.toString());
        telemetry.update();
    }

    @Override
    public void start() {
        timer.reset();
        state = SCAN_SECONDS > 0 ? State.SCAN : State.PLAN;
    }

    @Override
    public void loop() {
        switch (state) {
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
