package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.VisionFusedLocalizer;
import org.firstinspires.ftc.teamcode.vision.HiveLimelight;
import org.firstinspires.ftc.teamcode.vision.LimelightConstants;
import org.firstinspires.ftc.teamcode.vision.VisionPoseEstimate;

/**
 * Full-stack test: Pedro follower whose localizer fuses Pinpoint odometry with Limelight AprilTag poses.
 * Drive around with the sticks and watch the fused pose track the odometry pose between tags, then get
 * pulled onto the tags when they come into view.
 * <ul>
 *   <li>A: snap the pose to the latest AprilTag estimate</li>
 *   <li>B: toggle vision fusion on/off (odometry keeps running)</li>
 *   <li>X: reset the pose to {@code START_POSE}</li>
 *   <li>D-pad up / down: AprilTag pipeline / ball pipeline (fusion only works on the AprilTag pipeline)</li>
 * </ul>
 * Requires the Pinpoint and drive motors from {@link Constants}.
 */
@TeleOp(name = "Limelight: Pedro Fusion Test", group = "Limelight")
public class LimelightPedroFusionTest extends OpMode {
    /** Pedro coordinates: field center, facing +x. */
    public static Pose START_POSE = new Pose(72, 72, 0);

    private HiveLimelight limelight;
    private Follower follower;
    private VisionFusedLocalizer localizer;
    private String initError;

    @Override
    public void init() {
        limelight = new HiveLimelight(hardwareMap);
        limelight.start(LimelightConstants.PIPELINE_APRILTAG);
        try {
            follower = Constants.createWithVision(hardwareMap, limelight);
            localizer = (VisionFusedLocalizer) follower.localizer;
            follower.setPose(START_POSE);
        } catch (Exception e) {
            initError = e.getClass().getSimpleName() + ": " + e.getMessage();
        }
        telemetry.setMsTransmissionInterval(50);
    }

    @Override
    public void loop() {
        if (follower == null) {
            telemetry.addData("Follower failed to init", initError);
            telemetry.addLine("Check Constants motor / pinpoint names.");
            limelight.update();
            limelight.addTelemetry(telemetry);
            telemetry.update();
            return;
        }

        if (gamepad1.aWasPressed()) localizer.snapToVision();
        if (gamepad1.bWasPressed()) localizer.setVisionEnabled(!localizer.visionEnabled());
        if (gamepad1.xWasPressed()) follower.setPose(START_POSE);
        if (gamepad1.dpadUpWasPressed()) limelight.setPipeline(LimelightConstants.PIPELINE_APRILTAG);
        if (gamepad1.dpadDownWasPressed()) limelight.setPipeline(LimelightConstants.PIPELINE_BALL_DETECTOR);

        // Pedro DrivePowers are (forward, left, counter-clockwise); negate the x axes so stick-right means right.
        follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        follower.update(); // also updates the Limelight and applies any new vision measurement

        Pose fused = follower.pose();
        Pose odo = localizer.odometryPose();
        VisionPoseEstimate last = localizer.lastEstimate();

        if (!Constants.TUNED) telemetry.addLine("Constants.TUNED is false: drivetrain / Pinpoint values are placeholders");
        telemetry.addData("Fused pose", "x=%.1f y=%.1f h=%.1f", fused.x(), fused.y(), Math.toDegrees(fused.heading()));
        telemetry.addData("Odometry pose", "x=%.1f y=%.1f h=%.1f", odo.x(), odo.y(), Math.toDegrees(odo.heading()));
        telemetry.addData("Fused - odometry", "dx=%.1f dy=%.1f", fused.x() - odo.x(), fused.y() - odo.y());
        telemetry.addData("Vision", "%s accepted=%d rejected=%d", localizer.visionEnabled() ? "ON" : "OFF",
                localizer.accepted(), localizer.rejected());
        telemetry.addData("Last vision", last == null ? "-" : last.toString());
        limelight.addTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) follower.drivetrain.stop();
        limelight.stop();
    }
}
