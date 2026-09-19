package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.vision.FieldCoordinates;
import org.firstinspires.ftc.teamcode.vision.HiveLimelight;
import org.firstinspires.ftc.teamcode.vision.LimelightConstants;
import org.firstinspires.ftc.teamcode.vision.VisionPoseEstimate;

import java.util.Arrays;
import java.util.List;

/**
 * Bench test for AprilTag localization. Shows the raw MegaTag1 / MegaTag2 botpose in both the FTC frame
 * (inches) and Pedro coordinates, plus the visible tag ids and the reported standard deviations.
 * <p>
 * If the odometry localizer in {@link Constants} initializes, its heading is pushed to the Limelight so
 * MegaTag2 works; otherwise only MegaTag1 is shown. Press A to seed the odometry heading from MegaTag1.
 * Place the robot at a known field spot and compare the numbers.
 */
@TeleOp(name = "Limelight: AprilTag Test", group = "Limelight")
public class LimelightAprilTagTest extends OpMode {
    private HiveLimelight limelight;
    private Localizer odometry;
    private String odometryError;

    @Override
    public void init() {
        limelight = new HiveLimelight(hardwareMap);
        limelight.start(LimelightConstants.PIPELINE_APRILTAG);
        try {
            odometry = Constants.createOdometryLocalizer(hardwareMap);
        } catch (Exception e) {
            odometry = null;
            odometryError = e.getClass().getSimpleName() + ": " + e.getMessage();
        }
        telemetry.setMsTransmissionInterval(50);
    }

    @Override
    public void loop() {
        if (odometry != null) {
            odometry.update();
            limelight.updateRobotHeading(odometry.pose().heading());
        }
        limelight.update();

        LLResult r = limelight.latestResult();
        if (odometry != null && gamepad1.aWasPressed() && r != null) {
            Pose mt1 = FieldCoordinates.pose3DToPedro(r.getBotpose());
            if (mt1 != null && r.getBotposeTagCount() > 0) odometry.setPose(mt1);
        }

        limelight.addTelemetry(telemetry);
        telemetry.addData("Odometry", odometry == null ? "unavailable (" + odometryError + ") -> MegaTag1 only"
                : String.format("heading=%.1f deg (A = seed from MT1)", Math.toDegrees(odometry.pose().heading())));

        if (r != null) {
            telemetry.addData("Tags", "count=%d avgDist=%.2fm span=%.2f", r.getBotposeTagCount(), r.getBotposeAvgDist(), r.getBotposeSpan());
            List<LLResultTypes.FiducialResult> fids = r.getFiducialResults();
            if (fids != null) {
                StringBuilder ids = new StringBuilder();
                for (LLResultTypes.FiducialResult f : fids) ids.append(f.getFiducialId()).append(' ');
                telemetry.addData("Tag ids", ids.length() == 0 ? "-" : ids.toString());
            }
            describe("MT1", r.getBotpose(), r.getStddevMt1());
            describe("MT2", r.getBotpose_MT2(), r.getStddevMt2());
        }

        VisionPoseEstimate est = limelight.poseEstimate();
        telemetry.addData("Accepted estimate", est == null ? "-" : est.toString());
        telemetry.addData("Rejected", limelight.botposeRejections());
        telemetry.update();
    }

    private void describe(String label, Pose3D p, double[] stddev) {
        Pose ftc = FieldCoordinates.pose3DToFtcInches(p);
        Pose pedro = FieldCoordinates.pose3DToPedro(p);
        if (ftc == null || pedro == null) {
            telemetry.addData(label, "-");
            return;
        }
        telemetry.addData(label + " FTC in", "x=%.1f y=%.1f yaw=%.1f", ftc.x(), ftc.y(), Math.toDegrees(ftc.heading()));
        telemetry.addData(label + " Pedro", "x=%.1f y=%.1f h=%.1f", pedro.x(), pedro.y(), Math.toDegrees(pedro.heading()));
        telemetry.addData(label + " stddev", stddev == null ? "-" : Arrays.toString(stddev));
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
