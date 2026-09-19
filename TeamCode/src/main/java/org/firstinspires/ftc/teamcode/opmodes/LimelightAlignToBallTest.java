package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.vision.BallColor;
import org.firstinspires.ftc.teamcode.vision.BallDetection;
import org.firstinspires.ftc.teamcode.vision.HiveLimelight;
import org.firstinspires.ftc.teamcode.vision.LimelightConstants;

/**
 * First closed-loop test of vision driving the drivetrain. Needs the mecanum motors from
 * {@link Constants#drivetrainConfig} but no odometry.
 * <p>
 * Drive manually with the sticks. Hold the right bumper to turn toward, and approach, the confirmed
 * best ball of the selected color, stopping {@code STANDOFF_IN} away. X cycles the target color.
 * <p>
 * Pedro's {@code DrivePowers} convention is (forward, left, counter-clockwise), so a ball to the right
 * (positive tx) needs a negative turn power.
 */
@TeleOp(name = "Limelight: Align To Ball Test", group = "Limelight")
public class LimelightAlignToBallTest extends OpMode {
    public static double TURN_KP = 0.02;       // power per degree of tx
    public static double DRIVE_KP = 0.03;      // power per inch of range error
    public static double STANDOFF_IN = 12.0;   // stop this far from the ball center
    public static double MAX_TURN = 0.5;
    public static double MAX_DRIVE = 0.5;
    public static double TX_DEADBAND_DEG = 1.0;

    private HiveLimelight limelight;
    private Drivetrain drivetrain;
    private BallColor target = BallColor.YELLOW;

    @Override
    public void init() {
        limelight = new HiveLimelight(hardwareMap);
        limelight.start(LimelightConstants.PIPELINE_BALL_DETECTOR);
        drivetrain = Constants.createDrivetrain(hardwareMap);
        telemetry.setMsTransmissionInterval(50);
    }

    @Override
    public void loop() {
        if (gamepad1.xWasPressed()) {
            target = BallColor.values()[(target.ordinal() + 1) % BallColor.values().length];
        }

        limelight.update();
        BallDetection ball = limelight.confirmedBestOf(target);

        DrivePowers powers;
        String mode;
        if (gamepad1.right_bumper && ball != null) {
            double turn = Math.abs(ball.txDeg) < TX_DEADBAND_DEG ? 0 : -TURN_KP * ball.txDeg;
            double forward = 0;
            if (ball.hasRangeEstimate()) {
                forward = DRIVE_KP * (ball.distanceIn() - STANDOFF_IN);
            }
            powers = new DrivePowers(clamp(forward, MAX_DRIVE), 0, clamp(turn, MAX_TURN));
            mode = "AUTO";
        } else if (gamepad1.right_bumper) {
            powers = DrivePowers.zero();
            mode = "AUTO (no confirmed " + target + ")";
        } else {
            powers = new DrivePowers(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            mode = "MANUAL";
        }
        drivetrain.drive(powers, true);

        telemetry.addData("Mode", mode);
        telemetry.addData("Target", "%s (X to cycle)", target);
        telemetry.addData("Powers", powers.toString());
        telemetry.addData("Ball", ball == null ? "-" : ball.toString());
        if (!Constants.TUNED) telemetry.addLine("Constants.TUNED is false: motor names/directions are placeholders");
        limelight.addTelemetry(telemetry);
        telemetry.update();
    }

    private static double clamp(double v, double limit) {
        return Math.max(-limit, Math.min(limit, v));
    }

    @Override
    public void stop() {
        drivetrain.stop();
        limelight.stop();
    }
}
