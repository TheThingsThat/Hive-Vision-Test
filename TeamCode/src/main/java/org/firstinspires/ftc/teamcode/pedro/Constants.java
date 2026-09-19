package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.vision.HiveLimelight;

/**
 * Robot configuration for Pedro Pathing 3.
 * <p>
 * The drivetrain / localizer / Foresight blocks below are in exactly the format AutoTune generates
 * (run the "Tuning" OpMode, open the AutoTune web page, and paste its output over each block).
 * Until that is done {@link #TUNED} stays false and the test OpModes show a warning.
 */
public class Constants {
    /** Flip to true after pasting AutoTune output below. */
    public static boolean TUNED = false;

    // ------------------------------------------------------------------ drivetrain (Mecanum tuner)

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.frontRightName.set("rightFront");
        c.backLeftName.set("leftBack");
        c.backRightName.set("rightBack");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });

    // ------------------------------------------------------------------ localizer (Pinpoint tuner)

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(0.0);
        c.yPodOffset.set(0.0);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // ------------------------------------------------------------------ Foresight (Foresight tuner)
    // PLACEHOLDER values: they compile and let the vision test OpModes run, but path following will
    // not be accurate until AutoTune's numbers replace them.

    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller primaryTranslationalForward = Controller.proportional(0.1);
        Controller secondaryTranslationalForward = Controller.proportional(0.1);
        Controller primaryTranslationalLateral = Controller.proportional(0.1);
        Controller secondaryTranslationalLateral = Controller.proportional(0.1);

        c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
        c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

        c.coast.set(Controller.proportionalFeedforward(0.02));
        c.brake.set(Controller.proportionalFeedforward(0.02));

        c.headingFeedback.set(Controller.proportional(1.0));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05, 0.002));

        c.linearBrakeCoefficients.set(Matrix.diag(0.05, 0.05));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.002, 0.002));

        c.maxAchievableForwardVelocity.set(60.0);
        c.maxAchievableStrafeVelocity.set(50.0);
        c.naturalForwardDeceleration.set(100.0);
        c.naturalStrafeDeceleration.set(100.0);
    });

    // ------------------------------------------------------------------ factories

    public static Drivetrain createDrivetrain(HardwareMap h) {
        return new Mecanum(h, drivetrainConfig);
    }

    /** Odometry-only localizer (goBILDA Pinpoint). */
    public static Localizer createOdometryLocalizer(HardwareMap h) {
        return new PinpointLocalizer(h, localizerConfig);
    }

    /** Odometry fused with Limelight AprilTag poses through Pedro's Kalman filter. */
    public static VisionFusedLocalizer createVisionLocalizer(HardwareMap h, HiveLimelight limelight) {
        return new VisionFusedLocalizer(createOdometryLocalizer(h), limelight);
    }

    /** The standard Pedro follower: odometry only. */
    public static Follower create(HardwareMap h) {
        return new Follower(createOdometryLocalizer(h), createDrivetrain(h), new Foresight(foresightConfig));
    }

    /**
     * A follower whose localizer is corrected by the Limelight whenever AprilTags are in view.
     * The OpMode still owns the {@link HiveLimelight} lifecycle (start / pipeline / stop).
     */
    public static Follower createWithVision(HardwareMap h, HiveLimelight limelight) {
        return new Follower(createVisionLocalizer(h, limelight), createDrivetrain(h), new Foresight(foresightConfig));
    }
}
