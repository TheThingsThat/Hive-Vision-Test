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

public class Constants {
    /** AutoTune values for the drivetrain, localizer, and Foresight have been applied. */
    public static boolean TUNED = true;

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("front_left_motor");
        c.frontRightName.set("front_right_motor");
        c.backLeftName.set("back_left_motor");
        c.backRightName.set("back_right_motor");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(9.34158805787094);
        c.yPodOffset.set(-6.398817047359437);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.1648036515639273);
                Controller secondaryTranslationalForward = Controller.proportional(0.06089054561819845);
                Controller primaryTranslationalLateral = Controller.proportional(0.25813235229981263);
                Controller secondaryTranslationalLateral = Controller.proportional(0.09537300675129566);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.013513314552251535));
                c.brake.set(Controller.proportionalFeedforward(0.011486317369413804));
                c.headingFeedback.set(Controller.proportional(3.344482938865348));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04246431218874506, 0.007651015010992809));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09404624720945395, 0.08218212811085751));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0010108781114811487, 0.0010645998808842612));

                c.maxAchievableForwardVelocity.set(75.88383143601001);
                c.maxAchievableStrafeVelocity.set(60.291757743960716);
                c.naturalForwardDeceleration.set(35.369348348789096);
                c.naturalStrafeDeceleration.set(56.41847539262139);
            }
    );

    public static Drivetrain createDrivetrain(HardwareMap h) {
        return new Mecanum(h, drivetrainConfig);
    }

    public static Localizer createOdometryLocalizer(HardwareMap h) {
        return new PinpointLocalizer(h, localizerConfig);
    }

    public static VisionFusedLocalizer createVisionLocalizer(HardwareMap h, HiveLimelight limelight) {
        return new VisionFusedLocalizer(createOdometryLocalizer(h), limelight);
    }

    public static Follower create(HardwareMap h) {
        return new Follower(createOdometryLocalizer(h), createDrivetrain(h), new Foresight(foresightConfig));
    }

    public static Follower createWithVision(HardwareMap h, HiveLimelight limelight) {
        return new Follower(createVisionLocalizer(h, limelight), createDrivetrain(h), new Foresight(foresightConfig));
    }
}
