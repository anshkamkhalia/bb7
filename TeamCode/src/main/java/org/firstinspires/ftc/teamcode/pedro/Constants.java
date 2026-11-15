package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
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

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("m3");
        c.frontRightName.set("m1");
        c.backLeftName.set("m4");
        c.backRightName.set("m2");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("odom");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-0.4887997634767548);
        c.yPodOffset.set(0.621967390766294);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.18292460182564868);
                Controller secondaryTranslationalForward = Controller.proportional(0.06758575253919587);
                Controller primaryTranslationalLateral = Controller.proportional(0.22367866124618027);
                Controller secondaryTranslationalLateral = Controller.proportional(0.08264328852655874);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01518624989454038));
                c.brake.set(Controller.proportionalFeedforward(0.012908312410359323));

                c.headingFeedback.set(Controller.proportional(2.751350726427456));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.041585064964670064, 0.003244394647415319));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07283473404690263, 0.05210234535073746));
                c.quadraticBrakeCoefficients.set(Matrix.diag(7.396816657419462E-4, 0.0011782841917421798));

                c.maxAchievableForwardVelocity.set(64.98897450696029);
                c.maxAchievableStrafeVelocity.set(57.478603370834534);
                c.naturalForwardDeceleration.set(39.87779685471147);
                c.naturalStrafeDeceleration.set(59.76903154011675);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}