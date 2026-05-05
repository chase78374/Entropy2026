package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(15.2)
            .forwardZeroPowerAcceleration(-34 / 1.5)
            .lateralZeroPowerAcceleration(-68.5 / 1.5)
            .headingPIDFCoefficients(new PIDFCoefficients(1,.0,.001,.095))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(.05,.0,.0,.6,.03))
            //.translationalPIDFCoefficients(new PIDFCoefficients(1, 0, 0.05, 0.03))
            //.drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1,0.0,0.01,0.6,0.0))
            //.headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.01, 0.02))//default
            .translationalPIDFCoefficients(new PIDFCoefficients(.0375,.0,.0005,.0225))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(.15, 0, 0.15, .015))
            .centripetalScaling(0.0005)
            .useSecondaryHeadingPIDF(true);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 2.25, 0.25);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(81.3465677096149)
            .yVelocity(56.69571427472933);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .driveEncoderLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                /* other builder steps */
                .build();
    }

    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BR")
            .leftFrontMotorName("FL")
            .robotLength(8.5)
            .robotWidth(14)
            .leftFrontEncoderDirection(Encoder.FORWARD)
            .leftRearEncoderDirection(Encoder.FORWARD)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.REVERSE);

    public static Pose startingPose;// = new Pose(0,0,Math.toRadians(90));
}
