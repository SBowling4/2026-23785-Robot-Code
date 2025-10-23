package org.firstinspires.ftc.teamcode.lib.pedropathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveConstants;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(20);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(DriveConstants.RIGHT_FRONT_MOTOR_NAME)
            .rightRearMotorName(DriveConstants.RIGHT_BACK_MOTOR_NAME)
            .leftFrontMotorName(DriveConstants.LEFT_FRONT_MOTOR_NAME)
            .leftRearMotorName(DriveConstants.LEFT_BACK_MOTOR_NAME)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(DriveConstants.TICKS_TO_INCHES)
            .strafeTicksToInches(DriveConstants.TICKS_TO_INCHES)
            .turnTicksToInches(DriveConstants.TICKS_TO_INCHES)
            .leftPodY(0)
            .rightPodY(0)
            .leftEncoder_HardwareMapName(DriveConstants.LEFT_BACK_MOTOR_NAME)
            .rightEncoder_HardwareMapName(DriveConstants.RIGHT_FRONT_MOTOR_NAME)
            .strafeEncoder_HardwareMapName(DriveConstants.LEFT_FRONT_MOTOR_NAME)
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .build();

    }
}
