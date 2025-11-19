package org.firstinspires.ftc.teamcode.lib.pedropathing;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveConstants;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(35.35);

    public static PathConstraints pathConstraints = new PathConstraints(.99, 100, 1, 1);

    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(DriveConstants.RIGHT_FRONT_MOTOR_NAME)
            .rightRearMotorName(DriveConstants.RIGHT_BACK_MOTOR_NAME)
            .leftFrontMotorName(DriveConstants.LEFT_FRONT_MOTOR_NAME)
            .leftRearMotorName(DriveConstants.LEFT_BACK_MOTOR_NAME)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .leftPodY(6.140628)
            .rightPodY(-6.140628)
            .strafePodX(-2.139)
            .leftEncoder_HardwareMapName(DriveConstants.LEFT_BACK_MOTOR_NAME)
            .rightEncoder_HardwareMapName(DriveConstants.RIGHT_FRONT_MOTOR_NAME)
            .strafeEncoder_HardwareMapName(DriveConstants.LEFT_FRONT_MOTOR_NAME)
            .rightEncoderDirection(-1)
            .leftEncoderDirection(1)
            .strafeEncoderDirection(1)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                            RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                    )
            );


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .threeWheelIMULocalizer(localizerConstants)
                .build();

    }
}
