package org.firstinspires.ftc.teamcode.subsystems.Drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveSubsystem {

    private static DriveSubsystem instance;


    HardwareMap hardwareMap;

    MotorEx frontLeft, frontRight, backLeft, backRight;
    Motor.Encoder encoderLeft, encoderRight, encoderAux;
    IMU gyro;

    HolonomicOdometry odometry;
    MecanumDrive mecanum;

    Telemetry telemetry;
    Gamepad gamepad1;


    public DriveSubsystem(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
    }

    public void init() {
        frontLeft = new MotorEx(hardwareMap, DriveConstants.LEFT_FRONT_MOTOR_NAME);
        frontRight = new MotorEx(hardwareMap, DriveConstants.RIGHT_FRONT_MOTOR_NAME);
        backLeft = new MotorEx(hardwareMap, DriveConstants.LEFT_BACK_MOTOR_NAME);
        backRight = new MotorEx(hardwareMap, DriveConstants.RIGHT_BACK_MOTOR_NAME);

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        encoderLeft = frontLeft.encoder.setDistancePerPulse(DriveConstants.TICKS_TO_INCHES);
        encoderRight = frontRight.encoder.setDistancePerPulse(DriveConstants.TICKS_TO_INCHES);
        encoderAux = backLeft.encoder.setDistancePerPulse(DriveConstants.TICKS_TO_INCHES);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        gyro = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP, //TODO: Update
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        gyro.initialize(parameters);

        odometry = new HolonomicOdometry(() -> encoderLeft.getDistance(), () -> encoderRight.getDistance(),
                () -> encoderAux.getDistance(), 17, 0);
    }

    public void loop(){
        mecanum.driveFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gyro.getRobotYawPitchRollAngles().getYaw());

        odometry.update(encoderLeft.getDistance(), encoderRight.getDistance(), encoderAux.getDistance());

        if (gamepad1.y) {
            zeroGyro();
        }

        telemetry.addLine("//Odometry//");
        telemetry.addData("X Position (in)", getPose().getX());
        telemetry.addData("Y Position (in)", getPose().getY());
        telemetry.addData("Heading (rad)", getPose().getHeading());
        telemetry.addLine();
        telemetry.addLine();

        telemetry.update();
    }

    public void zeroGyro() {
        gyro.resetYaw();
    }

    public void resetPose() {
        odometry.updatePose(new Pose2d());
    }

    public Pose2d getPose() {
        return odometry.getPose();
    }

    public static DriveSubsystem getInstance(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1) {
        if (instance == null) {
            instance = new DriveSubsystem(telemetry, hardwareMap, gamepad1);
        }
        return instance;
    }

    public DriveSubsystem getInstance() {
        if (instance == null) {
            throw new IllegalStateException("DriveSubsystem not initialized. Call getInstance(telemetry, hardwareMap, gamepad1) first.");
        }
        return instance;
    }
}