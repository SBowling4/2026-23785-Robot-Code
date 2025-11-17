package org.firstinspires.ftc.teamcode.subsystems.Drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
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
import org.firstinspires.ftc.teamcode.lib.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Vision.VisionSubsystem;

public class DriveSubsystem {

    MotorEx frontLeft, frontRight, backLeft, backRight;
    Motor.Encoder encoderLeft, encoderRight, encoderAux;
    IMU gyro;

    PIDController alignPID;

    HolonomicOdometry odometry;
    public MecanumDrive mecanum;

    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepad1;

    private VisionSubsystem visionSubsystem;

    private static DriveSubsystem instance;



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

        backRight.setInverted(true);
        frontRight.setInverted(true);


        encoderLeft = frontLeft.encoder.setDistancePerPulse(DriveConstants.TICKS_TO_INCHES);
        encoderRight = frontRight.encoder.setDistancePerPulse(DriveConstants.TICKS_TO_INCHES);
        encoderAux = backLeft.encoder.setDistancePerPulse(DriveConstants.TICKS_TO_INCHES);

        alignPID = new PIDController(DriveConstants.kP, 0, 0);

        gyro = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        gyro.initialize(parameters);

        odometry = new HolonomicOdometry(() -> encoderLeft.getDistance(), () -> encoderRight.getDistance(),
                () -> encoderAux.getDistance(), 17, 0);

        mecanum = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        visionSubsystem = VisionSubsystem.getInstance(hardwareMap);
    }

    public void loop(){
        alignPID.setP(DriveConstants.kP);

        double leftX = applyDeadband(-gamepad1.left_stick_x);
        double leftY = applyDeadband(-gamepad1.left_stick_y);
        double rightX = applyDeadband(gamepad1.right_stick_x);

//        mecanum.driveFieldCentric(leftX, leftY, rightX, gyro.getRobotYawPitchRollAngles().getYaw());
        mecanum.driveRobotCentric(leftX, leftY, rightX);

        odometry.update(encoderLeft.getDistance(), encoderRight.getDistance(), encoderAux.getDistance());


        if (gamepad1.x) {
            align(leftX, leftY);
        }

    }

    public double applyDeadband(double value) {
        if (Math.abs(value) < DriveConstants.DEADBAND) {
            return 0.0;
        } else {
            return value;
        }
    }

    public void align(double strafe, double forward) {
        if (visionSubsystem.getTx().isEmpty()) return;

        double tx = visionSubsystem.getTx().get();

        double target = 1;

        double power = alignPID.calculate(tx, target);

        mecanum.driveRobotCentric(strafe, forward, -power);
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

    public double getDistanceTraveled() {
        return (encoderLeft.getDistance() + encoderRight.getDistance()) / 2.0;
    }

    public void stop() {
        frontLeft.stopMotor();
        backLeft.stopMotor();
        frontRight.stopMotor();
        backRight.stopMotor();
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