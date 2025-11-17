package org.firstinspires.ftc.teamcode.subsystems.Drive;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.subsystems.Vision.Vision;

public class DriveSubsystem {

    private MotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU gyro;

    private PIDController alignPID;
    public MecanumDrive mecanum;

    public final HardwareMap hardwareMap;
    private final Gamepad gamepad1;

    private Vision vision;

    private static DriveSubsystem instance;



    public DriveSubsystem(HardwareMap hardwareMap, Gamepad gamepad1) {
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


        alignPID = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);

        gyro = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        gyro.initialize(parameters);

        mecanum = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        vision = Vision.getInstance(hardwareMap);
    }

    public void loop(){
        alignPID.setP(DriveConstants.kP);

        double leftX = applyDeadband(-gamepad1.left_stick_x);
        double leftY = applyDeadband(-gamepad1.left_stick_y);
        double rightX = applyDeadband(gamepad1.right_stick_x);

//        mecanum.driveFieldCentric(leftX, leftY, rightX, gyro.getRobotYawPitchRollAngles().getYaw());
        mecanum.driveRobotCentric(leftX, leftY, rightX);


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
        if (vision.getTx().isEmpty()) return;

        double tx = vision.getTx().get();

        double target = 1;

        double power = alignPID.calculate(tx, target);

        mecanum.driveRobotCentric(strafe, forward, -power);
    }


    public void zeroGyro() {
        gyro.resetYaw();
    }


    public void stop() {
        frontLeft.stopMotor();
        backLeft.stopMotor();
        frontRight.stopMotor();
        backRight.stopMotor();
    }

    public static DriveSubsystem getInstance(HardwareMap hardwareMap, Gamepad gamepad1) {
        if (instance == null) {
            instance = new DriveSubsystem(hardwareMap, gamepad1);
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