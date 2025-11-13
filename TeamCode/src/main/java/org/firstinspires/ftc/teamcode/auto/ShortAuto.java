package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelConstants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterSubsystem;

@Autonomous(name = "ShortAuto")
public class ShortAuto extends OpMode {

    private DriveSubsystem driveSubsystem;
    private FlywheelSubsystem flywheelSubsystem;
    private FeederSubsystem feederSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private final ElapsedTime time = new ElapsedTime();
    private boolean isFinished = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driveSubsystem = DriveSubsystem.getInstance(telemetry, hardwareMap, gamepad1);
        driveSubsystem.init();

        flywheelSubsystem = FlywheelSubsystem.getInstance(hardwareMap, gamepad1, gamepad2);
        flywheelSubsystem.init();

        shooterSubsystem = ShooterSubsystem.getInstance(hardwareMap, gamepad1, gamepad2);
        shooterSubsystem.init();

        feederSubsystem = FeederSubsystem.getInstance(hardwareMap, gamepad1);
        feederSubsystem.init();

        intakeSubsystem = IntakeSubsystem.getInstance(hardwareMap,gamepad1);
        intakeSubsystem.init();

        time.startTime();
    }

    @Override
    public void loop() {
        double t = time.seconds();
        telemetry.addData("Time", t);

        if (isFinished) {
            driveSubsystem.stop();
            flywheelSubsystem.setPower(0);
            feederSubsystem.stop();
            shooterSubsystem.setAngle(0);
            intakeSubsystem.stop();
            telemetry.update();
            return;
        }

        if (t < 6) {
            closeShoot();
        }


        // Drive backwards for 1 second
        if (t < .85) {
            driveSubsystem.mecanum.driveRobotCentric(0.0, -0.5, 0);
        } else if (t < 2) {
            driveSubsystem.stop();
        }

        // Feed between 1s and 3s
        if (t > 2 && t < 7) {
            driveSubsystem.stop();
            feederSubsystem.autoFeed();
        } else {
            feederSubsystem.stop();
        }

        if (t > 7 && t < 8) {
            driveSubsystem.mecanum.driveRobotCentric(0, -.75, 0);
            stopShooter();
        }
        if (t > 8 && t < 8.625) {
            driveSubsystem.mecanum.driveRobotCentric(0, 0, 0);
        }
        if (t > 8.625 && t < 9.25) {
            driveSubsystem.mecanum.driveRobotCentric(0, 0, -1);
        }
        if (t > 9.25 && t < 11.5) {
            driveSubsystem.mecanum.driveRobotCentric(0, 0.375, 0);
            feederSubsystem.feed();
            intakeSubsystem.intake();
        }
        if (t > 11.5 && t < 12.5) {
            driveSubsystem.mecanum.driveRobotCentric(0, 0, 0);

        }
//        if (t > 11.5 && t < 12.5) {
//            driveSubsystem.mecanum.driveRobotCentric(0, -.5, 0);
//            feederSubsystem.stop();
//        }
//        if (t > 12.5 && t < 13.5) {
//            driveSubsystem.mecanum.driveRobotCentric(0, 0, 0);
//        }
//        if (t > 13.5 && t < 13.875) {
//            driveSubsystem.mecanum.driveRobotCentric(0, 0, 1);
//        }
//        if (t > 13.875 && t < 17) {
//            driveSubsystem.mecanum.driveRobotCentric(0, 0, 0);
//            midShoot();
//        }
//        if (t > 17 && t < 25) {
//            driveSubsystem.mecanum.driveRobotCentric(0, 0, 0);
//            feederSubsystem.autoFeed();
//        }
//        if (t > 25 && t < 26.5) {
//            driveSubsystem.mecanum.driveRobotCentric(.5, 0, 0);
//            feederSubsystem.stop();
//            stopShooter();
//
//        }

        // End after 10s
        if (t > 12.5) {
            isFinished = true;
        }

        telemetry.addData("Velocity", flywheelSubsystem.getVelocity());
        telemetry.addData("Target", flywheelSubsystem.lastTargetRadPerSec);
        telemetry.update();
    }


    @Override
    public void start() {
        time.reset();
    }

    private void closeShoot() {
        flywheelSubsystem.setVelocity(FlywheelConstants.CLOSE_VELOCITY);
        shooterSubsystem.setAngle(ShooterConstants.CLOSE_ANGLE);
    }

    private void midShoot() {
        flywheelSubsystem.setVelocity(FlywheelConstants.MID_VELOCITY);
        shooterSubsystem.setAngle(ShooterConstants.MID_ANGLE);
    }

    private void stopShooter() {
        flywheelSubsystem.stop();
        shooterSubsystem.setAngle(0);
    }
}