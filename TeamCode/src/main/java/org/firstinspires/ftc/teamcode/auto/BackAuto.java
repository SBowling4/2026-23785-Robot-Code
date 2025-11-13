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
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterSubsystem;

@Autonomous(name = "BackAuto")
public class BackAuto extends OpMode {

    private DriveSubsystem driveSubsystem;
    private FlywheelSubsystem flywheelSubsystem;
    private FeederSubsystem feederSubsystem;
    private ShooterSubsystem shooterSubsystem;

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

        time.startTime();
    }

    @Override
    public void loop() {
        driveSubsystem.setTelemetry();
        double t = time.seconds();
        telemetry.addData("Time", t);

        if (isFinished) {
            driveSubsystem.stop();
            flywheelSubsystem.setPower(0);
            feederSubsystem.stop();
            shooterSubsystem.setAngle(0);
            telemetry.update();
            return;
        }

        flywheelSubsystem.setVelocity(FlywheelConstants.FAR_AUTO_VELOCITY);
        shooterSubsystem.setAngle(ShooterConstants.FAR_ANGLE);

        if (t < .5) {
            driveSubsystem.mecanum.driveRobotCentric(0, .5, 0);
        } else if (t < 9){
            driveSubsystem.stop();
        }

        if (t > 4 && t < 9) {
            feederSubsystem.autoFeed();
        }

        if (t < 10 && t > 9) {
            feederSubsystem.stop();
            driveSubsystem.mecanum.driveRobotCentric(0, .5, 0);
        }

        if (t > 10) {
            isFinished = true;
        }

        telemetry.addData("Velocity", flywheelSubsystem.findVelocity());
        telemetry.addData("Target Velocity", flywheelSubsystem.lastTargetRadPerSec);
        telemetry.update();
    }


    @Override
    public void start() {
        time.reset();
    }
}
