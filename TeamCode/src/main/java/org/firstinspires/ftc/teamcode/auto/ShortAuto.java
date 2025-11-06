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

@Autonomous(name = "ShortAuto")
public class ShortAuto extends OpMode {

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

        flywheelSubsystem.setVelocity(FlywheelConstants.CLOSE_VELOCITY);
        shooterSubsystem.setAngle(ShooterConstants.CLOSE_ANGLE);


        // Drive backwards for 1 second
        if (t < .85) {
            driveSubsystem.mecanum.driveRobotCentric(0.0, -0.5, 0);
        } else {
            driveSubsystem.stop();
        }

        // Feed between 1s and 3s
        if (t > 2 && t < 15) {
            feederSubsystem.feed();
        } else {
            feederSubsystem.stop();
        }

        // End after 10s
        if (t > 15) {
            isFinished = true;
        }

        telemetry.update();
    }


    @Override
    public void start() {
        time.reset();
    }
}
