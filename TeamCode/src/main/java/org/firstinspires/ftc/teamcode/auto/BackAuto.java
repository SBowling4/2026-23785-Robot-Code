package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterSubsystem;

@Autonomous(name = "BackAuto")
public class BackAuto extends OpMode {

    private DriveSubsystem driveSubsystem;
    private FlywheelSubsystem flywheelSubsystem;
    private FeederSubsystem feederSubsystem;
    private ShooterSubsystem shooterSubsystem;

    private final ElapsedTime stateTimer = new ElapsedTime();

    private enum AutoState {
        SPIN_UP,
        FEED,
        WAIT_FOR_PIECE,
        PAUSE,
        DRIVE_FORWARD,
        DONE
    }

    private AutoState currentState = AutoState.SPIN_UP;

    // --- Parameters ---
    private static final double FLYWHEEL_SPINUP_TIME = 2.0;
    private static final double FEED_TIME = 3;
    private static final double PAUSE_TIME = 2.0;
    private static final double DRIVE_TIME = 1.5;
    private static final double DRIVE_POWER = 0.4;
    private static final int MAX_SHOTS = 3;  // Number of feed cycles before driving

    private int shotCount = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driveSubsystem = DriveSubsystem.getInstance(telemetry, hardwareMap, gamepad1);
        driveSubsystem.init();

        flywheelSubsystem = FlywheelSubsystem.getInstance(hardwareMap, gamepad1, telemetry);
        flywheelSubsystem.init();

        feederSubsystem = FeederSubsystem.getInstance(hardwareMap, gamepad1);
        feederSubsystem.init();

        shooterSubsystem = ShooterSubsystem.getInstance(hardwareMap, gamepad1, telemetry);
        shooterSubsystem.init();

        stateTimer.reset();
    }

    @Override
    public void start() {
        stateTimer.reset();
        currentState = AutoState.SPIN_UP;
        shotCount = 0;
    }

    @Override
    public void loop() {
        telemetry.addData("State", currentState);
        telemetry.addData("Time", stateTimer.seconds());
        telemetry.addData("Shot Count", shotCount);
        telemetry.addLine();
        telemetry.addData("Distance", feederSubsystem.getDistance());
        telemetry.addData("Has Piece", feederSubsystem.hasPiece());

        switch (currentState) {
            case SPIN_UP:
                // Start flywheel
                flywheelSubsystem.setPower(1.0);
                shooterSubsystem.setAngle(25);

                // Wait for flywheel to spin up
                if (stateTimer.seconds() > FLYWHEEL_SPINUP_TIME) {
                    currentState = AutoState.FEED;
                    stateTimer.reset();
                }
                break;

            case FEED:
                // Feed one piece
                feederSubsystem.feed();

                if (stateTimer.seconds() > FEED_TIME) {
                    feederSubsystem.stop();
                    currentState = AutoState.WAIT_FOR_PIECE;
                    stateTimer.reset();
                    shotCount++;
                }
                break;

            case WAIT_FOR_PIECE:
                feederSubsystem.feed();
                // Wait for next piece detected (if there are more)
                if (shotCount >= MAX_SHOTS) {
                    currentState = AutoState.DRIVE_FORWARD;
                    stateTimer.reset();
                } else if (feederSubsystem.hasPiece()) {
                    currentState = AutoState.PAUSE;
                    stateTimer.reset();
                }
                break;

            case PAUSE:
                // Pause before next feed
                if (stateTimer.seconds() > PAUSE_TIME) {
                    currentState = AutoState.FEED;
                    stateTimer.reset();
                }
                break;

            case DRIVE_FORWARD:
                // Drive forward for DRIVE_TIME
                driveSubsystem.mecanum.driveRobotCentric(0, DRIVE_POWER, 0); // forward only
                if (stateTimer.seconds() > DRIVE_TIME) {
                    driveSubsystem.stop();
                    currentState = AutoState.DONE;
                }
                break;

            case DONE:
                flywheelSubsystem.setPower(0);
                feederSubsystem.stop();
                driveSubsystem.stop();
                break;
        }

        telemetry.update();
    }
}
