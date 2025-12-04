package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Vision.Vision;
import org.firstinspires.ftc.teamcode.util.Alliance;

@Autonomous
public class ShortBlue_3 extends OpMode {
    private Follower follower;
    private Timer autoTimer, pathTimer;

    ShooterSubsystem shooterSubsystem;
    FlywheelSubsystem flywheelSubsystem;
    FeederSubsystem feederSubsystem;
    IntakeSubsystem intakeSubsystem;
    Vision vision;

    public enum PathState {
        DRIVE_STARTPOS_SHOOTPOS,
        SHOOT_PRELOAD,
        END
    }

    private PathState pathState;

    private final Pose startPose = new Pose(20.101083032490973, 124.24548736462094, Math.toRadians(144));
    private final Pose shootPose = new Pose(56.317689530685925, 87.33574007220216, Math.toRadians(144));

    private PathChain driveStartShoot;

    public void buildPaths() {
        driveStartShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOTPOS:
                follower.followPath(driveStartShoot, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    shooterSubsystem.shoot(false);
                    feederSubsystem.autoFeed();
                }

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 8) {
                    setPathState(PathState.END);
                }
                break;
            case END:
                flywheelSubsystem.stop();
                shooterSubsystem.setAngle(0);
                feederSubsystem.stop();
            default:
                break;
        }
    }


    @Override
    public void init() {
        Robot.alliance = Alliance.BLUE;
        Robot.isTele = false;

        telemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        pathState = PathState.DRIVE_STARTPOS_SHOOTPOS;

        pathTimer = new Timer();
        autoTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        intakeSubsystem = IntakeSubsystem.getInstance(hardwareMap, gamepad1);
        flywheelSubsystem = FlywheelSubsystem.getInstance(hardwareMap, gamepad1);
        shooterSubsystem = ShooterSubsystem.getInstance(hardwareMap, gamepad1, gamepad2);
        feederSubsystem = FeederSubsystem.getInstance(hardwareMap, gamepad1);
        vision = Vision.getInstance(hardwareMap);



        intakeSubsystem.init();
        flywheelSubsystem.init();
        shooterSubsystem.init();
        feederSubsystem.init();
        vision.init();

        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        vision.start();

        autoTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        vision.loop();

        follower.update();
        statePathUpdate();

        telemetry.addData("State", pathState.toString());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
        telemetry.addLine();


        telemetry.addData("Target Angle", shooterSubsystem.targetPos);
        telemetry.addData("Current Angle", shooterSubsystem.getPosition());
        telemetry.addLine();

        telemetry.addData("Flywheel Velocity", flywheelSubsystem.getVelocity());
        telemetry.addData("Flywheel Target", flywheelSubsystem.lastTargetRadPerSec);
        telemetry.addData("Flywheel Volts", flywheelSubsystem.lastTargetVolts);
        telemetry.addLine();



        telemetry.addLine("//Vision//");
        telemetry.addData("LL Valid", vision.llValid);
        telemetry.addData("Has Tag", vision.hasTag);
        telemetry.addData("Ta", vision.getTa().orElse(-1.0));
        telemetry.addData("Tx", vision.getTx().orElse(-1.0));
        telemetry.addData("Ty", vision.getTy().orElse(-1.0));
        telemetry.addData("Distance", vision.getDistance().orElse(-1.0));
        telemetry.addLine();
    }

}
