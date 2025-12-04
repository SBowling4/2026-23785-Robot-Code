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
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Vision.Vision;
import org.firstinspires.ftc.teamcode.util.Alliance;

@Autonomous
public class ShortBlue_6 extends OpMode {
    private Follower follower;
    private Timer autoTimer, pathTimer;

    ShooterSubsystem shooterSubsystem;
    FlywheelSubsystem flywheelSubsystem;
    FeederSubsystem feederSubsystem;
    IntakeSubsystem intakeSubsystem;
    Vision vision;

    public enum PathState {
        DRIVE_START_POS_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_READY_PICKUP_POS,
        PICKUP,
        DRIVE_BACK_SHOOT_POS,
        SHOOT_PICKUP,
        END
    }

    private PathState pathState;

    private final Pose startPose = new Pose(20.101083032490973, 124.24548736462094, Math.toRadians(144));
    private final Pose shootPose = new Pose(56.317689530685925, 87.33574007220216, Math.toRadians(144));
    private final Pose readyPickupPose = new Pose(41.588447653429604, 84.21660649819495, Math.toRadians(180));
    private final Pose pickupPose = new Pose(14.382671480144404, 83.87003610108303, Math.toRadians(180));

    private PathChain driveStartShoot, driveReadyPickup, drivePickup, drivePickupShoot;

    public void buildPaths() {
        driveStartShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveReadyPickup = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, readyPickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), readyPickupPose.getHeading())
                .build();

        drivePickup = follower.pathBuilder()
                .addPath(new BezierLine(readyPickupPose, pickupPose))
                .setLinearHeadingInterpolation(readyPickupPose.getHeading(), pickupPose.getHeading())
                .build();

        drivePickupShoot = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose, shootPose))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_START_POS_SHOOT_POS:
                follower.followPath(driveStartShoot, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    shooterSubsystem.shoot(false);
                    feederSubsystem.autoFeed();
                }

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 8) {
                    follower.followPath(drivePickupShoot);
                    setPathState(PathState.DRIVE_READY_PICKUP_POS);
                }
                break;
            case DRIVE_READY_PICKUP_POS:
                if (!follower.isBusy()) {
                    follower.followPath(drivePickup);
                    setPathState(PathState.PICKUP);
                }
            case PICKUP:
                intakeSubsystem.intake();
                feederSubsystem.feed();
                flywheelSubsystem.setPower(1);

                if (!follower.isBusy()) {
                    follower.followPath(drivePickupShoot);
                    setPathState(PathState.DRIVE_BACK_SHOOT_POS);
                }
            case DRIVE_BACK_SHOOT_POS:
                intakeSubsystem.stop();
                feederSubsystem.stop();
                flywheelSubsystem.stop();

                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PICKUP);
                }

            case SHOOT_PICKUP:
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
                intakeSubsystem.stop();
            default:
                break;
        }
    }


    @Override
    public void init() {
        Robot.alliance = Alliance.BLUE;
        Robot.isTele = false;

        telemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        pathState = PathState.DRIVE_START_POS_SHOOT_POS;

        pathTimer = new Timer();
        autoTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        shooterSubsystem = ShooterSubsystem.getInstance(hardwareMap, gamepad1, gamepad2);
        flywheelSubsystem = FlywheelSubsystem.getInstance(hardwareMap, gamepad1);
        feederSubsystem = FeederSubsystem.getInstance(hardwareMap, gamepad1);
        intakeSubsystem = IntakeSubsystem.getInstance(hardwareMap, gamepad1);
        vision = Vision.getInstance(hardwareMap);

        flywheelSubsystem.init();
        shooterSubsystem.init();
        feederSubsystem.init();
        intakeSubsystem.init();
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
