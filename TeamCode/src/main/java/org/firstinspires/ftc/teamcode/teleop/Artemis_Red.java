package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;

/**
 * DO NOT EDIT THIS CLASS STANDARDLY. JUST WORK ON BLUE, COPY PASTE OVER WHEN READY FOR COMP AND REPLACED ALLIANCE
 */
@TeleOp(name = "Artemis_Red", group = "Orion")
public class Artemis_Red extends OpMode {

    DriveSubsystem driveSubsystem;
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;
    VisionSubsystem visionSubsystem;
    FlywheelSubsystem flywheelSubsystem;
    FeederSubsystem feederSubsystem;

    private boolean lastUpState = false;
    private boolean lastDownState = false;


    @Override
    public void init() {
        Robot.alliance = Alliance.RED;

        telemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        driveSubsystem = DriveSubsystem.getInstance(telemetry, hardwareMap, gamepad1);
        driveSubsystem.init();

        intakeSubsystem = IntakeSubsystem.getInstance(hardwareMap, gamepad1);
        intakeSubsystem.init();

        flywheelSubsystem = FlywheelSubsystem.getInstance(hardwareMap, gamepad1, gamepad2);
        flywheelSubsystem.init();

        shooterSubsystem = ShooterSubsystem.getInstance(hardwareMap, gamepad1, gamepad2);
        shooterSubsystem.init();

        feederSubsystem = FeederSubsystem.getInstance(hardwareMap, gamepad1);
        feederSubsystem.init();

        visionSubsystem = VisionSubsystem.getInstance(hardwareMap);
        visionSubsystem.init();

    }

    @Override
    public void loop() {
        driveSubsystem.loop();
        intakeSubsystem.loop();
        shooterSubsystem.loop();
        flywheelSubsystem.loop();
        feederSubsystem.loop();
        visionSubsystem.loop();

        boolean currentUpState = gamepad1.dpad_up;
        boolean currentDownState = gamepad1.dpad_down;

        if (currentUpState && !lastUpState) {
            Robot.advanceState();
        }

        if (currentDownState && !lastDownState) {
            Robot.reverseState();
        }

        lastUpState = currentUpState;
        lastDownState = currentDownState;

        if (!Robot.tuningMode) {
            if (gamepad1.left_bumper) {
                flywheelSubsystem.setVelocity(Robot.shooterState.velocity);
                shooterSubsystem.setAngle(Robot.shooterState.angle);
            } else {
                flywheelSubsystem.stop();
                shooterSubsystem.setAngle(0);
            }
        }





        telemetry.addLine("//Shooter//");
        telemetry.addData("Shooter State", Robot.tuningMode ? "TUNING" : Robot.shooterState.toString());
        telemetry.addLine();

        telemetry.addData("Tuning Target Angle", shooterSubsystem.tuningPos);
//        telemetry.addData("Target Angle", shooterSubsystem.targetPos);
        telemetry.addData("Current Angle", shooterSubsystem.position);
        telemetry.addLine();

        telemetry.addData("Flywheel Velocity", flywheelSubsystem.getVelocity());
//        telemetry.addData("Flywheel Target", flywheelSubsystem.lastTargetRadPerSec);
        telemetry.addData("Tuning Flywheel Target", flywheelSubsystem.tuningVelocity);
        telemetry.addData("Flywheel Volts", flywheelSubsystem.lastTargetVolts);
        telemetry.addLine();



        telemetry.addLine("//Vision//");
        telemetry.addData("LL Valid", visionSubsystem.llValid);
        telemetry.addData("Ta", visionSubsystem.getTa());
        telemetry.addData("Distance", visionSubsystem.getDistance());
//        telemetry.addData("Horz", visionSubsystem.getHorizontalAngle().isPresent() ? visionSubsystem.getHorizontalAngle().get() : -1);
//        telemetry.addData("Vert", visionSubsystem.getVerticalAngle().isPresent() ? visionSubsystem.getVerticalAngle().get() : -1);
//        telemetry.addData("Distance", visionSubsystem.getDistance().isPresent() ? visionSubsystem.getDistance().get() : -1);



        telemetry.update();
    }

}