package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelSubsystem;

@TeleOp(name="Flywheel Tuning")
public class FlywheelTuning extends OpMode {
    FlywheelSubsystem flywheelSubsystem;
    FeederSubsystem feederSubsystem;

    double targetVolts = 0.0;
    double targetVelocity = 0.0;

    @Override
    public void init() {
        flywheelSubsystem = FlywheelSubsystem.getInstance(hardwareMap);
        flywheelSubsystem.init();

        feederSubsystem = FeederSubsystem.getInstance(hardwareMap);
    }

    @Override
    public void loop() {
        flywheelSubsystem.loop();

        if (gamepad1.a) {
            targetVelocity = 50;
        } else if (gamepad1.b) {
            targetVelocity = 100;
        } else if (gamepad1.x) {
            targetVelocity = 150;
        } else if (gamepad1.y) {
            targetVelocity = 200;
        } else {
            targetVelocity = 0;
        }

        if (gamepad1.left_bumper) {
            feederSubsystem.feed();
        } else {
            feederSubsystem.stop();
        }

        flywheelSubsystem.setVelocity(targetVelocity);


        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("Velocity (RAD/SEC)", flywheelSubsystem.getVelocity());
//        packet.put("Target Velocity", targetVelocity);
        packet.put("Velocity Error", Math.abs(targetVelocity - flywheelSubsystem.getVelocity()));

        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetry.addData("Velocity Error", Math.abs(targetVelocity - flywheelSubsystem.getVelocity()));


        telemetry.addData("Velocity (RAD/SEC)", flywheelSubsystem.getVelocity());
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Target Volts", flywheelSubsystem.lastTargetVolts);


        telemetry.addLine();

        telemetry.addData("Robot Voltage", flywheelSubsystem.getRobotVoltage());

        telemetry.update();
    }
}