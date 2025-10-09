package org.firstinspires.ftc.teamcode.subsystems.Flywheel;

import com.acmerobotics.dashboard.config.Config;

@Config
public class FlywheelConstants {
    public static final String LEFT_FLYWHEEL_MOTOR_NAME = "FlywheelMotorLeft";
    public static final String RIGHT_FLYWHEEL_MOTOR_NAME = "FlywheelMotorRight";

    public static final double TICKS_PER_REVOLUTION = 25.0;

    public static double kP = 0.1;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double kS = 1.94;
    public static double kV = .029;
    public static double kA = 0.0;

    public static final double NOMINAL_VOLTAGE = 12.0;
}
