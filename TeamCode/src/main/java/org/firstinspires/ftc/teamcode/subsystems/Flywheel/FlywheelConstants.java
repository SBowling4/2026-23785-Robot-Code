package org.firstinspires.ftc.teamcode.subsystems.Flywheel;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class FlywheelConstants {
    public static final String LEFT_FLYWHEEL_MOTOR_NAME = "FlywheelMotorLeft";
    public static final String RIGHT_FLYWHEEL_MOTOR_NAME = "FlywheelMotorRight";

    public static final double TICKS_PER_REVOLUTION = 25.0;

    public static final double FLYWHEEL_RADIUS = 2;

    public static double kP = 0.1;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double kS = 1.709;
    public static double kV = .0234;
    public static double kA = 0.0;

    public static double CLOSE_VELOCITY = 227.5;
    public static double MID_VELOCITY = 275;
    public static double FAR_VELOCITY = 340;

}
