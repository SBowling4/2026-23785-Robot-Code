package org.firstinspires.ftc.teamcode.subsystems.Drive;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class DriveConstants {
    public static final String LEFT_FRONT_MOTOR_NAME = "fl";
    public static final String LEFT_BACK_MOTOR_NAME = "bl";
    public static final String RIGHT_FRONT_MOTOR_NAME = "fr";
    public static final String RIGHT_BACK_MOTOR_NAME = "br";

//    public static final  double TICKS_TO_INCHES = 8192 / ((35 / 25.4) * Math.PI);
    public static final double DEADBAND = .2;

    public static double kP = 0.05;
    public static double kI = 0.0;
    public static double kD = 0.0;

}
