package org.firstinspires.ftc.teamcode.drive.hive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class HiveConstants {

    public static final double  driveSpeed = 0.6;
    public static final double  turnSpeed = 0.5;
    public static final double liftSpeed = 0.8;
    public static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;
    public static final double      WHEEL_DIAMETER_INCHES   = 75/25.4 ;     // For figuring circumference
    public static final double      DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.

    public static final double      COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final double     STRAFE_INCH_PER_REV     = 9;
    public static final double     STRAFE_COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            STRAFE_INCH_PER_REV;
    public static double     DEGREE_PER_REV          = 39;
    public static double      COUNTS_PER_DEGREE       = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            DEGREE_PER_REV;
    public static final double    LIFT_INCH_PER_REV       = 1;
    public static final double     LIFT_COUNTS_PER_INCH    = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            LIFT_INCH_PER_REV;


    public static final double  clawMin = -1;
    public static final double  clawMax = 1;





}
