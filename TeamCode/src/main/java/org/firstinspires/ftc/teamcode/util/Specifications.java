package org.firstinspires.ftc.teamcode.util;

//Cir is circumference

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class Specifications {
    public final double length = 43.18;
    public final int FINALPOS = 1230;
    //b
    public final double SideOdometryToCentre = 18.45;
    public final double lengthFromOdometrySideToFront = 4.7625;
    public final double odometryCir = 3.5*Math.PI;
    public final double odometryTick = 8192;
    //beta
    public final double sideOdometryAngleFromCentre = 0;
    public final double frontOdometryAngleFromCentre = Math.toRadians(67.93);


    public static double WHEEL_RADIUS = 4.8;
    public static final double MAX_RPM = 223;
    public static final double GEAR_RATIO = 19.2;/*30:23 for new provincial robot*/
    public static double MAX_ANGULAR_VEL = MAX_RPM/30*Math.PI;
    public static double MAX_VEL = MAX_ANGULAR_VEL*WHEEL_RADIUS;
    public static double MIN_VEL = -MAX_VEL;

    public static final double v4bLength = 11;
    public static final double intakePointLength = 1.625;
    public static final double xfOfCone = 21.565;
    public static final double maxHeight = 10.25;


    /* Y is axis robot lines up on at the start (horizontal), X is (vertical)
    public static final double LEFT_HIGH_POLE_X = 122;
    public static final double LEFT_HIGH_POLE_Y = 300;
    public static final double SIDE_PICK_UP_X = 50;
    public static final double SIDE_PICK_UP_Y = 150;
     */

    //DS: distance sensor

    public static final String INTAKE_MOTOR = "intakeMotor";
    public static final String INTAKE_SERVO = "intakeLinkage";
    public static final String INTAKE_ROLLER = "intakeRoller";
    public static final String EXTENSION_MOTOR_MAIN = "extension1";
    public static final String EXTENSION_MOTOR_AUX1 = "extension2";
    public static final String FTLF_MOTOR = "leftForward";
    public static final String FTRT_MOTOR = "rightForward";
    public static final String BKLF_MOTOR = "leftBack";
    public static final String BKRT_MOTOR = "rightBack";
    public static final String BK_ENCODER = "intakeMotor"; //3
    public static final String LF_ENCODER = "rightBack"; //port 1?
    public static final String RT_ENCODER = "leftBack"; //port 2?
    public static final String IMU = "imu";

    //output stuff, fill in names later
    public static final String LEFT_OUTPUT_ARM = "leftArm";
    public static final String RIGHT_OUTPUT_ARM = "rightArm";
    public static final String LEFT_OUTPUT_TILT = "leftTilt";
    public static final String RIGHT_OUTPUT_TILT = "rightTilt";
    public static final String PIXEL_GATE = "pixelGate";
    public static final String FIRST_COLOR_SENSOR = "pixelSensor";
    public static final String SECOND_COLOR_SENSOR = "pixelSensor2";
    public enum NavSystem{
        IMU,
        ODOMETRY,
        MIXED
    }
    public NavSystem navSystem = NavSystem.ODOMETRY;

    public Specifications() {
    }
}
