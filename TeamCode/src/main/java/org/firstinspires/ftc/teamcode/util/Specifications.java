package org.firstinspires.ftc.teamcode.util;

//Cir is circumference

public class Specifications {
    public final double length = 43.18;
    public final double wheelCir = 0;
    public final int FINALPOS = 1230;
    //b
    public final double SideOdometryToCentre = 16.188;
    public final double lengthFromOdometrySideToFront = 13.0705;
    public final double odometryCir = 3.5*Math.PI;
    public final double odometryTick = 8192;
    //beta
    public final double sideOdometryAngleFromCentre = 0;
    public final double frontOdometryAngleFromCentre = 0;

    public final double distance_sensor_back_margin = 18.8;
    public final double distance_sensor_back_space = 18.2;
    public final double distance_sensor_left_margin = 16.4;
    public final double distance_sensor_right_margin = 16.4;
    public final double distance_sensor_front_margin = 0;
    public final double distance_sensor_front_space = 0;
    public final double distance_sensor_right_space = 13.6;
    public final double distance_sensor_left_space = 13.6;
    public final double front_sensors_to_middle = 0;
    public final double right_sensors_to_middle = 0;
    public final double side_to_center = 0;


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
    public static final String EXTENSION_MOTOR_MAIN = "extension1";
    public static final String EXTENSION_MOTOR_AUX1 = "extension2";
    public static final String EXTENSION_MOTOR_AUX2 = "extension3";
    public static final String FTLF_MOTOR = "leftForward";
    public static final String FTRT_MOTOR = "rightForward";
    public static final String BKLF_MOTOR = "leftBack";
    public static final String BKRT_MOTOR = "rightBack";
    public static final String BK_ENCODER = "extension3"; //port 2
    public static final String LF_ENCODER = "extension2"; //port 1
    public static final String RT_ENCODER = "rt_encoder"; //port 3
    public static final String ROTATION = "rotation";
    public static final String CLAW = "claw";
    public static final String TILT = "tilt";
    public static final String LINKAGE = "linkage";
    public static final String LINKAGE_POTENTIOMETER = "potentiometer";
    public static final String INTAKE_LEFT = "intakeLeft";
    public static final String INTAKE_RIGHT = "intakeRight";
    public static final String IMU = "imu";
    public static final String DS_FT_LF = "distance_sensor_frontLeft";
    public static final String DS_FT_RT = "distance_sensor_frontRight";
    public static final String DS_BK_LF = "distance_sensor_backLeft";
    public static final String DS_BK_RT = "distance_sensor_backRight";
    public static final String DS_LF_FT = "distance_sensor_leftFront";
    public static final String DS_LF_BK = "distance_sensor_leftBack";
    public static final String DS_RT_FT = "distance_sensor_rightFront";
    public static final String DS_RT_BK = "distance_sensor_rightBack";
    public static final String DS_LF = "distance_sensor_left";
    public static final String DS_RT = "distance_sensor_right";
    public static final String DS_OUTPUT = "distance_sensor_output";
    public static final String DS_INTAKE = "distance_sensor_intake";
    public static final String INITIAL_CAM = "Webcam";
    public static final String AIMING_CAM = "Webcam 2";

    public enum NavSystem{
        IMU,
        ODOMETRY,
        MIXED
    }
    public NavSystem navSystem = NavSystem.ODOMETRY;

    public Specifications() {
    }
}
