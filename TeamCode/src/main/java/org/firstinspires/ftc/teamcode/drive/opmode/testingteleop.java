package org.firstinspires.ftc.teamcode.drive.opmode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.DroneShooter;
import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.command.MultiMotorCommand;
import org.firstinspires.ftc.teamcode.command.OutputCommand;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;
import org.firstinspires.ftc.teamcode.util.Specifications;
import org.firstinspires.ftc.teamcode.util.TimerList;

import java.lang.reflect.Executable;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

@TeleOp(name = "Pr0TeensTeleOpV1")
public class testingteleop extends LinearOpMode {
    private MecanumSubsystem mecanumSubsystem;
    private MecanumCommand mecanumCommand;
    private IMUSubsystem imuSubsystem;
    private MultiMotorSubsystem multiMotorSubsystem;
    private MultiMotorCommand multiMotorCommand;
    private OdometrySubsystem odometrySubsystem;
    private GyroOdometry gyroOdometry;
    private DroneShooter droneShooter;
    private Servo hangingServoL;
    private Servo hangingServoR;
    private DcMotor hangingMotor;
    private ElapsedTime timer;
    private TimerList timers = new TimerList();

    private IntakeCommand intakeCommand;

    private enum RUNNING_STATE { //mini "threads" to run (is actually run in main thread, just controlled simultaneously)
        LOWER_LIFT, RAISE_LIFT
    }

    private double rotation = 0;

    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() throws InterruptedException {
        droneShooter = new DroneShooter(hardwareMap);
        imuSubsystem = new IMUSubsystem(hardwareMap);
        mecanumSubsystem = new MecanumSubsystem(hardwareMap); //MAKE SURE THIS IS INSTANTIATED BEFORE ODOMETRY SUBSYSTEM
        gyroOdometry = new GyroOdometry(odometrySubsystem, imuSubsystem);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);
        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem,  gyroOdometry, this);
        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);
        intakeCommand = new IntakeCommand(hardwareMap);
        intakeCommand.lowerIntake();

        //INITIALIZES THE HANGING SERVO
        hangingServoL = hardwareMap.get(Servo.class, Specifications.HANGING_SERVO_L);
        hangingServoL.setPosition(0.35);

        hangingServoR = hardwareMap.get(Servo.class, Specifications.HANGING_SERVO_R);
        hangingServoR.setPosition(0.35);
////
////        //INITIALIZES THE HANGING MOTOR
        hangingMotor = hardwareMap.dcMotor.get(Specifications.HANGING_MOTOR);
        hangingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangingMotor.setPower(0);
        boolean hangingArmInPlace = false;
        boolean robotIsHanging = false;
        intakeCommand.lowerIntake();
        //odometrySubsystem.reset();
        imuSubsystem.resetAngle();
        Executor executor = Executors.newFixedThreadPool(4);

        waitForStart();
        Gamepad currentGamepad1 = new Gamepad();
        CompletableFuture.runAsync(this::moveRobot, executor);
        CompletableFuture.runAsync(this::updateTelemetry, executor);





        while (opModeIsActive()) {
            }


        }
    public void moveRobot(){
        while (opModeIsActive()) {
            mecanumSubsystem.fieldOrientedMove(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gyroOdometry.theta);
        }
    }

    public void updateTelemetry() {
        while (opModeIsActive()) {
            telemetry.addData("LEFT_X", gamepad1.left_stick_x);
            telemetry.addData("LEFT_Y", gamepad1.left_stick_y);
            telemetry.addData("rotation", gamepad1.right_stick_x);
            telemetry.addData("current theta", gyroOdometry.theta);
            telemetry.update();
        }
    }
}