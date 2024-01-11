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
        Gamepad previousGamepad1 = new Gamepad();
        while (opModeIsActive()) {
            boolean hangingTrue = false;
            boolean lastHangingState = false;
            //timers.resetTimer("GameTimeElapsed");
            previousGamepad1.copy(currentGamepad1);
            mecanumCommand.moveGlobalPartial(true, gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            //x is for taking pixels in, b is for spitting pixels out
            if (gamepad1.x) {
                intakeCommand.intakeIn(0.6);
                telemetry.addLine("Intake ran");
            } else if (gamepad1.b) {
                intakeCommand.intakeOut(0.6);
                telemetry.addLine("intake reversed");
            } else {
                intakeCommand.stopIntake();
            }
            //Drone Launcher
            if (gamepad1.back) {
            //    droneShooter.setContinuousServoPower(1);
                telemetry.addLine("Paper airplane launched");
            }else{
            //    droneShooter.setContinuousServoPower(0);
                //droneShooter.setContinuousServoPower(1);
                telemetry.addLine("Paper airplane launched");
            }else{
                //droneShooter.setContinuousServoPower(0);
            }
//          //hangingServo toggle
            if (gamepad1.dpad_right) {
                hangingServoL.setPosition(0);
                    hangingServoR.setPosition(0);
                telemetry.addLine("Servo in position");
            } else {
                hangingServoL.setPosition(0.2);
                hangingServoR.setPosition(0.2);
                telemetry.addLine("servo restarted");
            }
            //start button is for turning on the hanging motor
//            if (currentGamepad1.start && !previousGamepad1.start) {
//                timer.reset();
//                telemetry.addLine("Press dpad_up to cancel/reverse hanging");
//                while (timer.milliseconds() < 5000) {
//                    hangingMotor.setPower(0.7);
//                }
//                robotIsHanging = true;
//                hangingMotor.setPower(0);
//                //Dpad up for unhanging
//            }
//            if (gamepad1.dpad_up && robotIsHanging) {
//                timer.reset();
//                //motor will spin in the opposite direction until it reaches the end ground
//                while (timer.milliseconds() < 5000) {
//                    hangingMotor.setPower(-0.7);
//                }
//                hangingMotor.setPower(0);
//                telemetry.addLine("hanging reversed");
//
//            }

                    telemetry.update();
                //}
            }


        }
    private void moveRobot(){
        mecanumSubsystem.motorProcessTeleOp();
    }
}