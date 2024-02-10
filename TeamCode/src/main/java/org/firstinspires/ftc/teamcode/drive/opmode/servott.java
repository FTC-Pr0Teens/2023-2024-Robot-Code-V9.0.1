package org.firstinspires.ftc.teamcode.drive.opmode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.command.MecanumCommand;
import org.firstinspires.ftc.teamcode.command.MultiMotorCommand;
import org.firstinspires.ftc.teamcode.command.OutputCommand;
import org.firstinspires.ftc.teamcode.subsystems.DroneShooter;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;
import org.firstinspires.ftc.teamcode.util.Specifications;
import org.firstinspires.ftc.teamcode.util.TimerList;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

@TeleOp(name = "servott")
public class servott extends LinearOpMode {
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
    private OutputCommand outputCommand;

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

        outputCommand = new OutputCommand(hardwareMap);

        mecanumSubsystem = new MecanumSubsystem(hardwareMap); //MAKE SURE THIS IS INSTANTIATED BEFORE ODOMETRY SUBSYSTEM
        gyroOdometry = new GyroOdometry(odometrySubsystem, imuSubsystem);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);
        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem,  gyroOdometry, this);
        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);
        intakeCommand = new IntakeCommand(hardwareMap);

        //INITIALIZES THE HANGING SERVO
//        hangingServoL = hardwareMap.get(Servo.class, Specifications.HANGING_SERVO_L);
//        hangingServoL.setDirection(Servo.Direction.REVERSE);
//        hangingServoL.setPosition(0.35);
//
//        hangingServoR = hardwareMap.get(Servo.class, Specifications.HANGING_SERVO_R);
//        hangingServoR.setPosition(0.35);
////
////        //INITIALIZES THE HANGING MOTOR
        hangingMotor = hardwareMap.dcMotor.get(Specifications.HANGING_MOTOR);
        hangingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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


        double hangingPositionL = 0;
        double hangingPositionR = 0;
        double tiltPosition = 0;
        double dronePosition = 0;

        double intakePosition = 0.5;

        //0,89 close
        //

        double gatePos = 0;
        //0.03 close
        //0.115 open

        while (opModeIsActive()) {
            sleep(50);
//            hangingServoL.setPosition(hangingPositionL);
//            hangingServoR.setPosition(hangingPositionR);
            outputCommand.gateSetPos(gatePos);
            if(gamepad1.dpad_up){

                hangingPositionL += 0.01;
                hangingPositionR += 0.01;

//                intakePosition +=0.01;

            } else if(gamepad1.dpad_down){
                hangingPositionL -= 0.01;
                hangingPositionR -= 0.01;
                //gatePos-=0.01;
            }
            if(gamepad1.dpad_right){
                dronePosition += 0.005;
            } else if(gamepad1.dpad_left){
                dronePosition -= 0.005;
            }

            if(gamepad1.y){
                tiltPosition += 0.005;
            } else if(gamepad1.a){

                tiltPosition -= 0.005;
            }
            if(gamepad1.right_trigger > 0.5){
                intakeCommand.intakeIn(0.7);
            }
            else if(gamepad1.left_trigger > 0.5){
                intakeCommand.intakeIn(1);
            } else {
                intakeCommand.stopIntake();
            }


            telemetry.addData("armL", hangingPositionL);
            telemetry.addData("armR", hangingPositionR);
            telemetry.addData("drone", dronePosition);
            telemetry.addData("tilt", tiltPosition);
            telemetry.addData("intake", intakePosition);
            telemetry.addData("gate",gatePos);


                    telemetry.update();
                //}
            }

        }
    private void moveRobot(){
        mecanumSubsystem.motorProcessTeleOp();
    }
}