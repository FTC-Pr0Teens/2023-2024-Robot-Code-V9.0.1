package org.firstinspires.ftc.teamcode.drive.opmode;

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
import org.firstinspires.ftc.teamcode.util.GridAutoCentering;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;
import org.firstinspires.ftc.teamcode.util.Specifications;
import org.firstinspires.ftc.teamcode.util.TimerList;

import java.util.HashSet;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

@TeleOp (name = "Pr0Teens Main Teleop")
public class Pr0TeensMainTeleop extends LinearOpMode {

    private MultiMotorSubsystem multiMotorSubsystem;
    private MultiMotorCommand multiMotorCommand;

    private OdometrySubsystem odometrySubsystem;

    private MecanumSubsystem mecanumSubsystem;
    private MecanumCommand mecanumCommand;
    private GyroOdometry gyroOdometry;
    private IMUSubsystem imuSubsystem;


    ElapsedTime outputTimer = new ElapsedTime();
    ElapsedTime gateTimer = new ElapsedTime();

    private IntakeCommand intakeCommand;

    private OutputCommand outputCommand;

    private boolean armBeingProcessed;

    //true if lift has already reached destined location and ready to return to original pos

    private int level;

    private Servo leftArm;
    private Servo rightArm;
    private Servo leftTilt;
    private Servo rightTilt;
    private Servo gate;

    private DroneShooter droneShooter;

    private Servo hangingServoL;
    private Servo hangingServoR;
    private DcMotor hangingMotor;

    private TimerList timers = new TimerList();
    private ElapsedTime timer;

    private HashSet <LIFT_STATE> liftState = new HashSet<>();

    private GridAutoCentering gridAutoCentering;

    private enum LIFT_STATE {
        LIFT_IDLE,
        LIFT_MIDDLE,
        LIFT_END,
        HANGSERVO, DROP_PIXEL
    }

    private boolean right_stick_pressed = false;
    private boolean left_stick_pressed = false;

    private boolean doCentering = false;
    private double autoCenterAngle = 0;

    private int targetPosition = 0;



    @Override
    public void runOpMode() throws InterruptedException {
        liftState.clear();
        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);

        imuSubsystem = new IMUSubsystem(hardwareMap);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);
        gyroOdometry = new GyroOdometry(odometrySubsystem, imuSubsystem);

        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem,  gyroOdometry, this);
        gridAutoCentering = new GridAutoCentering(mecanumSubsystem, gyroOdometry);

        intakeCommand = new IntakeCommand(hardwareMap);
        outputCommand = new OutputCommand(hardwareMap);

        droneShooter = new DroneShooter(hardwareMap);
//
        //INITIALIZES THE HANGING SERVO
        hangingServoL = hardwareMap.get(Servo.class, Specifications.HANGING_SERVO_L);
        hangingServoL.setDirection(Servo.Direction.REVERSE);
        hangingServoL.setPosition(0.6);

        hangingServoR = hardwareMap.get(Servo.class, Specifications.HANGING_SERVO_R);
        hangingServoR.setPosition(0.6);

        hangingMotor = hardwareMap.dcMotor.get(Specifications.HANGING_MOTOR);
        hangingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangingMotor.setPower(0);

        Executor executor = Executors.newFixedThreadPool(5);

        armBeingProcessed = false;

        CompletableFuture.runAsync(this::processLift, executor);
        level = 0;
        droneShooter.lock();
        outputCommand.tiltToIdle(); //bring arm down BEFORE bringing lift down
        outputCommand.armToIdle();
        multiMotorSubsystem.reset();

        waitForStart();

        CompletableFuture.runAsync(this::processDriveMotor, executor);
        CompletableFuture.runAsync(this::processIMU, executor);
        CompletableFuture.runAsync(this::processDriveController);


        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        outputTimer.reset();

        timers.resetTimer("testTimer");

        multiMotorSubsystem.reset();

        while(opModeIsActive()) {
/*
            processLift has to continuously run because PID only allows you to set the lift to
            a certain height while the lift must run forever
             */
            //isLiftExtracted();
            checkLiftState();
            isPixelDropping();
//            runMovement(); moved to own thread



            //TODO: lift_middle and lift_end is the exact same

            if (gamepad2.x && !liftState.contains(LIFT_STATE.LIFT_END)) {
                //level = 0;
                outputTimer.reset();
                liftState.add(LIFT_STATE.LIFT_END); //go to level 2
            } else if (gamepad2.y && !liftState.contains(LIFT_STATE.LIFT_MIDDLE)) {
                //level = 1;
                outputTimer.reset();
                liftState.add(LIFT_STATE.LIFT_MIDDLE); //go to level 1
            } else if (gamepad2.a){
                //level = 2;
                outputTimer.reset();
                liftState.add(LIFT_STATE.LIFT_IDLE); //go to bottom, reset
            }


            //intake

            if (gamepad1.left_bumper) {
                intakeCommand.intakeIn(0.6);
                intakeCommand.intakeRollerIn();
            } else if (gamepad1.right_bumper) {
                intakeCommand.intakeOut(0.9);
                intakeCommand.intakeRollerOut();
            } else {
                intakeCommand.stopIntake();
            }


            //output gate
            if (gamepad1.right_trigger > 0.5){
                timers.resetTimer("gate");
                liftState.add(LIFT_STATE.DROP_PIXEL);
            }

            //TODO: Set positions for hangingServo
            if (gamepad2.dpad_right) {
                //idle
                hangingServoL.setPosition(0.6);
                hangingServoR.setPosition(0.6); //NOT PREPARED TO HANG

            } else if (gamepad2.dpad_left){
                //hang
                hangingServoL.setPosition(0.525);
                hangingServoR.setPosition(0.525);
            } else  if(gamepad2.dpad_up) {
                //hang
                hangingMotor.setPower(1);
            } else if(gamepad2.dpad_down){
                //hang down
                hangingMotor.setPower(-1);
            } else {
                hangingMotor.setPower(0);
            }


            //drone Launcher
            if (gamepad2.right_trigger > 0.5) {
                droneShooter.launch();
                telemetry.addLine("Paper airplane launched");
            } else if (gamepad2.left_trigger > 0.5){
                hangingServoL.setPosition(0.5635);
                hangingServoR.setPosition(0.5635);
            }

            if (gamepad2.right_bumper) {
                intakeCommand.raiseIntake();
            } else if (gamepad2.left_bumper){
                intakeCommand.lowerIntake();
            }

            if (gamepad1.a) {
                mecanumSubsystem.forward(0.5);
            } else {
                mecanumSubsystem.forward(0);
            }

            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("position", multiMotorSubsystem.getPosition());
            telemetry.addData("power", multiMotorSubsystem.getMainPower());
            telemetry.addData("auxpower", multiMotorSubsystem.getAux1Power());
            telemetry.addData("auxpos", multiMotorSubsystem.getAuxPos());
            telemetry.addData("derivativeValue", multiMotorSubsystem.getDerivativeValue());
            telemetry.addData("cascadeOutput", multiMotorSubsystem.getCascadeOutput());
            telemetry.addData("outputPositional", multiMotorSubsystem.getCascadePositional());
            telemetry.addData("outputVelocity", multiMotorSubsystem.getCascadeVelocity());
            telemetry.addData("level", level);
            telemetry.update();


        }
    }


    private void checkLiftState() {
        if (liftState.contains(LIFT_STATE.LIFT_IDLE)) {
            outputCommand.tiltToIdle(); //bring arm down BEFORE bringing lift down
            outputCommand.armToIdle();
            if (outputTimer.milliseconds() > 900) {
                level = 0;
                if (outputTimer.milliseconds() > 1300) liftState.clear(); //moved here
            }
        } else if (liftState.contains(LIFT_STATE.LIFT_MIDDLE)) {
            if (!liftState.contains(LIFT_STATE.LIFT_END)) {
                targetPosition = -1100;
                if (outputTimer.milliseconds() > 1000) {
                    liftState.clear();
                } else if (outputTimer.milliseconds() > 250) { //bring lift up BEFORE extending arm
                    outputCommand.armToBoard();
                    outputCommand.tiltToBoard();
                } else {
                    level = 1;
                }
            }
        } else if (liftState.contains(LIFT_STATE.LIFT_END)) {
            if (!liftState.contains(LIFT_STATE.LIFT_MIDDLE)) {
                targetPosition = -1400;
                if (outputTimer.milliseconds() > 1000) {
                    liftState.clear();
                } else if (outputTimer.milliseconds() > 250) { //bring lift up BEFORE extending arm
                    outputCommand.armToBoard();
                    outputCommand.tiltToBoard();
                } else {
                    level = 2;
                }
            }
        }
    }

    private void isPixelDropping(){
        if (liftState.contains(LIFT_STATE.DROP_PIXEL)){
            if (timers.checkTimePassed("gate", 500)) {
                outputCommand.closeGate();
                if (timers.checkTimePassed("gate", 550)){
                    intakeCommand.intakeRollerStop();
                    liftState.clear();
                }
            } else {
                outputCommand.openGate();
                intakeCommand.intakeRollerIn();
            }
        }
    }

    private void runMovement(){
        /**
         * These two will reset angle headings of the IMU, both field oriented and autocenter
         */
        //dont enable resetting if in "field is fucked up" state
        //This means that backdrop is to to the LEFT (meaning you are on BLUE side)
        if (gamepad1.dpad_left) {
            doCentering = false;
            imuSubsystem.resetAngle(); //for gyro odometry
            gridAutoCentering.reset(); //reset grid heading
            autoCenterAngle = Math.PI/2; //set autocenter to left 90 degrees
            gridAutoCentering.offsetTargetAngle(autoCenterAngle);

        }

        if (gamepad1.dpad_right) {
            doCentering = false;
            imuSubsystem.resetAngle(); //for gyro odometry
            gridAutoCentering.reset(); //reset grid heading
            autoCenterAngle = Math.PI/2; //set autocenter to right 90 degrees
            gridAutoCentering.offsetTargetAngle(autoCenterAngle);

        }

        if(gamepad1.left_trigger > 0.5) {
            doCentering = true;
        } else doCentering = false;
        
        mecanumSubsystem.partialMove(true, gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }

    private void processDriveMotor(){
        while(opModeIsActive()) {
            gridAutoCentering.process(doCentering);
            mecanumSubsystem.motorProcessTeleOp();
        }
    }

    private void processIMU() {
        while(opModeIsActive()) {
            imuSubsystem.gyroProcess();
            gyroOdometry.angleProcess();
        }
    }

    private void processDriveController(){
        while(opModeIsActive()){
            runMovement();
        }
    }

    private void processLift(){
        while(opModeIsActive()) multiMotorCommand.LiftUp(true, level);
    }


}