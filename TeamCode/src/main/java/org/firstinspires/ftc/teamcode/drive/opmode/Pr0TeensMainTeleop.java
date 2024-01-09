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
    private boolean liftDone;
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

    TimerList timers = new TimerList();
    private ElapsedTime timer;

    private HashSet <LIFT_STATE> liftState = new HashSet<>();

    private GridAutoCentering gridAutoCentering;

    private enum LIFT_STATE {
        LIFT_IDLE,
        LIFT_MIDDLE,
        LIFT_END,
        DROP_PIXEL
    }

    private boolean right_stick_pressed = false;
    private boolean left_stick_pressed = false;

    private boolean doCentering = false;
    private double autoCenterAngle = 0;
    private TimerList timers = new TimerList();

    private byte boardLeftRight = 0; //LEFT = 2, RIGHT = 1, UNSET = 0


    @Override
    public void runOpMode() throws InterruptedException {
        liftState.clear();
        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);

        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        imuSubsystem = new IMUSubsystem(hardwareMap);
        gyroOdometry = new GyroOdometry(odometrySubsystem, imuSubsystem);
        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem,  gyroOdometry, this);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);

        intakeCommand = new IntakeCommand(hardwareMap);

        outputCommand = new OutputCommand(hardwareMap);
        mecanumSubsystem.reset(); // delete later



        droneShooter = new DroneShooter(hardwareMap);

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

        Executor executor = Executors.newFixedThreadPool(4);

        armBeingProcessed = false;

        waitForStart();

        CompletableFuture.runAsync(this::processLift, executor);
        CompletableFuture.runAsync(this::processDriveMotor, executor);
        CompletableFuture.runAsync(this::processIMU, executor);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        outputTimer.reset();

        while(opModeIsActive()) {
            boolean hangingTrue = false;
            boolean lastHangingState = false;

            /*
            processLift has to continuously run because PID only allows you to set the lift to
            a certain height while the lift must run forever
             */
            //isLiftExtracted();
            checkLiftState();
            isPixelDropping();
            runMovement();

            //lift stuff

            //TODO: lift_middle and lift_end is the exact same

            if (gamepad1.x && !liftState.contains(LIFT_STATE.LIFT_END)) {
                //level = 0;
                outputTimer.reset();
                liftState.add(LIFT_STATE.LIFT_END); //go to level 2
            } else if (gamepad1.y && !liftState.contains(LIFT_STATE.LIFT_MIDDLE)) {
                //level = 1;
                outputTimer.reset();
                liftState.add(LIFT_STATE.LIFT_MIDDLE); //go to level 1
            } else if (gamepad1.a){
                //level = 2;
                outputTimer.reset();
                liftState.add(LIFT_STATE.LIFT_IDLE); //go to bottom, reset
            }

            //another set of code here (for testing)
            //multiMotorSubsystem.moveLift(-gamepad1.right_stick_y);

            if (gamepad1.right_trigger > 0.5){
                timers.resetTimer("gate");
                liftState.add(LIFT_STATE.DROP_PIXEL);
            }
            if (gamepad1.left_trigger > 0.5){
                outputCommand.openGate();
            }
            if(liftState.isEmpty()) {
                if (gamepad1.right_bumper) {
                    intakeCommand.intakeIn(0.7);
                    intakeCommand.intakeRollerIn();
                } else if (gamepad1.left_bumper) {
                    intakeCommand.intakeOut(0.7);
                    intakeCommand.intakeRollerOut();
                } else {
                    intakeCommand.stopIntake();
                }
            }

            //hangingServo toggle
            if (gamepad1.left_trigger > 0.5) {
                //left_bumper is used to toggle between hanging and not hanging
                hangingArmInPlace = !hangingArmInPlace;
                telemetry.addLine("Preparing to hang");
            }
            if (hangingArmInPlace) {
                hangingServoL.setPosition(0.95);
            hangingServoR.setPosition(0.95);
                telemetry.addLine("Servo in position");
            } else {
                hangingServoR.setPosition(0.2);
                telemetry.addLine("servo restarted");
            }
            //start button is for turning on the hanging motor
            if (currentGamepad1.start && !previousGamepad1.start) {
                timer.reset();
                telemetry.addLine("Press dpad_up to cancel/reverse hanging");
                while (timer.milliseconds() < 5000) {
                    hangingMotor.setPower(0.7);
                }
                robotIsHanging = true;
                hangingMotor.setPower(0);
                //Dpad up for unhanging
            }
            if (gamepad1.dpad_up && robotIsHanging) {
                timer.reset();
                //motor will spin in the opposite direction until it reaches the end ground
                while (timer.milliseconds() < 5000) {
                    hangingMotor.setPower(-0.7);
                }
                hangingMotor.setPower(0);
                telemetry.addLine("hanging reversed");
            }

            //Drone Launcher
            if (gamepad1.back) {
                droneShooter.setContinuousServoPower(1);
                telemetry.addLine("Paper airplane launched");
            }else{
                droneShooter.setContinuousServoPower(0);
            }

            mecanumCommand.moveGlobalPartial(true, gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            telemetry.addData("Left Output Arm Pos:", leftArm.getPosition());
            telemetry.addData("Right Output Arm Pos:", rightArm.getPosition());
            telemetry.addData("Left Tilt Arm Pos:", leftTilt.getPosition());
            telemetry.addData("Right Tilt Arm Pos:", rightTilt.getPosition());
            telemetry.addData("liftHeight (subsystem)", multiMotorSubsystem.getPosition());
            telemetry.addData("Motor 1:", multiMotorSubsystem.main.getCurrentPosition());
            telemetry.addData("Motor 2:", multiMotorSubsystem.aux1.getCurrentPosition());
            telemetry.addData("currentLevel (command)", multiMotorCommand.getLevel());
            telemetry.addData("READ THIS: Lift Level (1, 2, or 3", level);
            telemetry.addData("gateTimer:", gateTimer.time());
            telemetry.update();



        }

    }


    private void checkLiftState(){
        if (liftState.contains(LIFT_STATE.LIFT_IDLE)){
            outputCommand.tiltToIdle(); //bring arm down BEFORE bringing lift down
            outputCommand.armToIdle();
            if (outputTimer.milliseconds() > 900) {
                level = 0;
                if(outputTimer.milliseconds() > 1300) liftState.clear(); //moved here
            }
        } else if (liftState.contains(LIFT_STATE.LIFT_MIDDLE)){
            if (!liftState.contains(LIFT_STATE.LIFT_MIDDLE)) {
                if (outputTimer.milliseconds() > 1000) {
                    liftState.clear();
                } else if (outputTimer.milliseconds() > 250) { //bring lift up BEFORE extending arm
                    outputCommand.armToBoard();
                    outputCommand.tiltToBoard();
                } else {
                    level = 1;
                }
            }
        } else if (liftState.contains(LIFT_STATE.LIFT_END)){
            if (!liftState.contains(LIFT_STATE.LIFT_MIDDLE)) {
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
            if (timers.checkTimePassed("gate", 350)) {
                outputCommand.closeGate();
                if(timers.checkTimePassed("gate",500)) {
                    intakeCommand.intakeRollerIn();
                }
                if (timers.checkTimePassed("gate", 750)){
                    intakeCommand.intakeRollerStop();
                    liftState.clear();
                }
            } else {
                outputCommand.openGate();
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
            boardLeftRight = 2; //LEFT
            autoCenterAngle = Math.PI/2; //set autocenter to left 90 degrees
        }

        //This means that backdrop is to to the RIGHT (meaning you are on RED side)
        if (gamepad1.dpad_right) {
            doCentering = false;
            imuSubsystem.resetAngle(); //for gyro odometry
            gridAutoCentering.reset(); //reset grid heading
            boardLeftRight = 1; //RIGHT
            autoCenterAngle = Math.PI/2; //set autocenter to right 90 degrees
        }

        if(gamepad1.left_stick_button) { //TODO: rebind to game1 left trigger
            gridAutoCentering.offsetTargetAngle(autoCenterAngle);
            doCentering = true;
        } else doCentering = false;

        //movement
        mecanumSubsystem.partialMove(true, gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
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

    private void processLift(){
        while(opModeIsActive()) multiMotorCommand.LiftUp(true, level);
    }

}