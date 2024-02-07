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

    private IntakeCommand intakeCommand;

    private OutputCommand outputCommand;

    private boolean armBeingProcessed;

    //true if lift has already reached destined location and ready to return to original pos

    private int level;

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
        HANGSERVO,
        DROP_PIXEL
    }



    // TODO: leos stuff
    private enum LIFT{
        IDLE,
        RAISE,
        SET,
        LOWER, DROP, RETRACT,
    }

    private LIFT lift = LIFT.IDLE;
    private int setLevel = 1; //what we want
    private volatile int targetLevel = 0; //controls lift, added volatile so that unlike last time it actually changes



    private boolean right_stick_pressed = false;
    private boolean left_stick_pressed = false;

    private boolean doCentering = false;

    private enum alignDirection { //for autocenter
        LEFT,
        RIGHT,
        NONE,
    }
        private alignDirection autoCenterDirection = alignDirection.NONE;




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

        Executor executor = Executors.newFixedThreadPool(3);

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
        CompletableFuture.runAsync(this::manualLift,executor);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        outputTimer.reset();

        timers.resetTimer("testTimer");

        multiMotorSubsystem.reset();

        intakeCommand.lowerIntake();

        while(opModeIsActive()) {

            checkLiftState();
//            isPixelDropping();

            if(gamepad2.a) setLevel = 1;
            if(gamepad2.y) setLevel = 2;

            //TODO: lift_middle and lift_end is the exact same
//
//            if (gamepad2.a && !liftState.contains(LIFT_STATE.LIFT_END)) {
//                //level = 0;
//                outputTimer.reset();
//                liftState.add(LIFT_STATE.LIFT_END); //go to level 2
//            } else if (gamepad2.y && !liftState.contains(LIFT_STATE.LIFT_MIDDLE)) {
//                //level = 1;
//                outputTimer.reset();
//                liftState.add(LIFT_STATE.LIFT_MIDDLE); //go to level 1
//            } else if (gamepad2.b){
//                //level = 2;
//                outputTimer.reset();
//                liftState.add(LIFT_STATE.LIFT_IDLE); //go to bottom, reset
//            }


            //have 3 buttons to change setLevel


            //intake

            if (gamepad1.left_bumper) {
                intakeCommand.lowerIntake();
                intakeCommand.intakeIn(0.6);
                intakeCommand.intakeRollerIn();
            } else if (gamepad1.right_bumper) {
                intakeCommand.intakeOut(0.9);
                intakeCommand.intakeRollerOut();
            } else {
                intakeCommand.stopIntake();
            }

            if(gamepad1.left_trigger > 0.5){ //if autocentering, raise intake (so we dont crash into poles)
                intakeCommand.linkageIdle();
            }


            //output gate
//            if (gamepad1.right_trigger > 0.5){
//                timers.resetTimer("gate");
//                liftState.add(LIFT_STATE.DROP_PIXEL);
//            }

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
                intakeCommand.autoPixel(1); //
            } else if (gamepad2.left_bumper){
                intakeCommand.lowerIntake();
            }

            telemetry.addData("Target level (where the lift is going to)", targetLevel);
            telemetry.addData("Set level (Backdrop level that we want)", setLevel);
//            telemetry.addData("position", multiMotorSubsystem.getPosition());
//            telemetry.addData("power", multiMotorSubsystem.getMainPower());
//            telemetry.addData("auxpower", multiMotorSubsystem.getAux1Power());
//            telemetry.addData("auxpos", multiMotorSubsystem.getAuxPos());
//            telemetry.addData("derivativeValue", multiMotorSubsystem.getDerivativeValue());
//            telemetry.addData("cascadeOutput", multiMotorSubsystem.getCascadeOutput());
//            telemetry.addData("outputPositional", multiMotorSubsystem.getCascadePositional());
//            telemetry.addData("outputVelocity", multiMotorSubsystem.getCascadeVelocity());
//            telemetry.addData("level", level);
//            telemetry.update();


        }
    }


    private void checkLiftState() {
        switch(lift){
            case IDLE:
                outputCommand.tiltToIdle();
                outputCommand.armToIdle();
                level = 0;

                //INPUT COMMAND
                if(gamepad2.x) {
                    timers.resetTimer("lift");
                    lift = LIFT.RAISE;
                    targetLevel = setLevel;
                }

                break;
            case RAISE:
                targetLevel = setLevel;
                outputCommand.closeGate();
                intakeCommand.intakeRollerIn(); //keep pixel inside thing

                if(timers.checkTimePassed("lift", 250)) {
                    outputCommand.armToBoard();
                    outputCommand.tiltToBoard();
                    intakeCommand.intakeOut(0.7);
                }
                //controller inputs to change setLevel

                if(timers.checkTimePassed("lift", 1000)){
                    lift = LIFT.SET;
                    intakeCommand.stopIntake();
                    timers.resetTimer("lift");
                }
                break;
            case SET:
                targetLevel = setLevel;
                if(gamepad1.right_trigger > 0.5){
                    timers.resetTimer("gate");
                    lift = LIFT.DROP;
                }
                //condition to go back to bottom
                if(gamepad1.right_bumper){
                    lift = LIFT.RETRACT;
                    timers.resetTimer("lift");
                }
                break;
            case DROP:
                targetLevel = setLevel; //maintain height

                if(timers.checkTimePassed("gate", 500) && gamepad1.right_trigger < 0.5){ //when 500ms passed and trigger is let go
                    intakeCommand.intakeRollerStop();
                    outputCommand.closeGate();
                    lift = LIFT.SET;
                } else if (timers.checkTimePassed("gate", 200)) {
                    if(gamepad1.right_trigger < 0.5) outputCommand.closeGate();
                    intakeCommand.intakeRollerIn();
                } else {
                    outputCommand.openGate(); //if time < 200ms
                }
                break;
            case RETRACT:
                outputCommand.tiltToIdle();
                outputCommand.armToIdle();
                targetLevel = 0;
                if(timers.checkTimePassed("lift", 900)){
                    timers.resetTimer("lift");
                    telemetry.addLine("lowerLift");
                    lift = LIFT.LOWER;
                }
                break;
            case LOWER:
                telemetry.addLine("hi");
                level = 0;
                if(timers.checkTimePassed("lift", 400)){
                    lift = LIFT.IDLE;
                }
                break;
        }

//
//
//        if (liftState.contains(LIFT_STATE.LIFT_IDLE)) {
//            outputCommand.tiltToIdle(); //bring arm down BEFORE bringing lift down
//            outputCommand.armToIdle();
//            if (outputTimer.milliseconds() > 900) {
//                if (outputTimer.milliseconds() > 1300) liftState.clear(); //moved here
//            }
//        } else if (liftState.contains(LIFT_STATE.LIFT_MIDDLE)) {
//            if (!liftState.contains(LIFT_STATE.LIFT_END)) {
//                targetLevel = -900;
//                if (outputTimer.milliseconds() > 1000) {
//                    liftState.clear();
//                }
//            }
//        } else if (liftState.contains(LIFT_STATE.LIFT_END)) {
//            if (!liftState.contains(LIFT_STATE.LIFT_MIDDLE)) {
//                targetLevel = -2020;
//                if (outputTimer.milliseconds() > 1000) {
//                    liftState.clear();
//                }
//            }
//        }
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
        if (gamepad1.dpad_left) {
            autoCenterDirection = alignDirection.LEFT;
            gridAutoCentering.offsetTargetAngle(-Math.PI/2);
        } else if (gamepad1.dpad_right) {
            autoCenterDirection = alignDirection.RIGHT;
            gridAutoCentering.offsetTargetAngle(Math.PI/2);
        } else if(gamepad1.dpad_up){
            autoCenterDirection = alignDirection.NONE;
            gridAutoCentering.offsetTargetAngle(0);
        } else if(gamepad1.dpad_down){
            if(gamepad1.right_trigger > 0.5){
                switch (autoCenterDirection) { //resets heading if aligned to board, otherwise align to current robot position
                    case LEFT:
                        imuSubsystem.resetAngle(-Math.PI/2);
                        break;
                    case RIGHT:
                        imuSubsystem.resetAngle(Math.PI/2);
                        break;
                    default:
                        imuSubsystem.resetAngle();
                }
            } else imuSubsystem.resetAngle();
            gridAutoCentering.reset();
        }

        doCentering = gamepad1.left_trigger > 0.5 && !gamepad1.dpad_down; //if not resetting angle
        
        if(lift == LIFT.SET || gamepad1.left_trigger > 0.5) mecanumCommand.moveGlobalPartial(true, -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        else mecanumCommand.moveGlobalPartial(true, -gamepad1.left_stick_y * 0.6, gamepad1.left_stick_x * 0.6, gamepad1.right_stick_x);
        //slowmode if dropping pixel and autocentering

    }

    private void processDriveMotor(){
        while(opModeIsActive()) {
            runMovement();
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
//        while(opModeIsActive()) multiMotorCommand.LiftUp(true, level);
        while(opModeIsActive()) multiMotorCommand.LiftUp(true, targetLevel);
    }


    private void manualLift(){
        multiMotorSubsystem.moveLift(-gamepad2.left_stick_y);
    }

}