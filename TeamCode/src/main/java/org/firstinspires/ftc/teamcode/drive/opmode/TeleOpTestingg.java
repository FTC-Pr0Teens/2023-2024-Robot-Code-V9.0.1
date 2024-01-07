package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.command.MecanumCommand;
import org.firstinspires.ftc.teamcode.command.MultiMotorCommand;
import org.firstinspires.ftc.teamcode.command.OutputCommand;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;
import org.firstinspires.ftc.teamcode.util.Specifications;
import org.firstinspires.ftc.teamcode.util.TimerList;

import java.util.HashSet;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

@TeleOp (name = "TeleOpTesting")
public class TeleOpTestingg extends LinearOpMode {

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

    TimerList timers = new TimerList();

    private HashSet <LIFT_STATE> liftState = new HashSet<>();

    private enum LIFT_STATE {
        LIFT_IDLE,
        LIFT_MIDDLE,
        LIFT_END,
        DROP_PIXEL
    }

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

        leftArm = hardwareMap.get(Servo.class, Specifications.LEFT_OUTPUT_ARM);
        rightArm = hardwareMap.get(Servo.class, Specifications.RIGHT_OUTPUT_ARM);
        leftTilt = hardwareMap.get(Servo.class, Specifications.LEFT_OUTPUT_TILT);
        rightTilt = hardwareMap.get(Servo.class, Specifications.RIGHT_OUTPUT_TILT);

        leftArm.setDirection(Servo.Direction.REVERSE);
        rightArm.setDirection(Servo.Direction.FORWARD);

        leftTilt.setDirection(Servo.Direction.FORWARD);
        rightTilt.setDirection(Servo.Direction.REVERSE);

        Executor executor = Executors.newFixedThreadPool(3);

        armBeingProcessed = false;

        waitForStart();

        CompletableFuture.runAsync(this::processLift, executor);


        outputTimer.reset();

        while(opModeIsActive()) {
            //runMovement();
            /*
            processLift has to continuously run because PID only allows you to set the lift to
            a certain height while the lift must run forever
             */
            //isLiftExtracted();
            processLift();
            checkLiftState();
            isPixelDropping();


            //lift stuff

            if (gamepad1.x && !liftState.contains(LIFT_STATE.LIFT_END)) {
                //level = 0;
                outputTimer.reset();
                liftState.add(LIFT_STATE.LIFT_END); //go to level 2
            } else if (gamepad1.y) {
                //level = 1;
                outputTimer.reset();
                liftState.add(LIFT_STATE.LIFT_MIDDLE); //go to level 1
            } else if (gamepad1.a){
                //level = 2;
                outputTimer.reset();
                liftState.add(LIFT_STATE.LIFT_IDLE); //go to bottom, reset
            }

            if (gamepad1.dpad_down){
                leftArm.setPosition(0.77);
                rightArm.setPosition(0.77);
            } else if (gamepad1.dpad_right){
                leftArm.setPosition(0.85);
                rightArm.setPosition(0.85);
            } else if (gamepad1.dpad_up){
                leftArm.setPosition(0.95);
                rightArm.setPosition(0.95);
            } else if (gamepad1.dpad_left){
                leftArm.setPosition(0.7);
                rightArm.setPosition(0.7);
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

    private void processLift(){
        multiMotorCommand.LiftUp(true, level);
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
                    leftArm.setPosition(0.718);
                    rightArm.setPosition(0.718);
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

}