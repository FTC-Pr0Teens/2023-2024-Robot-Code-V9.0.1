package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.command.MecanumCommand;
import org.firstinspires.ftc.teamcode.command.MultiMotorCommand;
import org.firstinspires.ftc.teamcode.command.OutputCommand;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.util.GridAutoCentering;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;
import org.firstinspires.ftc.teamcode.util.TimerList;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;

@TeleOp(name="Main TeleOp")
public class MainTeleOp extends LinearOpMode{

    private MultiMotorSubsystem multiMotorSubsystem;
    private MultiMotorCommand multiMotorCommand;
    private OdometrySubsystem odometrySubsystem;
    private MecanumSubsystem mecanumSubsystem;
//    private MecanumCommand mecanumCommand;
//    private OutputCommand outputCommand;
    private IntakeCommand intakeCommand;
    private IMUSubsystem imuSubsystem;
    private GyroOdometry gyroOdometry;
    private GridAutoCentering gridAutoCentering;
    private OutputCommand outputCommand;
    private ColorSensorSubsystem colorSensorSubsystem;


    private boolean left_bumper_pressed = false;


    private ElapsedTime timer;

    private TimerList timers = new TimerList();

    private boolean runOutput = false;

    private byte heightLevel = 1;

    private boolean liftRaised = false;
    private MecanumCommand mecanumCommand;

    private enum RUNNING_STATE { //mini "threads" to run (is actually run in main thread, just controlled simultaneously)
        DROP_PIXEL,
        LOWER_LIFT, RAISE_LIFT
    }
    private HashSet<RUNNING_STATE> runningState = new HashSet<>();

    private ArrayList<String> pixelQueue = new ArrayList<>(); // queue of pixels ready to be deposited


    @Override
    public void runOpMode() throws InterruptedException {

        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);

        imuSubsystem = new IMUSubsystem(hardwareMap);

        mecanumSubsystem = new MecanumSubsystem(hardwareMap); //MAKE SURE THIS IS INSTANTIATED BEFORE ODOMETRY SUBSYSTEM

        gyroOdometry = new GyroOdometry(odometrySubsystem, imuSubsystem);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);

        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem,  gyroOdometry, this);

//        outputCommand = new OutputCommand(hardwareMap);
        intakeCommand = new IntakeCommand(hardwareMap);
        outputCommand = new OutputCommand(hardwareMap);

        gridAutoCentering = new GridAutoCentering(mecanumSubsystem, gyroOdometry);

        colorSensorSubsystem = new ColorSensorSubsystem(hardwareMap);

        timer = new ElapsedTime();

        odometrySubsystem.reset();
        imuSubsystem.resetAngle();

        intakeCommand.lowerIntake();
        outputCommand.closeGate();

        outputCommand.armToIdle();
        outputCommand.tiltToIdle();

        //TODO: Initialize

        waitForStart();

        CompletableFuture.runAsync(this::updateOdometry);

        while(opModeIsActive()){

            //Bring down
            runOutput();

            if(gamepad1.right_trigger > 0.5 && liftRaised){
                if(!runningState.contains(RUNNING_STATE.DROP_PIXEL)) { //runs once
                    timers.resetTimer("dropPixel");
                    runningState.add(RUNNING_STATE.DROP_PIXEL);
                }
            }

            if(gamepad1.x) heightLevel = 1;
            if(gamepad1.a) heightLevel = 2;
            if(gamepad2.b) heightLevel = 3;

            if(gamepad1.left_trigger > 0.5){
                if(!runningState.contains(RUNNING_STATE.RAISE_LIFT) && !runningState.contains(RUNNING_STATE.LOWER_LIFT)) { //runs once
                    pixelQueue.clear();

                    if(!colorSensorSubsystem.findColor2().equals("none")){
                        pixelQueue.add(colorSensorSubsystem.findColor1());
                    }
                    if(!colorSensorSubsystem.findColor2().equals("none")){
                        pixelQueue.add(colorSensorSubsystem.findColor2());
                    }

                    timers.resetTimer("raiseLift");
                    runningState.add(RUNNING_STATE.RAISE_LIFT);
                    multiMotorCommand.LiftUp(true, heightLevel);

                    //lock in color sensor

                }
            }

            if(!liftRaised){
                colorSensorSubsystem.setColor(colorSensorSubsystem.findColor1());
            }




            if(liftRaised && gamepad1.left_trigger > 0.5){
                if(!runningState.contains(RUNNING_STATE.LOWER_LIFT)){
                    runningState.add(RUNNING_STATE.LOWER_LIFT);
                    timers.resetTimer("lowerLift");
                    multiMotorCommand.LiftUp(true, 1);
                }
            }

            //

//
//            if(gamepad1.a){
//                timer.reset();
//                if(timer.milliseconds() < 2000) {
//                    outputCommand.armToIdle();
//                    outputCommand.tiltToIdle();
//                }
//                else {
//                    multiMotorCommand.LiftUp(true, 0);
//                }
//            }
            // timer and stuff is wrong.

//            else if(gamepad1.b){
//                multiMotorCommand.LiftUp(true, 3);
//            }
//            else if(gamepad1.y){
//                multiMotorCommand.LiftUp(true, 1);
//                outputCommand.armToBoard();
//                outputCommand.tiltToBoard();
//                if(gamepad1.right_bumper){
//                    timer.reset();
//                    outputCommand.openGate();
//                    while(timer.time(TimeUnit.MILLISECONDS) < 100){}
//                    outputCommand.closeGate();
//                    outputCommand.outputWheelOut();
//                    while(timer.time(TimeUnit.MILLISECONDS) < 100){}
//                }
//            }
//
//            else if(gamepad1.x){
//                multiMotorCommand.LiftUp(true, 2);
//            }
//            else {
//                multiMotorSubsystem.moveLift(0);
//            }

//            mecanumCommand.moveGlobalPartial(true, -gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x * 0.5);
            mecanumSubsystem.fieldOrientedMove(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, imuSubsystem.getTheta());
            if(gamepad1.left_bumper){
                intakeCommand.intakeIn(0.2);

                if(!left_bumper_pressed){
                    left_bumper_pressed = true;
                    if(runningState.contains(RUNNING_STATE.LOWER_LIFT)){
                        multiMotorCommand.LiftUp(true, 0);
                    }
                }

            }
            else if(gamepad1.right_bumper){
                left_bumper_pressed = false;
                intakeCommand.intakeOut(0.6);
            }
            else{
                left_bumper_pressed = false;
                intakeCommand.stopIntake();
            }

            if(gamepad1.dpad_up){
                odometrySubsystem.reset();
                imuSubsystem.resetAngle();
            }
//
//            if(gamepad1.x){
//                gridAutoCentering.offsetTargetAngle(Math.PI/2);
//                gridAutoCentering.secondaryProcess(true);
//            }
//            else if(gamepad1.b){
//                gridAutoCentering.offsetTargetAngle(-Math.PI/2);
//                gridAutoCentering.secondaryProcess(true);
//            }
//            else if(gamepad1.y){
//                gridAutoCentering.offsetTargetAngle(0);
//                gridAutoCentering.secondaryProcess(true);
//            }
//            else if(gamepad1.a){
//                gridAutoCentering.offsetTargetAngle(Math.PI);
//                gridAutoCentering.secondaryProcess(true);
//            } else{
//                gridAutoCentering.process(false);
//            }

            telemetry.addData("liftHeight", multiMotorSubsystem.getPosition());
            telemetry.addData("currentLevel", multiMotorCommand.getLevel());
            telemetry.addData("setLevel", 1);
            telemetry.addData("hashset", runningState);
            telemetry.addData("pixels", pixelQueue);

            telemetry.addData("red1", colorSensorSubsystem.getRed1());
            telemetry.addData("green1", colorSensorSubsystem.getGreen1());
            telemetry.addData("blue1", colorSensorSubsystem.getBlue1());
            telemetry.addData("red2", colorSensorSubsystem.getRed2());
            telemetry.addData("green2", colorSensorSubsystem.getGreen2());
            telemetry.addData("blue2", colorSensorSubsystem.getBlue2());
            telemetry.addData("detected1", colorSensorSubsystem.findColor1());
            telemetry.addData("detected2", colorSensorSubsystem.findColor2());
            telemetry.addData("liftRaised", liftRaised);
            telemetry.update();
        }
    }

    private void updateOdometry(){
        while(opModeIsActive()){
            gyroOdometry.odometryProcess();
//            gridAutoCentering.secondaryProcess(true);
        }
    }

    private void runOutput() {
        //for every running state

        if (runningState.contains(RUNNING_STATE.DROP_PIXEL)) { //if dropping pixel time
            double timerTime = timers.getTimerMillis("dropPixel");
            if(pixelQueue.size() > 1) {
                colorSensorSubsystem.setColor(pixelQueue.get(1));
            } else {
                colorSensorSubsystem.setColor("none");
            }

            if (timerTime > 400) { //yes, order matters. Higher first
                outputCommand.outputWheelStop();

                //now that its done is done, remove dropPixel as a running process
                runningState.remove(RUNNING_STATE.DROP_PIXEL);
                pixelQueue.remove(0);

            } else if (timers.checkTimePassed("dropPixel", 200)) {
                outputCommand.closeGate();
                outputCommand.outputWheelIn();
            } else {
                outputCommand.openGate();
            }
        }

        if (runningState.contains(RUNNING_STATE.LOWER_LIFT)) {
            if (timers.checkTimePassed("lowerLift", 2500)) {
                if (timers.checkTimePassed("lowerLift", 2500)) {
                    liftRaised = false;
                    runningState.remove(RUNNING_STATE.LOWER_LIFT); //if at bottom, cancel lowerlift
                }
            } else {
                outputCommand.armToIdle();
                outputCommand.tiltToIdle();
            }
        }

        if (runningState.contains(RUNNING_STATE.RAISE_LIFT)) {

            outputCommand.armToBoard();
            outputCommand.tiltToBoard();

            if (timers.checkTimePassed("raiseLift",2000)) {
                runningState.remove(RUNNING_STATE.RAISE_LIFT);
                liftRaised = true;
            }
        }


//        if (gamepad1.left_trigger > 0.5) { //if running
//            multiMotorCommand.LiftUp(true, 1);
//            if (!runOutput) {
//                outputCommand.armToBoard();
//                outputCommand.tiltToBoard();
//                runOutput = true;
//                timers.resetTimer("output");
//            }
//        } else {
//            if (runOutput) {
//                if (multiMotorCommand.getLevel() >= 1 && timers.checkTimerExist("output") && timers.checkTimePassed("output", 1000)) {
//                    multiMotorCommand.LiftUp(true, 0);
//                }
//            } //if output is running and lift is released midway
//        }
//
//        if (runOutput) { //if to run

//
//            if(timers.checkTimePassed("output", 500))
//
//                timer.reset();
//                outputCommand.openGate();
//                while(timer.time(TimeUnit.MILLISECONDS) < 100){}
//                outputCommand.closeGate();
//                outputCommand.outputWheelOut();
//                while(timer.time(TimeUnit.MILLISECONDS) < 100){}
//            }
    }
}