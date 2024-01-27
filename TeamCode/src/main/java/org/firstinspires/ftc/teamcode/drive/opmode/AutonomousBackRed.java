package org.firstinspires.ftc.teamcode.drive.opmode;

// Android and FTC SDK imports for robot operation and telemetry
import android.os.Build;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.command.MecanumCommand;
import org.firstinspires.ftc.teamcode.command.MultiMotorCommand;
import org.firstinspires.ftc.teamcode.command.OutputCommand;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WebcamSubsystem;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

@Autonomous(name="Autonomous Back Red")
public class AutonomousBackRed extends LinearOpMode {

    //Custom imports
    private MecanumSubsystem mecanumSubsystem;
    private MecanumCommand mecanumCommand;
    private IMUSubsystem imu;
    private OdometrySubsystem odometrySubsystem;
    private GyroOdometry gyroOdometry;
    private IntakeCommand intakeCommand;
    private WebcamSubsystem webcamSubsystem;
    private OutputCommand outputCommand;
    private MultiMotorSubsystem multiMotorSubsystem;
    private MultiMotorCommand multiMotorCommand;
    FtcDashboard dashboard;
    TelemetryPacket packet;
    private ElapsedTime timer;
    //67, -3, 0
    //54, 24, 0
    //57, -22, -0.832
    //38, 80, -1.58
    private int level = -1;
    private String position = "initalized";


    @Override
    public void runOpMode() throws InterruptedException {
        //initializing the subsystems
        imu = new IMUSubsystem(hardwareMap);
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);
        gyroOdometry = new GyroOdometry(odometrySubsystem, imu);
        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem, gyroOdometry, this);
        //Note different for autonomous front red --> kpy
        mecanumCommand.setConstants(0.07, 0.01, 0.0075/2, 0.05, 0.005, 0.0075/2, 2, 0.05, 0.0);
        intakeCommand = new IntakeCommand(hardwareMap);
        outputCommand = new OutputCommand(hardwareMap);
        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);
        webcamSubsystem = new WebcamSubsystem(hardwareMap, WebcamSubsystem.PipelineName.CONTOUR_RED);
        timer = new ElapsedTime();

        //initializing some variables
        boolean middle = false;
        boolean left = false;
        boolean right = false;
        double intakePower = 0;

        //resets the different subsystems to for preparation
        odometrySubsystem.reset();
        imu.resetAngle();

        intakeCommand.raiseIntake();
        outputCommand.closeGate();

        outputCommand.armToIdle();
        outputCommand.tiltToIdle();
        waitForStart();

        Executor executor = Executors.newFixedThreadPool(4);
        CompletableFuture.runAsync(this::updateOdometry, executor);
        CompletableFuture.runAsync(this::updateTelemetry, executor);
        CompletableFuture.runAsync(this::liftProcess, executor);
        CompletableFuture.runAsync(this::updateMovement, executor);




        mecanumCommand.setFinalPosition(true,3,-10,0,0);
//        webcamSubsystem.getXProp();
//        double propPosition = 0; //propPosition - using the prop the identify the place of he robot
//        timer.reset();
//
//        while(opModeInInit()) {
//            propPosition = webcamSubsystem.getXProp();
//        }
//        intakeCommand.raiseIntake();
////        sleep(8000);
////        sleep(8000);
////        sleep(8000);
//        timer.reset();
//            //TODO: tune
//        if (propPosition > 175) {
//            position = "middle";
//            mecanumCommand.moveToGlobalPosition(-116, 0, 0);
//            sleep(1000);
//            intakePower = 0.4;
//        } else if (propPosition <= 175 && propPosition > 0) {
//            position = "left";
//            mecanumCommand.moveToGlobalPosition(-95, 0, 0);
//            sleep(1100);
//            mecanumCommand.moveToGlobalPosition(-95, 0, 0.75);
//            intakePower = 0.5;
//        } else {
//            mecanumCommand.moveToGlobalPosition(-80, 0, 0);
//            sleep(1880);
//            mecanumCommand.moveToGlobalPosition(-80, 0, -0.96);
//            intakePower = 0.62;
//            //move to board
//            //mecanumCommand.moveToGlobalPosition(-400.5, 17.5, 0.2);
//            // changing theta to either more negative or more positive causes the robot to strafe / act weird
//
//        }
        timer.reset();
        while(timer.milliseconds() < 2500) {
            intakeCommand.intakeOut(intakePower);
        }
        intakeCommand.stopIntake();
        stop();
//        //prep for putting a pixel on to the backboard
//        level = 1; //rise the lift to level 1
//        outputCommand.armToBoard(); // arm towards the board
//        outputCommand.tiltToBoard(); //tilt the output to the board
//        timer.reset();
//
//        //move to board functions
//        while(timer.milliseconds() < 3500) {
//            //TODO: tune
//            if (propPosition > 100) {
//                //pos right
//                mecanumCommand.moveToGlobalPosition(46, -78.5, 1.65); //1.65 radians = 94.53804 degrees
//                right = true;
//            } else if (propPosition <= 100 && propPosition > 0) {
//                //pos middle
//                mecanumCommand.moveToGlobalPosition(61, -80, 1.65);
//                middle = true;
//            } else {
//                //pos left
//                mecanumCommand.moveToGlobalPosition(68, -81.5, 1.65);
//                left = true;
//            }
//        }
//        timer.reset();
//        while (timer.milliseconds() < 500){
//            outputCommand.openGate();
//        }
//        //sets every output related components to its idle position in preparation of the driver period
//        outputCommand.closeGate();
//        outputCommand.tiltToIdle();
//        outputCommand.armToIdle();
//        sleep(6000);
//        level = 0;
//
//        //move to board functions
//        while(timer.milliseconds() < 3500) {
//            //TODO: tune
//            if (propPosition > 100) {
//                //pos right
//                mecanumCommand.moveToGlobalPosition(46, -78.5, 1.65); //1.65 radians = 94.53804 degrees
//                right = true;
//            } else if (propPosition <= 100 && propPosition > 0) {
//                //pos middle
//                mecanumCommand.moveToGlobalPosition(61, -80, 1.65);
//                middle = true;
//            } else {
//                //pos left
//                mecanumCommand.moveToGlobalPosition(68, -81.5, 1.65);
//                left = true;
//            }
//        }
//        timer.reset();
//        while (timer.milliseconds() < 500){
//            outputCommand.openGate();
//        }
//        //sets every output related components to its idle position in preparation of the driver period
//        outputCommand.closeGate();
//        outputCommand.tiltToIdle();
//        outputCommand.armToIdle();
//        sleep(6000);
//        level = 0;
//
//        //attempt on getting more pixels(rough values)
//        if(right == true) {
//            mecanumCommand.moveToGlobalPosition(-10, -78.5, 0); //strafe leftward to the middle: 180 degrees? - coordinates not right/measured
//        }else if(middle = true) {
//            mecanumCommand.moveToGlobalPosition(-10, -78.5, 0); //coordinates not right/measured
//        }else if(left = true) {
//            mecanumCommand.moveToGlobalPosition(-10, -78.5, 0); //coordinates not right/measured
//        }
//
//        mecanumCommand.moveToGlobalPosition(-10, 100, 0); //going forward to white pixels
//
//        timer.reset();
//        while (timer.milliseconds() < 1000){
//            intakeCommand.intakeIn(0.3);
//        }
//        intakeCommand.stopIntake();
//        //prep for putting a pixel on to the backboard
//        level = 1; //rise the lift to level 1
//        outputCommand.armToBoard(); // arm towards the board
//        outputCommand.tiltToBoard(); //tilt the output to the board
//        timer.reset();
//
//        mecanumCommand.moveToGlobalPosition(-10, -78.5, 0); //going backward - coordinates not right/measured
//        mecanumCommand.moveToGlobalPosition(46, -78.5, 0); //going leftward to the board - coordinates not right/measured
//
//        timer.reset();
//        while (timer.milliseconds() < 500){
//            outputCommand.openGate();
//        }
//        //sets every output related components to its idle position in preparation of the driver period
//        outputCommand.closeGate();
//        outputCommand.tiltToIdle();
//        outputCommand.armToIdle();
//        sleep(6000);
//        level = 0;
//
//
//        mecanumCommand.moveToGlobalPosition(0, -84, 1.65); //checkpoint
//        timer.reset();
//        while(opModeInInit()) {
//            propPosition = webcamSubsystem.getXProp();
//        }
//        timer.reset();
//        mecanumCommand.moveToGlobalPosition(-95, 0, 0);
//        sleep(500);
//        mecanumCommand.moveToGlobalPosition(-95, 0, -1.1);
//        sleep(1000);
//        //        sleep(8000);
        /*
        if(propPosition > 175){ //if middle
            //middle
            mecanumCommand.moveToGlobalPosition(-125, 0, 0);
            sleep(1000);
        }
        else if(propPosition <= 175 && propPosition > 0){  // if right
            //right
            mecanumCommand.moveToGlobalPosition(-95, 0, 0);
            sleep(500);
            mecanumCommand.moveToGlobalPosition(-95, 0, -1.1);
            sleep(1000);
        }
        else{ //if left
            position = "left";
        double propPosition = 0;
        timer.reset();
        while(opModeInInit()) {
            propPosition = webcamSubsystem.getXProp();
        }
        timer.reset();
        while (timer.time() > 3500) {
            mecanumCommand.moveToGlobalPosition(-95, 0, 0);
            sleep(500);
            mecanumCommand.moveToGlobalPosition(-95, 0, -1.1);
            sleep(1000);
        }
        //        sleep(8000);
        /*
        if(propPosition > 175){ //if middle
            //middle
            mecanumCommand.moveToGlobalPosition(-125, 0, 0);
            sleep(1000);
        }
        else if(propPosition <= 175 && propPosition > 0){  // if right
            //right
            mecanumCommand.moveToGlobalPosition(-95, 0, 0);
            sleep(500);
            mecanumCommand.moveToGlobalPosition(-95, 0, -1.1);
            sleep(1000);
        }
        else{ //if left
            position = "left";
            mecanumCommand.moveToGlobalPosition(-107, 0, 0);
            sleep(500);
            mecanumCommand.moveToGlobalPosition(-107, 0, 0.8);
            sleep(500);
        }
//        mecanumCommand.moveToGlobalPosition(65, -3.5, 0);
        sleep(3000);
         */

//
//            mecanumCommand.moveToGlobalPosition(-107, 0, 0);
//            sleep(500);
//            mecanumCommand.moveToGlobalPosition(-107, 0, 0.8);
//            sleep(500);
        }
//        mecanumCommand.moveToGlobalPosition(65, -3.5, 0);



//        //attempt on getting more pixels(rough values)
//        if(right == true) {
//            mecanumCommand.moveToGlobalPosition(-10, -78.5, 0); //strafe leftward to the middle: 180 degrees? - coordinates not right/measured
//        }else if(middle = true) {
//            mecanumCommand.moveToGlobalPosition(-10, -78.5, 0); //coordinates not right/measured
//        }else if(left = true) {
//            mecanumCommand.moveToGlobalPosition(-10, -78.5, 0); //coordinates not right/measured
//        }
//
//        mecanumCommand.moveToGlobalPosition(-10, 100, 0); //going forward to white pixels
//
//        timer.reset();
//        while (timer.milliseconds() < 1000){
//            intakeCommand.intakeIn(0.3);
//        }
//        intakeCommand.stopIntake();
//        //prep for putting a pixel on to the backboard
//        level = 1; //rise the lift to level 1
//        outputCommand.armToBoard(); // arm towards the board
//        outputCommand.tiltToBoard(); //tilt the output to the board
//        timer.reset();
//
//        mecanumCommand.moveToGlobalPosition(-10, -78.5, 0); //going backward - coordinates not right/measured
//        mecanumCommand.moveToGlobalPosition(46, -78.5, 0); //going leftward to the board - coordinates not right/measured
//
//        timer.reset();
//        while (timer.milliseconds() < 500){
//            outputCommand.openGate();
//        }
//        //sets every output related components to its idle position in preparation of the driver period
//        outputCommand.closeGate();
//        outputCommand.tiltToIdle();
//        outputCommand.armToIdle();
//        sleep(6000);
//        level = 0;
//
//
//        mecanumCommand.moveToGlobalPosition(0, -84, 1.65); //checkpoint


    public void updateOdometry() {
        while (opModeIsActive()) {
            mecanumCommand.pidProcess();
        }
    }

    public void updateMovement(){
        while (opModeIsActive()) {
            mecanumSubsystem.motorProcessTeleOp();
        }
    }

    public void updateTelemetry() {
        while (opModeIsActive()) {
//            packet.put("x", gyroOdometry.x);
//            packet.put("y", gyroOdometry.y);
            telemetry.addData("x", gyroOdometry.x);
            telemetry.addData("y", gyroOdometry.y);
            telemetry.addData("theta", gyroOdometry.theta);
            telemetry.addData("position", position);
            telemetry.addData("global x", mecanumCommand.globalXController.getOutputPositionalValue());
            telemetry.addData("global y", mecanumCommand.globalYController.getOutputPositionalValue());
            telemetry.addData("global theta", mecanumCommand.globalThetaController.getOutputPositionalValue());
            telemetry.addData("xprop", webcamSubsystem.getXProp());
            telemetry.addData("back encoder count", odometrySubsystem.backEncoder());
            telemetry.addData("left encoder count", odometrySubsystem.leftEncoder());
            telemetry.addData("right encoder count", odometrySubsystem.rightEncoder());
//            packet.put("x", gyroOdometry.x);
//            packet.put("y", gyroOdometry.y);
//            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
    public void liftProcess() {
        while(opModeIsActive()) {
            multiMotorCommand.LiftUp(true, level);
        }
    }
}