package org.firstinspires.ftc.teamcode.drive.opmode;

// Android and FTC SDK imports for robot operation and telemetry

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.command.MecanumCommand;
import org.firstinspires.ftc.teamcode.command.MultiMotorCommand;
import org.firstinspires.ftc.teamcode.command.OutputCommand;
import org.firstinspires.ftc.teamcode.subsystems.AprilCamSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WebcamSubsystem;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;
import org.firstinspires.ftc.teamcode.util.Specifications;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

@Autonomous(name="Autonomous Back Blue Rough")
public class AutonomousBackBlueRough extends LinearOpMode {

    //Custom imports
    private MecanumSubsystem mecanumSubsystem;
    private MecanumCommand mecanumCommand;
    private IMUSubsystem imu;
    private OdometrySubsystem odometrySubsystem;
    private GyroOdometry gyroOdometry;
    private IntakeCommand intakeCommand;
    //private WebcamSubsystem webcamSubsystem;
    private OutputCommand outputCommand;
    private MultiMotorSubsystem multiMotorSubsystem;
    private MultiMotorCommand multiMotorCommand;

    private AprilCamSubsystem aprilCamSubsystem;
    private Servo hangingServoL;
    private Servo hangingServoR;
    FtcDashboard dashboard;
    TelemetryPacket packet;
    private ElapsedTime timer;
    //67, -3, 0
    //54, 24, 0
    //57, -22, -0.832
    //38, 80, -1.58
    private int level = -1;
    private String position = "initalized";
    private String progress = "initalization";
    private int finalX = -125;
    private int finalY = 6;
    private int finalTheta = 0;

    private double targetX = 0;
    private double targetY = 0;

    private Integer aprilID = 2;

    private boolean goToAprilTag = false;
    private String autoColor = "blue";


    @Override
    public void runOpMode() throws InterruptedException {
        //initializing the subsystems
        imu = new IMUSubsystem(hardwareMap);
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);
        gyroOdometry = new GyroOdometry(odometrySubsystem, imu);
        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem, gyroOdometry, this);
        hangingServoL = hardwareMap.get(Servo.class, Specifications.HANGING_SERVO_L);
        hangingServoL.setDirection(Servo.Direction.REVERSE);
        hangingServoR = hardwareMap.get(Servo.class, Specifications.HANGING_SERVO_R);
        //Note different for autonomous front red --> kpy
        //TODO: constants are set higher than usual, approximately started at 0.07 for kpx, kpy and integral terms were 0.007
        //kp overshoots, causing a lot of back and forth movement, thus making y reach less of its actual position
        //most recent pid constants are for x axis
        //if undershooting, increase integral or all of thje constantst

        //TODO: increase kp to adjust faster, but this also increases oscillations so increase kd a little
        //derivative value is at 0.01
        mecanumCommand.setConstants(0.07, 0.01, 0.0075/2, 0.059, 0.0005, 0.0075/2, 2.1, 0.0, 0.0);
        intakeCommand = new IntakeCommand(hardwareMap);
        outputCommand = new OutputCommand(hardwareMap);
        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        //webcamSubsystem = new WebcamSubsystem(hardwareMap, WebcamSubsystem.PipelineName.CONTOUR_BLUE);
        timer = new ElapsedTime();
        aprilCamSubsystem = new AprilCamSubsystem(hardwareMap);


        //initializing some variables

        //resets the different subsystems to for preparation
        odometrySubsystem.reset();
        imu.resetAngle();

        hangingServoL.setPosition(0.6);
        hangingServoR.setPosition(0.6);

        intakeCommand.raiseIntake();
        outputCommand.closeGate();

        outputCommand.armToIdle();
        outputCommand.tiltToIdle();
        waitForStart();

        Executor executor = Executors.newFixedThreadPool(4);
        CompletableFuture.runAsync(this::updateOdometry, executor);
        CompletableFuture.runAsync(this::updateTelemetry, executor);
        CompletableFuture.runAsync(this::liftProcess, executor);
        CompletableFuture.runAsync(this::tagDetectionProcess);
       // CompletableFuture.runAsync(this::ThreadStop);
        //setPropPosition();



        //TODO: positive x goes towards red side when turning 90 counter clockwise
        //TODO: positive y goes away from the board

        //TODO: when turning clockwise it is the opposite of the text above me
        //TODO: below is left
        //TODO: below is left
        telemetry.addData("test", gyroOdometry.x);
        goToLeftSpike();
        //goToBoardLeft();


        goToAprilTag = true;
        sleep(1000);

        while(goToAprilTag && !isStopRequested()) {
            if(aprilCamSubsystem.getHashmap().containsKey(aprilID)){
                mecanumCommand.setFinalPosition(true, 30, getTargetX(0.0), getTargetY(-20.0), getTargetTheta());
            }
            while(!mecanumCommand.isPositionReached(true, true)){}
        }



        //go to correct spike
//        if (position.equals("left")){
//            goToLeftSpike();
//        }
//        else if (position.equals("middle")){
//            goToMiddleSpike();
//        }
//        else if (position.equals("right")){
//            goToRightSpike();
//        }

//        if (position.equals("left")){
//            goToBoardLeft();
//        }
//        else if (position.equals("middle")){
//            goToBoardMiddle();
//        }
//        else if (position.equals("right")){
//            goToBoardRight();
//        }

        sleep(1000);
        stop();




/*
        //prep for putting a pixel on to the backboard
        level = 5; //rise the lift to level 1
        outputCommand.armToBoard(); // arm towards the board
        outputCommand.tiltToBoard(); //tilt the output to the board
        level = 1;


        timer.reset();

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

*/

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

    }

    public void updateOdometry() {
        while (opModeIsActive()) {
            gyroOdometry.odometryProcess();
        }
    }



    private void propRight(){
        //pos is good
        moveToPos(-98,-37,0,2.5,7,1.5);
        sleep(1000);
    }

    private void propMiddle(){
        //pos is good
        moveToPos(-124,0,0,2.5,2.5,1.5);
    }

    private void propLeft(){

//        moveToPos(0,0,-  1.6,2.5,2.5,0.5);
//        sleep(3000);
//        moveToPos(-10,0,-1.6,2.5,7,0.5);
//        sleep(1000);
//        moveToPos(-10,-20,0,2.5,2.5,0.5);

        //TODO: after turning 90 degrees counterclockwise, positive y goes towards to the board, decreasing x goes towards left side wall







//        moveToPos(-20,0,0,2.5,2.5,0.05);
//        sleep(1000);
//        moveToPos(-20,0,1.6,2.5,2.5,0.05);
//        sleep(1000);
//        moveToPos(-75,0,-1.6,2.5,2.5,0.05);


    }

    public void updateTelemetry() {
        while (opModeIsActive()) {
            packet.put("x", gyroOdometry.x);
            packet.put("y", gyroOdometry.y);
            packet.put("theta", gyroOdometry.theta);
            packet.put("x pos", mecanumCommand.globalXController.getError());
            packet.put("x integral", mecanumCommand.globalXController.getIntegralSum());
            packet.put("x derivative", mecanumCommand.globalXController.getDerivative());
            packet.put("y pos", mecanumCommand.globalYController.getError());
            packet.put("y integral", mecanumCommand.globalYController.getIntegralSum());
            packet.put("y derivative", mecanumCommand.globalYController.getDerivative());
            packet.put("Theta pos", mecanumCommand.globalThetaController.getError());
            packet.put("Theta integral", mecanumCommand.globalThetaController.getIntegralSum());
            packet.put("Theta derivative", mecanumCommand.globalThetaController.getDerivative());
            packet.put("final x", finalX);
            packet.put("final y", finalY);
            packet.put("final theta", finalTheta);
            telemetry.addData("x", gyroOdometry.x);
            telemetry.addData("y", gyroOdometry.y);
            telemetry.addData("theta",  gyroOdometry.theta);
//            telemetry.addData("position", position);
//            telemetry.addData("global x", mecanumCommand.globalXController.getOutputPositionalValue());
//            telemetry.addData("global y", mecanumCommand.globalYController.getOutputPositionalValue());
//            telemetry.addData("global theta", mecanumCommand.globalThetaController.getOutputPositionalValue());
            //telemetry.addData("xprop", webcamSubsystem.getXProp());
            telemetry.addData("back encoder count", odometrySubsystem.backEncoder());
            telemetry.addData("left encoder count", odometrySubsystem.leftEncoder());
            telemetry.addData("right encoder count", odometrySubsystem.rightEncoder());
            telemetry.addData("progress", progress);
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
    public void liftProcess() {
        while(opModeIsActive()) {
            multiMotorCommand.LiftUp(true, level);
        }
    }

    public void ThreadStop(){
        while (opModeIsActive()){
            isStopRequested();
        }
    }

    public void moveToPos(double x, double y, double theta, double toleranceX, double toleranceY, double toleranceTheta){
        mecanumCommand.moveIntegralReset();
        // stop moving if within 5 ticks or 0.2 radians from the position
        while ((Math.abs(x - gyroOdometry.x) > toleranceX  //if within 2.5 ticks of target X position
                || Math.abs(y - gyroOdometry.y) > toleranceY //if within 2.5 ticks of target y position
                || Math.abs(theta - gyroOdometry.theta) > toleranceTheta)
                && this.opModeIsActive() && !this.isStopRequested()) {
            mecanumCommand.moveToGlobalPos(x, y, theta);
        }
        mecanumSubsystem.stop(true);
    }

    private void setPropPosition(){
        double propPosition = 0;
        timer.reset();
        while(opModeInInit()) {
            //propPosition = webcamSubsystem.getXProp();
        }
        timer.reset();

        if (propPosition < 100 && propPosition > 0) {
            position = "left";
        } else if (propPosition > 100) {
            position = "middle";
            sleep(1000);
        } else {
            position = "right";
        }
    }

    private void goToRightSpike(){
        //pos is good
        moveToPos(-102,-22,0,5,5,1.5);
    }

    private void goToMiddleSpike(){
        //pos is good
        moveToPos(-125,0,0,5,5,1.5);
    }

    private void goToLeftSpike(){
        //
        moveToPos(-80,-5,0,2.5,2.5,0.05); //-70, -20
        progress = "1";
        sleep(500);
        moveToPos(-80,-5,Math.PI/2,3,3,0.05);
        progress = "2";
        sleep(500);

        timer.reset();
        progress = "intake start";
        intakeCommand.lowerIntake();
        sleep(1500);
        while (timer.milliseconds() < 1500) {
            intakeCommand.intakeOut(0.7);
        }
        intakeCommand.stopIntake();
        progress = "intake stop";
        sleep (100);
        progress = "checkpoint 1 start";
        moveToPos(-150,-3,Math.PI/2,5,5,0.05);
        progress = "checkpoint1 end";
        sleep(100);
        progress = "checkpoint 2 start";
        moveToPos(-150,150,Math.PI/2,2.5,2.5,0.05);
        progress = "checkpoint 2 end";
        sleep(100);
        progress = "checkpoint 3 start";
        moveToPos(-150,150,-Math.PI/2,2.5,2.5,0.05);
        progress = "checkpoint 3 end";
        sleep(1000);
        moveToPos(-90,150,-Math.PI/2,2.5,2.5,0.05);
        sleep(1000);
        moveToPos(-90,170,-Math.PI/2,2.5,2.5,0.05);
        progress = "boardLeft starting";


    }

    private void goToBoardRight(){
        moveToPos(46, -78.5, 1.65, 5, 5, 0.2); //1.65 radians = 94.53804 degrees
    }

    private void goToBoardMiddle(){
        moveToPos(61, -80,1.65, 5,5,0.2);
    }

    private void goToBoardLeft(){

    }


    /*
    public void stopIfPosReached(double targetX, double targetY, double targetTheta){
        while ((Math.abs(x - gyroOdometry.x) > 2.5   //if within 2.5 ticks of target X position
                || Math.abs(y - gyroOdometry.y) > 2.5 //if within 2.5 ticks of target y position
                || Math.abs(theta - gyroOdometry.theta) > 0.15)){

        }
    }

     */

    public void tagDetectionProcess(){
        while(opModeIsActive()) {
            aprilCamSubsystem.runDetections();
        }
    }

    public Double getTargetX(Double offset){
        if(autoColor == "red"){
            return(gyroOdometry.x - aprilCamSubsystem.getAprilXDistance(aprilID, offset));
        }
        else if (autoColor == "blue"){
            return(gyroOdometry.x + aprilCamSubsystem.getAprilXDistance(aprilID, offset));
        }
        return(gyroOdometry.x); //this line won't be called unless autoColor is set to something else
    }

    public Double getTargetY(Double offset){
        if(autoColor == "red"){
            return(gyroOdometry.y - aprilCamSubsystem.getAprilYDistance(aprilID, offset));
        }
        else if (autoColor == "blue"){
            return(gyroOdometry.y + aprilCamSubsystem.getAprilYDistance(aprilID, offset));
        }
        return(gyroOdometry.y); //this line won't be called unless autoColor is set to something else
    }

    public Double getTargetTheta(){
        if(autoColor == "red"){
            return(-Math.PI/2);
        }
        else if (autoColor == "blue"){
            return(Math.PI/2);
        }
        return(0.0); //this line won't be called unless autoColor is set to something else
    }

}