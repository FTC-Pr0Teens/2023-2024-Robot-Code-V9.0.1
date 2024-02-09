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
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorSubsystem;

import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WebcamSubsystem;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;
import org.firstinspires.ftc.teamcode.util.Specifications;

import java.util.HashSet;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

@Autonomous(name="Autonomous Back Red")
public class AutonomousBackRed extends LinearOpMode {

    //Custom imports
    private MecanumSubsystem mecanumSubsystem;
    private MecanumCommand mecanumCommand;
    private IMUSubsystem imu;
    private OdometrySubsystem odometrySubsystem;
    private GyroOdometry gyroOdometry;
    private IntakeCommand intakeCommand;
//    private WebcamSubsystem webcamSubsystem;
    private OutputCommand outputCommand;
    private MultiMotorSubsystem multiMotorSubsystem;
    private MultiMotorCommand multiMotorCommand;
    private Servo hangingServoL;
    private Servo hangingServoR;
    FtcDashboard dashboard;
    TelemetryPacket packet;
    private ElapsedTime timer;

    AprilCamSubsystem aprilCamSubsystem;

    private ColorSensorSubsystem colorSensor;

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
    double propPosition = 0;


    ElapsedTime outputTimer;

    private HashSet<LIFT_STATE> liftState = new HashSet<>();


    private enum LIFT_STATE {
        LIFT_IDLE,
        LIFT_MIDDLE,
        LIFT_END,
        HANGSERVO, DROP_PIXEL
    }




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

        aprilCamSubsystem = new AprilCamSubsystem(hardwareMap);
        colorSensor = new ColorSensorSubsystem(hardwareMap);
        //Note different for autonomous front red --> kpy
        //TODO: constants are set higher than usual, approximately started at 0.07 for kpx, kpy and integral terms were 0.007
        //kp overshoots, causing a lot of back and forth movement, thus making y reach less of its actual position
        //most recent pid constants are for x axis
        //if undershooting, increase integral or all of thje constantst

        //TODO: increase kp to adjust faster, but this also increases oscillations so increase kd a little
        //derivative value is at 0.01
        mecanumCommand.setConstants(0.10, 0.03, 0.0095/2, 0.059, 0.004, 0.0095/2, 2.15, 0.0, 0.001);
        intakeCommand = new IntakeCommand(hardwareMap);
        outputCommand = new OutputCommand(hardwareMap);
        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
//        webcamSubsystem = new WebcamSubsystem(hardwareMap, WebcamSubsystem.PipelineName.CONTOUR_RED);
        timer = new ElapsedTime();
        outputTimer = new ElapsedTime();

        //initializing some variables

        //resets the different subsystems to for preparation
        odometrySubsystem.reset();
        imu.resetAngle();


        intakeCommand.raiseIntake();
        outputCommand.closeGate();

        outputCommand.armToIdle();
        outputCommand.tiltToIdle();
//
//        timer.reset();
//        while (opModeInInit()) {
//            propPosition = webcamSubsystem.getXProp();
//        }double propPosition = 0;
////
////        timer.reset();
////        while (opModeInInit()) {
////            propPosition = webcamSubsystem.getXProp();
////        }
        waitForStart();

        Executor executor = Executors.newFixedThreadPool(5);
        CompletableFuture.runAsync(this::updateOdometry, executor);
        CompletableFuture.runAsync(this::updateTelemetry, executor);
        CompletableFuture.runAsync(this::liftProcess, executor);
//        CompletableFuture.runAsync(this::checkLiftState,executor);


        //TODO: positive x goes towards red side when turning 90 counter clockwise
        //TODO: positive y goes away from the board

        //TODO: when turning clockwise it is the opposite of the text above me
        //TODO: below is left

        goToRightSpike();

//        if (propPosition > 475) {
//            goToMiddleSpike();
//        } else if (propPosition > 0 && propPosition < 475) {
//            goToLeftSpike();
//        } else {
//            goToRightSpike();
//        }




    }

    public void updateOdometry() {
        while (opModeIsActive()) {
            gyroOdometry.odometryProcess();
        }
    }

    public void updateTelemetry() {
        while (opModeIsActive()) {
            packet = new TelemetryPacket(true);
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
            packet.put("lift pos", multiMotorSubsystem.aux1);
            double robotWidth = 16.5/2;
            double robotLength = 18/2;
            double posX = -odometrySubsystem.y* 0.3937;
            double posY = -odometrySubsystem.x* 0.3937;
            double posTheta = Math.PI + odometrySubsystem.theta;
            double offsetX = (-0.5) + (-2 * 24) + (robotWidth); //1 * 24 inch tile down + (0.5) inch mat side tab distance + 9 inch robot radius
            double offsetY = -(3*24) + (robotLength);
            packet.fieldOverlay() //in inches
                    .setTranslation(offsetX, offsetY)
                    //3 * 24inch tiles left - 9 inches robot radius
                    //1 * 24 inch tile down + (0.5) inch mat side tab distance + 9 inch robot radius
                    .setFill("blue")
                    .setAlpha(0.4)
                    .fillCircle(posX,posY,9)
                    .setFill("black")
                    .setAlpha(1)
                    .strokeLine(posX,posY, posX + (9*Math.sin(posTheta)), posY - (9 * Math.cos(posTheta)));
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
            telemetry.addData("liftState:", liftState);
            telemetry.addData("level:", level);
            telemetry.addData("timer", timer.milliseconds());
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

    public void maintainPos(double x, double y, double theta, double toleranceX, double toleranceY, double toleranceTheta){
        mecanumCommand.moveIntegralReset();
        // stop moving if within 5 ticks or 0.2 radians from the position
        if ((Math.abs(x - gyroOdometry.x) > toleranceX  //if within 2.5 ticks of target X position
                || Math.abs(y - gyroOdometry.y) > toleranceY //if within 2.5 ticks of target y position
                || Math.abs(theta - gyroOdometry.theta) > toleranceTheta)
                && this.opModeIsActive() && !this.isStopRequested()) {
            mecanumCommand.moveToGlobalPos(x, y, theta);
        } else {
            mecanumSubsystem.stop(true);
        }
    }

    private void goToRightSpike(){
        //pos is good
        moveToPos(-69.48,-16,2.11,2,2,0.025);
//        sleep(1000);
//        moveToPos(-80,-24,-Math.PI/2,5,5,0.05);
        timer.reset();
        progress = "intake start";
        intakeCommand.autoPixel(1);

        while (timer.milliseconds() < 400) {
            intakeCommand.intakeOutNoRoller(0.3);
        }
        intakeCommand.stopIntake();
        intakeCommand.raiseIntake();
        progress = "intake stop";
        progress = "checkpoint 1 start";
        //129, 43.5, -Math.PI/2
        //129, 60.8, -Math.PI/2
        moveToPos(-124,39,-Math.PI/2,3,3,0.015);
        timer.reset();

        while(opModeIsActive() && timer.milliseconds() < 3000) {
            intakeCommand.autoPixel(1);

            intakeCommand.intakeIn(0.7);
            maintainPos(-118,39, -Math.PI/2,2.5,2.5,0.015);
            intakeCommand.autoPixel(5);
            if (colorSensor.findColor2().equalsIgnoreCase("white")) break;
        }

        timer.reset();
//        ejectPixelLoop

        while(opModeIsActive() && timer.milliseconds() < 1500){
            intakeCommand.autoPixel(1); //above max stack
            intakeCommand.intakeOutNoRoller(1);
            intakeCommand.intakeRollerIn();
            maintainPos(-118,30, -Math.PI/2,2.5,2.5,0.05);
        }

        intakeCommand.stopIntake();

        progress = "checkpoint 2 end";
//        moveToPos(-150, -177,Math.PI/2,7,5,0.05);
//        progress = "checkpoint 3 end";
//        sleep(1000);
        moveToPos(-120,-180,-Math.PI/2,2.5,2.5,0.05);

        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < 4500){
            maintainPos(-20,-232,-Math.PI/2,2.5,2.5,0.05);
            level = 1;
            outputCommand.armToBoard();
            outputCommand.tiltToBoard();
        }

        timer.reset();
        while (timer.milliseconds() < 1000) {
            outputCommand.openGate();
            outputCommand.outputWheelIn();
        }


        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < 2000){
            maintainPos(-27,-220,-Math.PI/2,2.5,2.5,0.05);
            outputCommand.armToIdle();
            outputCommand.tiltToIdle();
        }


        level = 5;
        outputCommand.closeGate();
        level = 5;


//        moveToPos(-125,-118.898,-Math.PI/2,1.5,1.5,0.015);
//        moveToPos(-125,-30,-Math.PI/2,5,5,0.015);
//        moveToPos(-99,37,-Math.PI/2,1.5,1.5,0.015);
//        //other stack: x: -71, y: 37
//        while(opModeIsActive() && timer.milliseconds() < 3000) {
//            intakeCommand.autoPixel(2);
//            intakeCommand.intakeIn(0.7);
//            maintainPos(-99,37, -Math.PI/2,2.5,2.5,0.15);
//            intakeCommand.autoPixel(5);
//            if (colorSensor.findColor2().equalsIgnoreCase("white")) break;
//        }
//
//        timer.reset();
////        ejectPixelLoop
//
//        while(opModeIsActive() && timer.milliseconds() < 1500){
//            intakeCommand.autoPixel(1); //above max stack
//            intakeCommand.intakeOutNoRoller(1);
//            intakeCommand.intakeRollerIn();
//            maintainPos(-99,30, -Math.PI/2,2.5,2.5,0.05);
//        }
//
//        intakeCommand.stopIntake();
//
//        moveToPos(-120,-64,Math.PI/2,3,10,1);
//        moveToPos(-120,-190,Math.PI/2,2.5,2.5,0.05);
//        moveToPos(-62.3,-218.931,-Math.PI/2,2.5,4,0.05);
//        timer.reset();
//        while (opModeIsActive() ) {
//            if (timer.milliseconds() > 2200){
//                outputCommand.armToIdle();
//                outputCommand.tiltToIdle();
//                sleep(500);
//                break;
//            } else if (timer.milliseconds() > 1600) {
//                outputCommand.openGate();
//                outputCommand.outputWheelIn();
//            } else if (timer.milliseconds() > 500) {
//                outputCommand.armToBoard();
//                outputCommand.tiltToBoard();
//            } else {
//                level = 1;
//            }
//        }
//        sleep(700);
//        level = 0;

    }


    private void goToMiddleSpike(){
        //pos is good
        moveToPos(-113,5,0.7854,2.5,2.5,0.025);
        timer.reset();
        progress = "intake start";
        intakeCommand.autoPixel(1);

        while (timer.milliseconds() < 300) {
            intakeCommand.intakeOutNoRoller(0.3);
        }
        intakeCommand.stopIntake();
        intakeCommand.raiseIntake();
        moveToPos(-116,11,0.7854,3,3,0.5);
//        sleep(1000);
//        moveToPos(-80,-24,-Math.PI/2,5,5,0.05);

        //129, 43.5, -Math.PI/2
        //129, 60.8, -Math.PI/2

        moveToPos(-120,39,-Math.PI/2,1.5,1.5,0.015);
        timer.reset();

        while(opModeIsActive() && timer.milliseconds() < 1800) {
            intakeCommand.autoPixel(2);
            intakeCommand.intakeIn(0.7);
            maintainPos(-116,37, -Math.PI/2,1.5,1.5,0.015);
            if (colorSensor.findColor2().equalsIgnoreCase("white")) break;
        }

        timer.reset();
//        ejectPixelLoop

        while(opModeIsActive() && timer.milliseconds() < 1500){
            intakeCommand.autoPixel(1); //above max stack
            intakeCommand.intakeOutNoRoller(1);
            intakeCommand.intakeRollerIn();
            maintainPos(-116,30, -Math.PI/2,1.5,1.5,0.015);
        }

        intakeCommand.stopIntake();

        progress = "checkpoint 2 end";
//        moveToPos(-150, -177,Math.PI/2,7,5,0.05);
//        progress = "checkpoint 3 end";
//        sleep(1000);
        moveToPos(-126, 0, -Math.PI/2, 5, 6, 0.25);
        moveToPos(-119,-160,-Math.PI/2,5,6,0.25);

        //middle: -82,-235

        timer.reset();
        maintainPos(-55,-200,-Math.PI/2,2.5,2.5,0.05);
        while (opModeIsActive() ) {
            double milliseconds = timer.milliseconds();
            //go to board
            if (milliseconds > 3100){
                maintainPos(-55,-200,-Math.PI/2,2.5,2.5,0.015); //moves back from board
                outputCommand.armToIdle();
                outputCommand.tiltToIdle();
                break;
            } else if (milliseconds > 2400) {
                outputCommand.openGate();
                outputCommand.outputWheelIn();
            } else if (milliseconds > 500) {
                outputCommand.armToBoard();
                outputCommand.tiltToBoard();
            } else {
                level = 1;
            }
            if(milliseconds <= 3100) maintainPos(-50,-235,-Math.PI/2,2.5,2.5,0.05); //go to board
        }
        timer.reset();
        while (opModeIsActive()) {
            maintainPos(-55,-220,-Math.PI/2,6,3,0.25);
            if(timer.milliseconds() > 700) break;
        }
        level = 0;
        outputCommand.closeGate();
        stop();
//        moveToPos(-90,-200,-Math.PI/2,6,3,0.25);
//        moveToPos(-90,-235,-Math.PI/2,6,3,0.25);//keep going to the point if not there already, otherwise this won't wor
    }







    private void goToLeftSpike(){
        moveToPos(-95,2,-0.468,2.5,2.5,0.1);
        timer.reset();
        progress = "intake start";
        intakeCommand.autoPixel(1);

        while (timer.milliseconds() < 300) {
            intakeCommand.intakeOutNoRoller(0.3);
        }
        intakeCommand.stopIntake();
        intakeCommand.raiseIntake();
        moveToPos(-118.79,-10,-Math.PI/2,2.5,2.5,0.05);
        moveToPos(-118,30,-Math.PI/2,1.5,1.5,0.05);
        timer.reset();

        while(opModeIsActive() && timer.milliseconds() < 1900) {
            intakeCommand.autoPixel(2);
            intakeCommand.intakeIn(0.7);
            maintainPos(-116,36, -Math.PI/2,1.5,1.5,0.05);
            if (colorSensor.findColor2().equalsIgnoreCase("white")) break;
        }

        timer.reset();
//        ejectPixelLoop

        while(opModeIsActive() && timer.milliseconds() < 1500){
            intakeCommand.autoPixel(1); //above max stack
            intakeCommand.intakeOutNoRoller(1);
            intakeCommand.intakeRollerIn();
            maintainPos(-116,30, -Math.PI/2,1.5,1.5,0.05);
        }

        intakeCommand.stopIntake();

        progress = "checkpoint 2 end";
//        moveToPos(-150, -177,Math.PI/2,7,5,0.05);
//        progress = "checkpoint 3 end";
//        sleep(1000);
        moveToPos(-120, 0, -Math.PI/2, 7, 10, 0.05);
        moveToPos(-110,-160,-Math.PI/2,5,10,0.05);

        //middle: -82,-235

        timer.reset();
//        maintainPos(-36,-200,-Math.PI/2,2.5,2.5,0.05);
        while (opModeIsActive() ) {
            double milliseconds = timer.milliseconds();
            //go to board
            if (milliseconds > 3100){
                maintainPos(-63,-230,-Math.PI/2,7,7,0.05);
                outputCommand.armToIdle();
                outputCommand.tiltToIdle();
                break;
            } else if (milliseconds > 2400) {
                outputCommand.openGate();
                outputCommand.outputWheelIn();
            } else if (milliseconds > 500) {
                outputCommand.armToBoard();
                outputCommand.tiltToBoard();
            } else {
                level = 1;
            }
            if(milliseconds < 3100) maintainPos(-63,-238,-Math.PI/2,2.5,2.5,0.05); //go to board
        }
        level = 0;
        outputCommand.closeGate();
//        moveToPos(-90,-200,-Math.PI/2,6,3,0.05);
//        moveToPos(-90,-235,-Math.PI/2,6,3,0.05);
    }

//    private void checkLiftState() {
//        if (liftState.contains(LIFT_STATE.LIFT_IDLE)) {
//            outputCommand.tiltToIdle(); //bring arm down BEFORE bringing lift down
//            outputCommand.armToIdle();
//            if (outputTimer.milliseconds() > 900) {
//                level = 0;
//                if (outputTimer.milliseconds() > 1300) liftState.clear(); //moved here
//            }
//        } else if (level == 1) {
//                if (outputTimer.milliseconds() > 1000) {
//                    //liftState.clear();
//                } else if (outputTimer.milliseconds() > 250) { //bring lift up BEFORE extending arm
//                    outputCommand.armToBoard();
//                    outputCommand.tiltToBoard();
//                }
//        }
//    }


    /*
    public void stopIfPosReached(double targetX, double targetY, double targetTheta){
        while ((Math.abs(x - gyroOdometry.x) > 2.5   //if within 2.5 ticks of target X position
                || Math.abs(y - gyroOdometry.y) > 2.5 //if within 2.5 ticks of target y position
                || Math.abs(theta - gyroOdometry.theta) > 0.15)){

        }
    }

     */

}