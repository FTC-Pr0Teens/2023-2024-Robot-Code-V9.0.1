package org.firstinspires.ftc.teamcode.drive.opmode;

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

@Autonomous(name="Autonomous Back Blue")
public class AutonomousBackBlue extends LinearOpMode {
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

    private String position = "initialized";


    @Override
    public void runOpMode() throws InterruptedException {
        //contour location before 60
        imu = new IMUSubsystem(hardwareMap);
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);
        gyroOdometry = new GyroOdometry(odometrySubsystem, imu);
        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem, gyroOdometry, this);
        intakeCommand = new IntakeCommand(hardwareMap);
        outputCommand = new OutputCommand(hardwareMap);
        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);
        webcamSubsystem = new WebcamSubsystem(hardwareMap, WebcamSubsystem.PipelineName.CONTOUR_BLUE);
        timer = new ElapsedTime();

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

        //find the prop position
        //setPropPosition();

        position = "left";

        //go to correct spike
        if (position.equals("left")){
            goToLeftSpike();
        }
        else if (position.equals("middle")){
            goToMiddleSpike();
        }
        else if (position.equals("right")){
            goToRightSpike();
        }

        //output prop
        timer.reset();
        intakeCommand.raiseIntake();
        while(timer.milliseconds() < 1000) {
            intakeCommand.intakeOut(0.5);
        }
        intakeCommand.stopIntake();

        sleep(1000);
        stop();

    }

    public void updateOdometry() {
        while (opModeIsActive()) {
            gyroOdometry.odometryProcess();
        }
    }

    public void updateTelemetry() {
        while (opModeIsActive()) {
            telemetry.addData("x", gyroOdometry.x);
            telemetry.addData("y", gyroOdometry.y);
            telemetry.addData("theta", gyroOdometry.theta);
            telemetry.addData("position", position);
            telemetry.addData("global x", mecanumCommand.globalXController.getOutputPositionalValue());
            telemetry.addData("global y", mecanumCommand.globalYController.getOutputPositionalValue());
            telemetry.addData("global theta", mecanumCommand.globalThetaController.getOutputPositionalValue());
            telemetry.addData("xprop", webcamSubsystem.getXProp());
            telemetry.update();
        }

        while (opModeInInit()){
            telemetry.addData("prop", webcamSubsystem.getXProp());
            telemetry.addData("position", position);
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
            propPosition = webcamSubsystem.getXProp();
        }
        timer.reset();

        if (propPosition < 100 && propPosition > 0) {
            position = "left";
        } else if (propPosition > 100) {
            position = "right";
            sleep(1000);
        } else {
            position = "middle";
        }
    }

    private void goToRightSpike(){
        moveToPos(57, 0, 0, 5, 5, 0.2);
        sleep(1500);
        moveToPos(55, -17, -0.832, 5, 5, 0.2);
    }

    private void goToMiddleSpike(){
        moveToPos(54, 24, 0, 5,5, 0.2);
    }

    private void goToLeftSpike(){
        moveToPos(67,-3,0, 5,5, 0.2);
    }



}
