package org.firstinspires.ftc.teamcode.drive.opmode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import org.firstinspires.ftc.teamcode.subsystems.WebcamSubsystem;
import org.firstinspires.ftc.teamcode.util.GridAutoCentering;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;
import org.firstinspires.ftc.teamcode.util.TimerList;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

@Autonomous(name="Thompson's Tests")
public class ThompsonsTests extends LinearOpMode {
    private MultiMotorSubsystem multiMotorSubsystem;
    private MultiMotorCommand multiMotorCommand;
    private OdometrySubsystem odometrySubsystem;
    private MecanumSubsystem mecanumSubsystem;
    private MecanumCommand mecanumCommand;
    private IntakeCommand intakeCommand;
    private IMUSubsystem imuSubsystem;
    private GyroOdometry gyroOdometry;
    private GridAutoCentering gridAutoCentering;
    private OutputCommand outputCommand;
    private ColorSensorSubsystem colorSensorSubsystem;
    private WebcamSubsystem webcamSubsystem;
    private ElapsedTime pixelTimer, liftTimer;
    private int level = -1;
    private int pixelCounter;
    private boolean running = true;
    private ElapsedTime timer;

    private String status = "Uninitialized";
    private enum RUNNING_STATE {
        LIFT_STOP,
        RETRACT_LIFT,
        RAISE_LIFT,
        DROP
    }
    private RUNNING_STATE state = RUNNING_STATE.LIFT_STOP;

    private final TimerList timerList = new TimerList();

    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() throws InterruptedException {
        instantiateSubsystems();
        readyRobot();
        pixelCounter = 0;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(webcamSubsystem.webcam, 24);

        waitForStart();

        startThreads();


        for (int i = 30; i > 0; i--) {
            status = "Completed Op - Powering off in " + i + " seconds";
            sleep(1000);
        }

    }

    // Auto Processes
    public void updateTelemetry(){
        while(opModeIsActive()) {
            telemetry.addData("spike location", webcamSubsystem.findSpikePosition());
            telemetry.addData("april tags", webcamSubsystem.getDetections());
            telemetry.update();
        }
    }


    public void liftProcess() {
        while(opModeIsActive()){
            if (running) {
                multiMotorCommand.LiftUpPositional(level);
                if ((level == 0 &&
                        (multiMotorSubsystem.getDerivativeValue() == 0
                                && multiMotorSubsystem.getPosition() < 5))
                        || (multiMotorSubsystem.getDerivativeValue() < 0
                        && multiMotorSubsystem.getPosition() < -5)) {
                    multiMotorSubsystem.reset();
                    running = false;
                }
            }
        }
    }
    public void updateOdometry() {
        while (opModeIsActive()) {
            gyroOdometry.odometryProcess();
        }
    }

    // COmmand/Helper Functions
    private void instantiateSubsystems() {
        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);

        imuSubsystem = new IMUSubsystem(hardwareMap);

        //MAKE SURE THIS IS INSTANTIATED BEFORE ODOMETRY SUBSYSTEM
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);

        odometrySubsystem = new OdometrySubsystem(hardwareMap);

        gyroOdometry = new GyroOdometry(odometrySubsystem,imuSubsystem);

        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem,
                gyroOdometry, this);

        intakeCommand = new IntakeCommand(hardwareMap);
        outputCommand = new OutputCommand(hardwareMap);

        gridAutoCentering = new GridAutoCentering(mecanumSubsystem, gyroOdometry);

        colorSensorSubsystem = new ColorSensorSubsystem(hardwareMap);
        webcamSubsystem = new WebcamSubsystem(hardwareMap, WebcamSubsystem.PipelineName.CONTOUR_RED);

        pixelTimer = new ElapsedTime();
        liftTimer = new ElapsedTime();
        timer = new ElapsedTime();
    }

    private void readyRobot() {
        odometrySubsystem.reset();
        imuSubsystem.resetAngle();

        outputCommand.closeGate();

        outputCommand.armToIdle();
        outputCommand.tiltToIdle();
    }

    private void startThreads() {
        Executor executor = Executors.newFixedThreadPool(4);
        CompletableFuture.runAsync(this::updateOdometry, executor);
        CompletableFuture.runAsync(this::updateTelemetry, executor);
        CompletableFuture.runAsync(this::liftProcess, executor);
    }

    private void dropPixel(int level) {
        // Lift Slider to ready position
        this.level = 5;

        // Wait until slider is high enough before bringing out the arm
        while (!multiMotorSubsystem.isPositionReached());
        outputCommand.armToBoard();
        outputCommand.tiltToBoard();

        // Wait until the arm is far enough before moving the arm to its desired position
        while (outputCommand.getLeftArmPosition() < .7);
        this.level = level;

        // Wait until in position before dropping the pixel
        while (!multiMotorSubsystem.isPositionReached());
        outputCommand.openGate();

        // Wait until the pixel has (probably) fallen out before resetting
        sleep(300);
        outputCommand.closeGate();
        outputCommand.armToIdle();
        outputCommand.tiltToIdle();
        this.level = 5;

        // Wait until the arm has retracted enough before lowering the slider to 0
        while (outputCommand.getLeftArmPosition() > .8);
        this.level = 0;

    }

    private void moveTo(double x, double y, double theta) {
        mecanumCommand.moveIntegralReset();
        // stop moving if within 5 ticks or 0.2 radians from the position
        while ((Math.abs(x - gyroOdometry.x) > .1  //if within 2.5 ticks of target X position
                || Math.abs(y - gyroOdometry.y) > .1 //if within 2.5 ticks of target y position
                || Math.abs(theta - gyroOdometry.theta) > .05)
                && this.opModeIsActive() && !this.isStopRequested()) {
            mecanumCommand.moveToGlobalPos(x, y, theta);
        }
        mecanumSubsystem.stop(true);
    }
    private void moveToCheckpoint(double x, double y, double theta) {
        mecanumCommand.moveIntegralReset();
        // stop moving if within 5 ticks or 0.2 radians from the position
        while ((Math.abs(x - gyroOdometry.x) > 5  //if within 2.5 ticks of target X position
                || Math.abs(y - gyroOdometry.y) > 5 //if within 2.5 ticks of target y position
                || Math.abs(theta - gyroOdometry.theta) > .5)
                && this.opModeIsActive() && !this.isStopRequested()) {
            mecanumCommand.moveToGlobalPos(x, y, theta);
        }
        mecanumSubsystem.stop(true);

    }

    private void releaseIntakePixel() {
        timer.reset();
        intakeCommand.raiseIntake();
        intakeCommand.intakeOut(0.5);
        sleep(500);
        intakeCommand.stopIntake();
    }

    private void intakePixel() {
        intakeCommand.lowerIntake();
        intakeCommand.intakeIn(1);
        intakeCommand.intakeRollerIn();
        sleep(700);
        intakeCommand.raiseIntake();
        intakeCommand.stopIntake();
        intakeCommand.intakeRollerStop();
    }

}
