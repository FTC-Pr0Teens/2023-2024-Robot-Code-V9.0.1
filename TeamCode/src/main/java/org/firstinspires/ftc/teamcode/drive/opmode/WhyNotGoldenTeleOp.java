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

@TeleOp (name = "WhyNotGoldenTeleoOp")
//TODO: I am bored so I made this lol
public class WhyNotGoldenTeleOp extends LinearOpMode {

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

    private TimerList timers = new TimerList();
    private ElapsedTime timer;

    private HashSet<LIFT_STATE> liftState = new HashSet<>();

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

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        // Initialize the Executor for async tasks
        Executor executor = Executors.newFixedThreadPool(4);

        waitForStart();
        //execute the 4 threads
        CompletableFuture.runAsync(this::processLift, executor);
        CompletableFuture.runAsync(this::processDriveMotor, executor);
        CompletableFuture.runAsync(this::processIMU, executor);
        /*
        Why use executors?
        THis will improve the perfomance, because in this multi-thread code, wbere most thread are independent and can be executed concurrently
        The executor enables asynchronous processing by allowing tasks to be executed in a non-blocking manner.
        This is evident in your use of CompletableFuture.runAsync, which executes tasks asynchronously, improving the responsiveness of your application.
        another thing, it looks more advanced lol
         */
        while (opModeIsActive()) {
            processGamepadInput();
            updateTelemetry();
        }
    }



    private void initializeRobot() {
        liftState.clear();
        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);

        imuSubsystem = new IMUSubsystem(hardwareMap);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);
        gyroOdometry = new GyroOdometry(odometrySubsystem, imuSubsystem);

        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem, gyroOdometry, this);
        gridAutoCentering = new GridAutoCentering(mecanumSubsystem, gyroOdometry);

        intakeCommand = new IntakeCommand(hardwareMap);
        outputCommand = new OutputCommand(hardwareMap);

        droneShooter = new DroneShooter(hardwareMap);
//
        //INITIALIZES THE HANGING SERVO
        hangingServoL = hardwareMap.get(Servo.class, Specifications.HANGING_SERVO_L);
        hangingServoL.setDirection(Servo.Direction.REVERSE);

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

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        outputTimer.reset();

        timers.resetTimer("testTimer");
    }

    private void processGamepadInput() {
        handleMovement();
        handleLift();
        handleIntake();
        handleHanging();
        handleDroneLauncher();
        runMovement();
    }


        private void handleMovement () {
        // Mecanum drive control
        mecanumSubsystem.fieldOrientedMove(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                imuSubsystem.getTheta()
        );
    }

        private void handleLift () {
        // Lift control logic
        checkLiftState();
        isPixelDropping();
        manageLiftState();
    }

        private void manageLiftState() {
        if (gamepad1.x && !liftState.contains(LIFT_STATE.LIFT_END)) {
            outputTimer.reset();
            liftState.add(LIFT_STATE.LIFT_END); // Go to level 2
        } else if (gamepad1.y && !liftState.contains(LIFT_STATE.LIFT_MIDDLE)) {
            outputTimer.reset();
            liftState.add(LIFT_STATE.LIFT_MIDDLE); // Go to level 1
        } else if (gamepad1.a) {
            outputTimer.reset();
            liftState.add(LIFT_STATE.LIFT_IDLE); // Go to bottom, reset
        }

        if (gamepad1.right_trigger > 0.5) {
            timers.resetTimer("gate");
            liftState.add(LIFT_STATE.DROP_PIXEL);
        }
    }

        private void handleIntake() {
        // Intake control logic
        if (liftState.isEmpty()) {
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
    }

        private void handleHanging() {
        // Hanging mechanism control
        if (gamepad1.dpad_up) {
            hangingMotor.setPower(1);
        } else if (gamepad1.dpad_down) {
            hangingMotor.setPower(-1);
        }

        if (gamepad1.dpad_right) {
            hangingServoL.setPosition(0.36);
            hangingServoR.setPosition(0.36);
            telemetry.addLine("Preparing to hang");

        }
    }


        private void checkLiftState () {
        if (liftState.contains(LIFT_STATE.LIFT_IDLE)) {
            outputCommand.tiltToIdle(); //bring arm down BEFORE bringing lift down
            outputCommand.armToIdle();
            if (outputTimer.milliseconds() > 900) {
                level = 0;
                if (outputTimer.milliseconds() > 1300) liftState.clear(); //moved here
            }
        } else if (liftState.contains(LIFT_STATE.LIFT_MIDDLE)) {
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
        } else if (liftState.contains(LIFT_STATE.LIFT_END)) {
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

        private void isPixelDropping () {
        if (liftState.contains(LIFT_STATE.DROP_PIXEL)) {
            if (timers.checkTimePassed("gate", 200)) {
                outputCommand.closeGate();
                if (timers.checkTimePassed("gate", 350)) {
                    intakeCommand.intakeRollerIn();
                }
                if (timers.checkTimePassed("gate", 550)) {
                    intakeCommand.intakeRollerStop();
                    liftState.clear();
                }
            } else {
                outputCommand.openGate();
            }
        }
    }

        private void runMovement () {
        // Reset angle headings of the IMU and autocenter as needed
        if (gamepad1.dpad_left || gamepad1.dpad_right) {
            doCentering = false;
            imuSubsystem.resetAngle();
            gridAutoCentering.reset();
            autoCenterAngle = gamepad1.dpad_left ? Math.PI / 2 : -Math.PI / 2;
        }
        //This means that backdrop is to to the RIGHT (meaning you are on RED side)
        if (gamepad1.b) {
            intakeCommand.lowerIntake();
        }
        if (gamepad1.left_stick_button) {
            intakeCommand.raiseIntake();
        }
        if (gamepad1.left_stick_button) {
            gridAutoCentering.offsetTargetAngle(autoCenterAngle);
            doCentering = true;
        } else {
            doCentering = false;
        }

        // Movement control
        mecanumSubsystem.partialMove(true, -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }

    void handleDroneLauncher() {
         if (gamepad1.back) {
        droneShooter.launch();
        telemetry.addLine("Paper airplane launched");
        }
    }

    void updateTelemetry(){
//            telemetry.addData("Left Output Arm Pos:", leftArm.getPosition());
//            telemetry.addData("Right Output Arm Pos:", rightArm.getPosition());
//            telemetry.addData("Left Tilt Arm Pos:", leftTilt.getPosition());
//            telemetry.addData("Right Tilt Arm Pos:", rightTilt.getPosition());
//            telemetry.addData("liftHeight (subsystem)", multiMotorSubsystem.getPosition());
//            telemetry.addData("Motor 1:", multiMotorSubsystem.main.getCurrentPosition());
//            telemetry.addData("Motor 2:", multiMotorSubsystem.aux1.getCurrentPosition());
//            telemetry.addData("currentLevel (command)", multiMotorCommand.getLevel());
//            telemetry.addData("READ THIS: Lift Level (1, 2, or 3", level);
        telemetry.addData("gateTimer:", gateTimer.time());

        telemetry.addData("angle:", gyroOdometry.getAngle());
        telemetry.addLine();
        telemetry.addData("RIGHTX", gamepad1.right_stick_x);
        telemetry.addData("RIGHTY", gamepad1.right_stick_y);
        telemetry.addData("LEFTX", gamepad1.left_stick_x);
        telemetry.addData("LEFTY", gamepad1.left_stick_y);


        telemetry.addLine("MAINTIMER" + timers.getTimerMillis("testTimer"));
        telemetry.update();
    }
        private void processDriveMotor () {
        while (opModeIsActive()) {
            try {
                synchronized (this) {
                    gridAutoCentering.process(doCentering);
                    mecanumSubsystem.motorProcessTeleOp();
                }
            } catch (Exception e) {
                telemetry.addData("Error in processDriveMotor", e.getMessage());
            }
        }
    }


        private void processIMU () {
        while (opModeIsActive()) {
            try {
                synchronized (this) {
                    imuSubsystem.gyroProcess();
                    gyroOdometry.angleProcess();
                }
            } catch (Exception e) {
                telemetry.addData("Error in processIMU", e.getMessage());
            }
        }
    }


        private void processLift () {
        while (opModeIsActive()) {
            try {
                // Safeguarded execution
                synchronized (this) {
                    multiMotorCommand.LiftUp(true, level);
                }
            } catch (Exception e) {
                // Handle any unexpected exceptions
                telemetry.addData("Error in processLift", e.getMessage());
            }
        }
    }

    }


