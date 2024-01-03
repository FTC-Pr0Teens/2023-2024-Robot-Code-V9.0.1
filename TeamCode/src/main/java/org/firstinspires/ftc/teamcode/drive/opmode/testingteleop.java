package org.firstinspires.ftc.teamcode.drive.opmode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DroneShooter;
import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.command.MultiMotorCommand;
import org.firstinspires.ftc.teamcode.command.OutputCommand;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;
import org.firstinspires.ftc.teamcode.util.TimerList;

@TeleOp(name = "Pr0TeensTeleOpV1")
public class testingteleop extends LinearOpMode {
    private MecanumSubsystem mecanumSubsystem;
    private IMUSubsystem imuSubsystem;
    private MultiMotorSubsystem multiMotorSubsystem;
    private MultiMotorCommand multiMotorCommand;
    //private DroneShooter droneShooter;
    private Servo hangingServo;
    private DcMotor hangingMotor;
    private ElapsedTime timer;
    private TimerList timers = new TimerList();

    private IntakeCommand intakeCommand;

    private enum RUNNING_STATE { //mini "threads" to run (is actually run in main thread, just controlled simultaneously)
        LOWER_LIFT, RAISE_LIFT
    }

    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() throws InterruptedException {
        // droneShooter = new DroneShooter(hardwareMap);
        imuSubsystem = new IMUSubsystem(hardwareMap);
        mecanumSubsystem = new MecanumSubsystem(hardwareMap); //MAKE SURE THIS IS INSTANTIATED BEFORE ODOMETRY SUBSYSTEM
        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);
        intakeCommand = new IntakeCommand(hardwareMap);
        intakeCommand.lowerIntake();

        //INITIALIZES THE HANGING SERVO
//        Servo hangingServo = hardwareMap.get(Servo.class, "hangingServo");
//        hangingServo.setPosition(0.75);

        //INITIALIZES THE HANGING MOTOR BC I DON'T WANT TO USE A SUBSYSTEM FOR IT
//        DcMotor hangingMotor = hardwareMap.dcMotor.get("hangingMotor");
//        hangingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        hangingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        Gamepad currentGamepad1 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        while (opModeIsActive()) {
            boolean hangingTrue = false;
            boolean lastHangingState = false;
            //timers.resetTimer("GameTimeElapsed");
            previousGamepad1.copy(currentGamepad1);
            mecanumSubsystem.fieldOrientedMove(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, imuSubsystem.getTheta());
            //x is for taking pixels in, b is for spitting pixels out
            if (gamepad1.x) {
                intakeCommand.intakeIn(0.6);
                telemetry.addLine("Intake ran");
            } else if (gamepad1.b) {
                intakeCommand.intakeOut(0.6);
                telemetry.addLine("intake reversed");
            } else {
                intakeCommand.stopIntake();
            }
            if (gamepad1.back) {
//                droneShooter.setContinuousServoPower(0.7);
//                telemetry.addLine("Paper airplane launched");
//            }
//            if (gamepad1.right_trigger > 0.5) {
//                //left_bumper is used to toggle between hanging and not hanging
//                hangingTrue = !hangingTrue;
//                telemetry.addLine("Preparing to hang");
//            }
//            if (hangingTrue) {
//                lastHangingState = !lastHangingState;
//                hangingServo.setPosition(1);
//                telemetry.addLine("Servo in position");
//                if (hangingTrue && gamepad1.start) {
//                    timer.reset();
//                    while (timer.milliseconds() < 3000) {
//                        hangingMotor.setPower(0.7);
//                    }
//                    hangingMotor.setPower(0);
//                }
                //if (timers.checkTimePassed("hangingServoTime", 500)) {
////                    hangingServo.setPower(0);
////                    hangingMotor.setPower(0.7);
////                    timers.resetTimer("hangingMotorTime");
////                    if(timers.checkTimePassed("hangingMotorTime", 2000)){
////                        //once x amount of miliseconds is reached, the things should like stop yknow idk
////                        hangingMotor.setPower(0);
////                    }
//            } else {
//                hangingServo.setPosition(0.75);
//                hangingMotor.setPower(-0.7);
//                telemetry.addLine("Hanging stopped");
//            }
                    //press  dpad up to cancel hanging just in case. Could probabluy just do dpad up for reversing tbh

                    //to reverse the hanging
                    //slides code(dpleft up for lvl 1, down for lvl 2, and right for lvl 3)
                    if (gamepad1.dpad_left) {
                        multiMotorCommand.LiftUp(true, 1);
                        telemetry.addLine("Level 1 ran");
                    } else if (gamepad1.dpad_down) {
                        multiMotorCommand.LiftUp(true, 2);
                        telemetry.addLine("Level 2 ran");
                    } else if (gamepad1.dpad_right) {
                        multiMotorCommand.LiftUp(true, 3);
                        telemetry.addLine("Level 3 ran");
                    } else {
                        multiMotorCommand.LiftUp(true, 0);
                        telemetry.addLine("No input detected for slides");
                    }
                    telemetry.update();
                //}
            }
        }
    }
}