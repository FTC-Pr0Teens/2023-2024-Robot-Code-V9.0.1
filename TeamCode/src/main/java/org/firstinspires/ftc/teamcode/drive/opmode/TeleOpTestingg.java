package org.firstinspires.ftc.teamcode.drive.opmode;

import androidx.loader.content.Loader;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
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
import org.firstinspires.ftc.teamcode.util.Specifications;
import org.firstinspires.ftc.teamcode.util.TimerList;

import java.util.HashSet;

@TeleOp (name = "TeleOpTesting")
public class TeleOpTestingg extends LinearOpMode {

    private MultiMotorSubsystem multiMotorSubsystem;
    private MultiMotorCommand multiMotorCommand;

    private OdometrySubsystem odometrySubsystem;

    private MecanumSubsystem mecanumSubsystem;
    private MecanumCommand mecanumCommand;
    private GyroOdometry gyroOdometry;
    private IMUSubsystem imuSubsystem;


    ElapsedTime timerrr = new ElapsedTime();

    //private IntakeCommand intakeCommand;

    private OutputCommand outputCommand;

    private double rotation;

    //true if lift has already reached destined location and ready to return to original pos
    private boolean liftDone;
    private int level;

    private Servo leftArm;
    private Servo rightArm;
    private Servo leftTilt;
    private Servo rightTilt;

    private HashSet <LIFT_STATE> liftState = new HashSet<>();

    private enum LIFT_STATE {
        LIFT_IDLE,
        LIFT_MIDDLE,
        LIFT_END

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

        //intakeCommand = new IntakeCommand(hardwareMap);

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

        waitForStart();

        timerrr.reset();

        while(opModeIsActive()) {
            telemetry.addData("liftHeight (subsystem)", multiMotorSubsystem.getPosition());
            telemetry.addData("Motor 1:", multiMotorSubsystem.main.getCurrentPosition());
            telemetry.addData("Motor 2:", multiMotorSubsystem.aux1.getCurrentPosition());
            telemetry.addData("currentLevel (command)", multiMotorCommand.getLevel());
            telemetry.addData("READ THIS: Lift Level (1, 2, or 3", level);

            telemetry.update();
            //runMovement();
            /*
            processLift has to continuously run because PID only allows you to set the lift to
            a certain height while the lift must run forever
             */
            //isLiftExtracted();
            processLift();
            checkLiftState();


            //lift stuff

            if (gamepad1.x) {
                //level = 0;
                timerrr.reset();
                liftState.add(LIFT_STATE.LIFT_END);
            } else if (gamepad1.y) {
                //level = 1;
                timerrr.reset();
                liftState.add(LIFT_STATE.LIFT_MIDDLE);
            } else if (gamepad1.a){
                //level = 2;
                timerrr.reset();
                liftState.add(LIFT_STATE.LIFT_IDLE);
            }



            //another set of code here (for testing)
            //multiMotorSubsystem.moveLift(-gamepad1.right_stick_y);



            //dropper stuff

            /*
            if (gamepad1.right_trigger > 0.5){
                outputCommand.openGate();
            } else if (gamepad1.b){
                outputCommand.tiltToBoard();
                outputCommand.armToBoard();
            } else if (gamepad1.left_trigger > 0.5){
                outputCommand.tiltToIdle();
                outputCommand.armToIdle();
            }

             */
            if (gamepad1.right_trigger > 0.5){
                outputCommand.armToBoard();
                outputCommand.tiltToBoard();
            } else if (gamepad1.left_trigger > 0.5){
                outputCommand.armToIdle();
                outputCommand.tiltToIdle();
            }

            if (gamepad1.dpad_right){
               outputCommand.openGate();
            } else if (gamepad1.dpad_down){
               outputCommand.closeGate();
            } else if (gamepad1.dpad_up){
                leftArm.setPosition(0.12);
                rightArm.setPosition(0.12);
            } else if (gamepad1.dpad_left){
                leftArm.setPosition(0.13);
                rightArm.setPosition(0.13);
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

            telemetry.update();

/*
            switch (level){
                case 0:
                    //go back to arm idle position (lift retracts so intake can intake pixel for outtake)
                    outputCommand.tiltToIdle();
                    outputCommand.armToIdle();
                    break;
                case 1:
                case 2:
                    //case 1 and 2 are lift levels, procedure for arm and tilt will be the same
                    outputCommand.tiltToBoard();
                    outputCommand.armToBoard();
                    break;

            }

 */

        }

    }

    private void runMovement(){
        //rotation = gamepad1.right_stick_x;
        //mecanumCommand.moveGlobalPartial(true, gamepad1.left_stick_x, -gamepad1.left_stick_y, rotation);
        mecanumSubsystem.fieldOrientedMove(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, imuSubsystem.getTheta());
    }

    private void processLift(){
        multiMotorCommand.LiftUp(true, level);
       }

    private void checkLiftState(){

        if (liftState.contains(LIFT_STATE.LIFT_IDLE)){
            level = 0;
            if (timerrr.milliseconds() > 250) {
                outputCommand.tiltToIdle();
                outputCommand.armToIdle();
            }


            liftState.clear();
        } else if (liftState.contains(LIFT_STATE.LIFT_MIDDLE)){
            level = 1;
           if (!liftState.contains(LIFT_STATE.LIFT_END)){
               if(timerrr.milliseconds() > 250){
                   outputCommand.tiltToIdle();
                   outputCommand.armToIdle();
               }
            }
            liftState.clear();
        } else if (liftState.contains(LIFT_STATE.LIFT_END)){
            level = 2;
            if(!liftState.contains(LIFT_STATE.LIFT_MIDDLE)){
                if(timerrr.milliseconds() > 250){
                    outputCommand.tiltToIdle();
                    outputCommand.armToIdle();
                }
            }
            liftState.clear();
        }


    }


}
