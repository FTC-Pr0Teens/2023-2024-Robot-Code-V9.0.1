package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.command.OutputCommand;
import org.firstinspires.ftc.teamcode.util.Specifications;

@Config
@TeleOp(name="Output Test")
public class OutputTest extends LinearOpMode {
    private OutputCommand outputCommand;
    private IntakeCommand intakeCommand;


    private Servo leftArm;
    private Servo rightArm;
    private Servo leftTilt;
    private Servo rightTilt;
    private Servo gate;

    public static double position = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        outputCommand = new OutputCommand(hardwareMap);
        intakeCommand = new IntakeCommand(hardwareMap);



        leftArm = hardwareMap.get(Servo.class, Specifications.LEFT_OUTPUT_ARM);
        rightArm = hardwareMap.get(Servo.class, Specifications.RIGHT_OUTPUT_ARM);
        leftTilt = hardwareMap.get(Servo.class, Specifications.LEFT_OUTPUT_TILT);
        rightTilt = hardwareMap.get(Servo.class, Specifications.RIGHT_OUTPUT_TILT);
        gate = hardwareMap.get(Servo.class, Specifications.PIXEL_GATE);
        intakeCommand = new IntakeCommand(hardwareMap);

        leftArm.setDirection(Servo.Direction.REVERSE);
        rightArm.setDirection(Servo.Direction.FORWARD);

        leftTilt.setDirection(Servo.Direction.FORWARD);
        rightTilt.setDirection(Servo.Direction.REVERSE);

        gate.setDirection(Servo.Direction.FORWARD);
//        outputCommand.initialize();
        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                //intakeCommand.raiseIntake();
                leftTilt.setPosition(0.1);
                rightTilt.setPosition(0.1);
            }
            else if(gamepad1.b){
               // intakeCommand.lowerIntake();
                leftTilt.setPosition(0.3);
                rightTilt.setPosition(0.3);
            }
            else if(gamepad1.x){
                //outputCommand.armToBoard();
                leftTilt.setPosition(0.5);
                rightTilt.setPosition(0.5);
            }
            else if(gamepad1.y){
                //outputCommand.armToIdle();
                leftTilt.setPosition(0.9);
                rightTilt.setPosition(0.9);
            }
            else if(gamepad1.right_trigger > 0.5){
                //outputCommand.outputWheelOut();
                leftTilt.setPosition(0.2);
                rightTilt.setPosition(0.2);
            }
            else{
                //outputCommand.outputWheelStop();
            }


            if (gamepad1.left_trigger > 0.5){
                outputCommand.armToBoard();
            } else if (gamepad1.left_bumper){
                outputCommand.armToIdle();
            }


            if (gamepad1.dpad_right){
                leftTilt.setPosition(0.2);
                rightTilt.setPosition(0.2);
            } else if (gamepad1.dpad_left){
                leftTilt.setPosition(0.4);
                rightTilt.setPosition(0.4);
            } else if (gamepad1.dpad_down){
                leftTilt.setPosition(0.6);
                rightTilt.setPosition(0.6);
            } else if (gamepad1.dpad_up){
                leftTilt.setPosition(0.8);
                rightTilt.setPosition(0.8);
            }


            telemetry.addData("gate position", outputCommand.getGatePosition());
            telemetry.addData("left arm position", outputCommand.getLeftArmPosition());
            telemetry.addData("right arm position", outputCommand.getRightArmPosition());
            telemetry.addData("left tilt position", outputCommand.getLeftTiltPosition());
            telemetry.addData("right tilt position", outputCommand.getRightTiltPosition());
            telemetry.update();
        }
    }
}
