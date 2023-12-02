package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.command.OutputCommand;

@TeleOp(name="Output Test")
public class OutputTest extends LinearOpMode {
    private OutputCommand outputCommand;

    @Override
    public void runOpMode() throws InterruptedException {
        outputCommand = new OutputCommand(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                outputCommand.openGate();
            }
            else if(gamepad1.b){
                outputCommand.closeGate();
            }
            else if(gamepad1.x){
                outputCommand.armToBoard();
            }
            else if(gamepad1.y){
                outputCommand.armToIdle();
            }
            else if(gamepad1.dpad_up){
                outputCommand.tiltToIdle();
            }
            else if(gamepad1.dpad_down){
                outputCommand.tiltToBoard();
            }
            else if(gamepad1.right_trigger > 0.5){
                outputCommand.outputWheelOut();
            }

            telemetry.addData("gate position", outputCommand.getGatePosition());
            telemetry.addData("left arm position", outputCommand.getLeftArmPosition());
            telemetry.addData("right arm position", outputCommand.getRightArmPosition());
            telemetry.addData("left tilt position", outputCommand.getLeftTiltPosition());
            telemetry.addData("right tilt position", outputCommand.getRightTiltPosition());
        }
    }
}
