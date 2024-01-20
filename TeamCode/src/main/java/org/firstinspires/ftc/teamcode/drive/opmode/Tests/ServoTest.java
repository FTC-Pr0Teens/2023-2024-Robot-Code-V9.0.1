package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoTest extends LinearOpMode {
    Servo servo = hardwareMap.get(Servo.class, "intakeServo");
    double position = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while(opModeIsActive()){
            if (gamepad1.left_bumper){
            }
        }
    }
}
