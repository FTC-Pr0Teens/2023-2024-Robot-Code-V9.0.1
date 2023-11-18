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
            position = gamepad1.left_stick_x;
            if(position > 1){
                position = 1;
            }
            if(position < -1){
                position = -1;
            }
            servo.setPosition(position);
            sleep(1000);
        }
    }
}
