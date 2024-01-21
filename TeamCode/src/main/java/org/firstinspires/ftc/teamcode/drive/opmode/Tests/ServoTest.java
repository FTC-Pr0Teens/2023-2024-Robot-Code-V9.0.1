package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Specifications;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

public class ServoTest extends LinearOpMode {
    Servo servo = hardwareMap.get(Servo.class, "intakeServo");
    double position = 0;

    private Servo leftTilt;
    private Servo rightTilt;


    @Override
    public void runOpMode() throws InterruptedException {
        leftTilt = hardwareMap.get(Servo.class, Specifications.LEFT_OUTPUT_TILT);
        rightTilt = hardwareMap.get(Servo.class, Specifications.RIGHT_OUTPUT_TILT);



        leftTilt.setDirection(Servo.Direction.FORWARD);
        rightTilt.setDirection(Servo.Direction.REVERSE);


        Executor executor = Executors.newFixedThreadPool(4);
        CompletableFuture.runAsync(this::updateTelemetry);
        waitForStart();
        while(opModeIsActive()){

        }
    }


    public void updateTelemetry(){
        telemetry.addData("leftTilt:", leftTilt.getPosition());
        telemetry.addData("rightTilt:", rightTilt.getPosition());
        telemetry.update();
    }

}
