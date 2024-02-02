package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ColorSensorSubsystem;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

@TeleOp (name = "colourSensorTest")
public class ColourSubsystemTest extends LinearOpMode{
    private ColorSensorSubsystem colorSensorSubsystem;
    private String color1 ="";
    private String color2 = "";



    Executor executor = Executors.newFixedThreadPool(5);


    @Override
    public void runOpMode() {
        CompletableFuture.runAsync(this::telemetry, executor);
        colorSensorSubsystem = new ColorSensorSubsystem(hardwareMap);
        colorSensorSubsystem.findColor2();

        ;
    }

    private void telemetry(){
        while (opModeIsActive()){
            telemetry.addLine(colorSensorSubsystem.findColor2());
            telemetry.update();
        }
    }
}
