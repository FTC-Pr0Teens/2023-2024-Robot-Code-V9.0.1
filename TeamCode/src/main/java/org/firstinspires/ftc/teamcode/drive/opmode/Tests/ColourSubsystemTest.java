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
        colorSensorSubsystem = new ColorSensorSubsystem(hardwareMap);

        waitForStart();

        Executor executor = Executors.newFixedThreadPool(5);
        CompletableFuture.runAsync(this::telemetry, executor);

        while (opModeIsActive()){
            if (colorSensorSubsystem.findColor2().equalsIgnoreCase("White")){
                telemetry.addLine("white");
            }
        }


    }

    public void telemetry(){
        while (opModeIsActive()){
            telemetry.addLine(colorSensorSubsystem.findColor2());
            telemetry.addLine("test");
            telemetry.update();
        }
    }
}
