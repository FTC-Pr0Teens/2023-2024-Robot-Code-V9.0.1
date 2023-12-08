package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.ColorSensorSubsystem;

public class ColourSubsystemTest extends LinearOpMode{
    private ColorSensorSubsystem colorSensorSubsystem;
    private String color1 ="";
    private String color2 = "";

    @Override
    public void runOpMode() {
        colorSensorSubsystem = new ColorSensorSubsystem(hardwareMap);

        colorSensorSubsystem.findColor1();
        colorSensorSubsystem.findColor2();

        ;
    }
}
