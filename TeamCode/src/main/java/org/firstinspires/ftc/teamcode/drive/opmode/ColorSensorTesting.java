package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.ColorSensorSubsystem;

@Autonomous(name = "GoldyLightyMadeByGolden")
public class ColorSensorTesting extends OpMode {
    private ColorSensorSubsystem LightSystem;

    @Override
    public void init() {
        LightSystem = new ColorSensorSubsystem(hardwareMap);
    }

    @Override
    public void loop() {
        String detectedColor = LightSystem.setColorPatternBasedOnSensor();
        int getRed = LightSystem.getRed();
        int getGreen = LightSystem.getGreen();
        int getBlue = LightSystem.getBlue();
        telemetry.addData("Detected Color: ", detectedColor);
        telemetry.addData("red",getRed);
        telemetry.addData("green", getGreen);
        telemetry.addData("blue", getBlue);
        telemetry.update();
    }
}