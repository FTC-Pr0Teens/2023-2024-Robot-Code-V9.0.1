package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class ColorSensorSubsystem {

    private RevBlinkinLedDriver Light;
    private ColorSensor colorSensor;

    public ColorSensorSubsystem(HardwareMap hardwareMap) {
        Light = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
    }

    public String setColorPatternBasedOnSensor() {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        if (isDominant(blue, red, green)) {
            setPatternLilacPurple();
            return "Lilac Purple";
        } else if (isYellow(red, green, blue)) {
            setPatternYellow();
            return "Yellow";
        } else if (isSignificantlyDominant(green, red, blue)) {
            setPatternGreen();
            return "Green";
        } else if (isSignificantlyDominant(red, blue, green)) {
            setPatternRed();
            return "Red";
        } else if (isWhite(red, green, blue)) {
            setPatternWhite();
            return "White";
        } else {
            setPatternNothing();
            return "Unknown";
        }
    }

    private boolean isWhite(int r, int g, int b) {
        int threshold = 15; // This threshold may need adjustment
        return r > threshold && g > threshold && b > threshold;
    }

    private boolean isYellow(int r, int g, int b) {
        return r > g && g > b && b < 100; // You might need to adjust the value 100 based on your observations
    }

    private boolean isDominant(int a, int b, int c) {
        return a > b && a > c;
    }

    private boolean isSignificantlyDominant(int a, int b, int c) {
        double threshold = 1.5;  // This value can be adjusted based on testing
        return a > threshold * b && a > threshold * c;
    }


    // Set pattern methods
    public void setPatternGreen() {
        Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    public void setPatternLilacPurple() {
        Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
    }

    public void setPatternRed() {
        Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
    }
    public int getRed(){ return colorSensor.red();}
    public int getGreen(){ return colorSensor.green();}
    public int getBlue(){ return colorSensor.blue();}
    public void setPatternYellow() {
        Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
    }

    public void setPatternNothing() {
        Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);
    }
    public void setPatternWhite(){
        Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }
}
