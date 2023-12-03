package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class ColorSensorSubsystem {

    private RevBlinkinLedDriver Light;
    private RevColorSensorV3 colorSensor1;
    private RevColorSensorV3 colorSensor2;
    public ColorSensorSubsystem(HardwareMap hardwareMap) {
        Light = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        colorSensor1 = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        colorSensor2 = hardwareMap.get(RevColorSensorV3.class, "colorSensor2");
    }

    public void setColor(String color){
        switch (color) {
            case "Lilac Purple":
                setPatternLilacPurple();
                break;
            case "Yellow":
                setPatternYellow();
                break;
            case "Green":
                setPatternGreen();
                break;
            case "Red":
                setPatternRed();
                break;
            case "White":
                setPatternWhite();
                break;
            case "none":
                setPatternNothing();
                break;
        }
    }
    public String setColorPatternBasedOnSensor() {
        int red = colorSensor1.red();
        int green = colorSensor1.green();
        int blue = colorSensor1.blue();

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
        } else if (isBlack(red, green, blue)) {
            setPatternNothing();
            return "none";
        }else {
            return "error";
        }
    }
    
    
    public String findColor1(){
        int red = colorSensor1.red();
        int green = colorSensor1.green();
        int blue = colorSensor1.blue();

        if (isDominant(blue, red, green)) {
            return "Lilac Purple";
        } else if (isYellow(red, green, blue)) {
            return "Yellow";
        } else if (isSignificantlyDominant(green, red, blue)) {
            return "Green";
        } else if (isSignificantlyDominant(red, blue, green)) {
            return "Red";
        } else if (isWhite(red, green, blue)) {
            return "White";
        } else if (isBlack(red, green, blue)) {
            return "none";
        } else {
            return "error";
        }
    }


    public String findColor2(){
        int red = colorSensor2.red();
        int green = colorSensor2.green();
        int blue = colorSensor2.blue();

        if (isDominant(blue, red, green)) {
            return "Lilac Purple";
        } else if (isYellow(red, green, blue)) {
            return "Yellow";
        } else if (isSignificantlyDominant(green, red, blue)) {
            return "Green";
        } else if (isSignificantlyDominant(red, blue, green)) {
            return "Red";
        } else if (isWhite(red, green, blue)) {
            return "White";
        } else if (isBlack(red, green, blue)) {
            return "none";
        } else {
            return "error";
        }
    }
    private boolean isWhite(int r, int g, int b) {
        int threshold = 1700; // This threshold may need adjustment
        return r > threshold && g > threshold && b > threshold;
    }
    private  boolean isBlack(int r, int g, int b) {
        return g < 400 && b < 400 && r < 400;
    }
    private boolean isYellow(int r, int g, int b) {
//        return r > g && g > b && b < 100;
        return g > r && r > b && b < 1000;
    }

    private boolean isDominant(int a, int b, int c) {
        return a > b && a > c;
    }

    private boolean isSignificantlyDominant(int a, int b, int c) {
        double threshold = 1.5;  // Value can be adjusted based on testing
        return a > threshold * b && a > threshold * c;
    }


    // Set pattern methods
    public void setPatternGreen() {
        Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    public void setPatternLilacPurple() {
        Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
    }

    public int getRed1(){ return colorSensor1.red();}
    public int getGreen1(){ return colorSensor1.green();}
    public int getBlue1(){ return colorSensor1.blue();}

    public int getRed2(){ return colorSensor2.red();}
    public int getGreen2(){ return colorSensor2.green();}
    public int getBlue2(){ return colorSensor2.blue();}
    public void setPatternRed() {
        Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
    }

    public void setPatternYellow() {
        Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
    }

    public void setPatternNothing() {
        Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }
    public void setPatternWhite(){
        Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }
}
