package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp
public class lightsTest extends OpMode{
    RevBlinkinLedDriver LED;
    ColorSensor Colour;

    public void init(){
        LED = hardwareMap.get(RevBlinkinLedDriver.class,"LED");
        Colour = hardwareMap.get(ColorSensor.class, "Colour");
        LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    public void loop(){
        String[] colours = {"Red","Red"};
        int number = 0;
        if (number>1){
            number = 1;
        }
        telemetry.addData("Red", Colour.red());
        telemetry.addData("Green", Colour.green());
        telemetry.addData("Blue", Colour.blue());
        telemetry.update();

        if(gamepad1.b){
            colours[number] = "Red";
            number = 0;
        }

        if (Colour.red()>150 && Colour.red()<300 && Colour.green()>250 && Colour.green()<400 && Colour.blue()>50 && Colour.blue()<200){
            colours[number] = "Yellow";
            number += 1;
            telemetry.addLine("yellow");
        } else if (Colour.red()>150 && Colour.red()<350 && Colour.green()>250 && Colour.green()<450 && Colour.blue()>350 && Colour.blue()<550) {
            colours[number] = "Purple";
            number += 1;
            telemetry.addLine("purple");
        } else if(Colour.red()>50 && Colour.red()<250 && Colour.green()>250 && Colour.green()<450 && Colour.blue()>50 && Colour.blue()<250) {
            colours[number] = "Green";
            number += 1;
            telemetry.addLine("green");
        } else if(Colour.red()>400 && Colour.green()>400 && Colour.blue()>400){
            colours[number] = "White";
            number += 1;
            telemetry.addLine("white");
        }

        //first pixel
        if (colours[0].equals("Yellow")){
            LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        if (colours[0].equals("Purple")){
            LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        if (colours[0].equals("Green")){
            LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        if (colours[0].equals("White")){
            LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        if (colours[0].equals("Red")){
            LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        //second pixel
        if (colours[1].equals("Yellow")){
            LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        if (colours[1].equals("Green")){
            LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        if (colours[1].equals("Purple")){
            LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        if (colours[1].equals("White")){
            LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        if (colours[1].equals("Red")){
            LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

    }
}