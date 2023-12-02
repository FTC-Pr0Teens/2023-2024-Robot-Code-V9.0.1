package org.firstinspires.ftc.teamcode.command;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.SingleMotorSubsystem;
import org.firstinspires.ftc.teamcode.util.Specifications;
import org.firstinspires.ftc.teamcode.util.TimerList;

import java.util.Timer;

public class OutputCommand {

    //Assuming lift is at the front of robot and intake at back
    private Servo leftArm;
    private Servo rightArm;
    private Servo leftTilt;
    private Servo rightTilt;
    private Servo gate;
    private CRServo outputWheel;

    private TimerList timers = new TimerList();


    public OutputCommand(HardwareMap hardwareMap) {
        leftArm = hardwareMap.get(Servo.class, Specifications.LEFT_OUTPUT_ARM);
        rightArm = hardwareMap.get(Servo.class, Specifications.RIGHT_OUTPUT_ARM);
    }

    public void dropPixel(Gamepad gamepad){
        //i might actualy put this back in teleop ngl

        // check block code for specific timings. It is 200 ms for the gate, 200 ms for the wheel.
    }


}
