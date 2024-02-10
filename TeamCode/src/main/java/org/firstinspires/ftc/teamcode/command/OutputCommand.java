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
    private IntakeCommand intakeCommand;
    private TimerList timers = new TimerList();
    private MultiMotorCommand multiMotorCommand;


    public OutputCommand(HardwareMap hardwareMap) {
        leftArm = hardwareMap.get(Servo.class, Specifications.LEFT_OUTPUT_ARM);
        rightArm = hardwareMap.get(Servo.class, Specifications.RIGHT_OUTPUT_ARM);
        leftTilt = hardwareMap.get(Servo.class, Specifications.LEFT_OUTPUT_TILT);
        rightTilt = hardwareMap.get(Servo.class, Specifications.RIGHT_OUTPUT_TILT);
        gate = hardwareMap.get(Servo.class, Specifications.PIXEL_GATE);
        intakeCommand = new IntakeCommand(hardwareMap);

        leftArm.setDirection(Servo.Direction.REVERSE);
        rightArm.setDirection(Servo.Direction.FORWARD);

        leftTilt.setDirection(Servo.Direction.FORWARD);
        rightTilt.setDirection(Servo.Direction.REVERSE);

        gate.setDirection(Servo.Direction.FORWARD);

    }

    public void initialize(){
        closeGate();
        armToIdle();
        tiltToIdle();
    }

    public void openGate(){
        gate.setPosition(0.71);
    }
    public void closeGate(){
        gate.setPosition(0.9);
    }
    public void outputWheelOut(){
        intakeCommand.intakeRollerOut();
    }
    public void outputWheelIn(){
        intakeCommand.intakeRollerIn();
    }
    public void outputWheelStop(){
        intakeCommand.intakeRollerStop();
    }
    public void armToPos(double position){
        leftArm.setPosition(position);
        rightArm.setPosition(position);
    }
    public void armToIdle(){
        //TODO: Find value
        leftArm.setPosition(0.875);
        rightArm.setPosition(0.875);
    }

    public void armToBoard(){
        //TODO: Find value
        leftArm.setPosition(0.718);
        rightArm.setPosition(0.718);
    }

    public void tiltToIdle(){
        leftTilt.setPosition(1);
        rightTilt.setPosition(1);
    }
    public void tiltToBoard(){
        leftTilt.setPosition(0.071);
        rightTilt.setPosition(0.071);
    }

    public void tiltToPos(double pos){
        leftTilt.setPosition(pos);
        rightTilt.setPosition(pos);
    }

    public void gateSetPos(double position){
        gate.setPosition(position);
    }



    public double getGatePosition(){
        return gate.getPosition();
    }
    public double getLeftArmPosition(){
        return leftArm.getPosition();
    }
    public double getRightArmPosition(){
        return rightArm.getPosition();
    }
    public double getLeftTiltPosition(){
        return leftTilt.getPosition();
    }
    public double getRightTiltPosition(){
        return rightTilt.getPosition();
    }


}
