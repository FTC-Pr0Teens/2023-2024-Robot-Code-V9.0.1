package org.firstinspires.ftc.teamcode.threadopmode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Specifications;

public class ContinuousServoSubsystem extends Specifications {
    private CRServo servo;
    private double power; //default power

    public enum ServoType {
        INTAKE_LEFT,
        INTAKE_RIGHT
    }

    public ContinuousServoSubsystem(HardwareMap hardwareMap, ServoType servotype){

//        if (tapeMeasure == TapeMeasure.horizontal) {
//            servo = hardwareMap.get(CRServo.class, "horizontal");
//            power = 0.5;
//        } else if (tapeMeasure == TapeMeasure.vertical) {
//            servo = hardwareMap.get(CRServo.class, "vertical");
//            power = 0.5;
        /*} else */
        //determine which servo is being initialized
        if (servotype == ServoType.INTAKE_LEFT) {
            servo = hardwareMap.get(CRServo.class, INTAKE_LEFT);
            servo.setDirection(DcMotorSimple.Direction.FORWARD);
            power = 1;
        } else if(servotype == ServoType.INTAKE_RIGHT){
            servo = hardwareMap.get(CRServo.class, INTAKE_RIGHT);
            servo.setDirection(DcMotorSimple.Direction.FORWARD);
            power = 1;
        }
    }

    public void turnForward(boolean run){
        if(run){
            servo.setPower(power);
        }
    }
    public void turnBackward(boolean run){
        if(run){
            servo.setPower(-power);
        }
    }

    public void stop(boolean run){
        if (run){
            servo.setPower(0);
        }
    }

    public void turnPower(boolean run, double power) {
        if (run){
            servo.setPower(power);
        }
    }

    public void sleep(long ms){
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public boolean isBusy(){
        return servo.getPower() != 0;
    }

    public double getPower(){
        return servo.getPower();
    }
}
