package org.firstinspires.ftc.teamcode.threadopmode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Specifications;

public class NormalServoSubsystem extends Specifications {

    public enum ServoType{
        ROTATION,
        LINKAGE,
        CLAW,
        TILT
    }

    private Servo servo;
    private ServoType servoType;
    double position;
    private double kp;
    private double ki;
    private double kd;
    private double error = 0;
    private double derivative = 0;
    private double integral = 0;
    private double lastError = 0;
    private double adjustment = 0;
    private double lowerBound;
    private double upperBound;
    private double PIDPosition;
    private ElapsedTime timer;

    public NormalServoSubsystem(HardwareMap hardwareMap, ServoType servoType) {
        this.servoType = servoType;
        if (servoType == ServoType.ROTATION){
            this.servo = hardwareMap.get(Servo.class, ROTATION);
            servo.setDirection(Servo.Direction.REVERSE);
        } else if (servoType == ServoType.CLAW){
            this.servo = hardwareMap.get(Servo.class, CLAW);
            servo.setDirection(Servo.Direction.REVERSE);
        } else if (servoType == ServoType.TILT){
            this.servo = hardwareMap.get(Servo.class, TILT);
            servo.setDirection(Servo.Direction.REVERSE);
        } else if (servoType == ServoType.LINKAGE){
            this.servo = hardwareMap.get(Servo.class, LINKAGE);
            servo.setDirection(Servo.Direction.REVERSE);
        }
        servo.scaleRange(0, 1);
        //TODO: do this
        if (servoType == ServoType.LINKAGE){ //Upper bound = inwards, 0.7 is idle
            lowerBound = 0;
            upperBound = 0.9;
        } else if (servoType == ServoType.CLAW){
            lowerBound = 0;
            upperBound = 1;
        } else if (servoType == ServoType.TILT){
            lowerBound = 0;
            upperBound = 1;
        } else {
            lowerBound = 0.38;
            upperBound = 0.61;
        }
        position = servo.getPosition();
        timer = new ElapsedTime();
    }

    public void toPosition(boolean run, double position){
        if (run){
            if (position > upperBound){
                position = upperBound;
            } else if (position < lowerBound){
                position = lowerBound;
            }
            servo.setPosition(position);
            this.position = position;
        }
    }

    public double findIntakeLength(double angle){
        double length = Specifications.v4bLength;
        double intakePoint = Specifications.intakePointLength;
        return Math.pow(length, 2) + Math.pow(intakePoint, 2) - 2*length*intakePoint*Math.cos(Math.PI/2 + angle);
    }

    public void setPosition(boolean run, double position){
        if (run){
            this.position = position;
        }
    }

    public void continuousTurn(boolean run, double power){
        //TODO: tune
        if (run){
            if (servoType == ServoType.ROTATION){
                position += power * 0.00125;
                servo.setPosition(position);
            } else if (servoType == ServoType.LINKAGE){
                position += power * 0.005;
                servo.setPosition(position);
            } else if (servoType == ServoType.TILT){
                position += power * 0.005;
                servo.setPosition(position);
            }
            else {
                position += power * 0.00125;
                servo.setPosition(position);
            }
            if (position < lowerBound) {
                position = lowerBound;
            } else if (position > upperBound) {
                position = upperBound;
            }
        }
    }

    public void pidToPosition(boolean run, double position){
        if (run){
            timer.reset();
            lastError = error;
            error = position - servo.getPosition();
            derivative = (error - lastError) / timer.time();
            integral += error * timer.time();
            adjustment = kp * error + ki * integral + kd * derivative;
            PIDPosition += adjustment;
            servo.setPosition(PIDPosition);
        }
    }

    public double getPosition(){
        return servo.getPosition();
    }

    public double getTargetPosition(){
        return position;
    }

    public double getAngle(){
        if ((servoType == ServoType.ROTATION)){
            return (servo.getPosition() * 300);
        } else {
            return (servo.getPosition() * 180);
        }
    }

    public double getLowerBound() {
        return lowerBound;
    }

    public double getUpperBound() {
        return upperBound;
    }

    public boolean isBusy(){
        return servo.getPosition() != position;
    }
}
