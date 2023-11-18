package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.PIDCore;
import org.firstinspires.ftc.teamcode.util.Specifications;

public class SingleMotorSubsystem extends Specifications {

    private DcMotorEx motor;
    private PIDCore pidCore;

    public SingleMotorSubsystem(HardwareMap hardwareMap, String motorName){
        this.motor = hardwareMap.get(DcMotorEx.class, motorName);
        pidCore = new PIDCore(0.1, 0.0001, 0);
    }

    public void motorForward(boolean run, int position){
        if (run){
            motor.setTargetPosition(position);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(-1);
        }
    }

    public void motorBack(boolean run, int position){
        if (run){
            motor.setTargetPosition(position);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(0.8);
        }
    }

    public void motorTurn(boolean run, int position){
        if (run){
            motor.setTargetPosition(position);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (motor.getCurrentPosition() > position){
                motor.setPower(-1);
            } else if (motor.getCurrentPosition() < position){
                motor.setPower(1);
            }
        }
    }

    public void motorTurn(boolean run, int position, double power){
        if (run){
            motor.setTargetPosition(position);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (motor.getCurrentPosition() > position){
                motor.setPower(-power);
            } else if (motor.getCurrentPosition() < position){
                motor.setPower(power);
            }
        }
    }

    public void motorTurnPID(boolean run, int position){
        if (run){
            while (Math.abs(motor.getCurrentPosition() - position) > 2){
                motor.setPower(pidCore.outputPositional(position, motor.getCurrentPosition()));
            }
            stop(true);
        }
    }

    public void motorTurnPower(boolean run, double power){
        if (run){
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower(power);
        }
    }

    public void motorTurnVelocity(boolean run, double velocity){
        if (run){
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setVelocity(velocity);
        }
    }

    public void stop(boolean run){
        if (run){
            motor.setPower(0.0);
        }
    }

    public void reset(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean isBusy() {
        return motor.isBusy();
    }

    public int getPosition() {
        return motor.getCurrentPosition();
    }
}
