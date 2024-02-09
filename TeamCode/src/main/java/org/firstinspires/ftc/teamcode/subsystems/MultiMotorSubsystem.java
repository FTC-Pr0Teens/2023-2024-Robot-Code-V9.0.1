package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.arcrobotics.ftclib.util.LUT;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.Interval;
import org.firstinspires.ftc.teamcode.util.IntervalControl;
import org.firstinspires.ftc.teamcode.util.PIDCore;
import org.firstinspires.ftc.teamcode.util.Specifications;

public class MultiMotorSubsystem extends Specifications {
    public DcMotorEx main; //main motor (motor with encoder)
    public DcMotor aux1; //first auxiliary motor (motor without encoder)
    private DcMotor aux2; //second auxiliary motor (motor without encoder)
    private ElapsedTime mainTimer;
    private ElapsedTime safetyTimer; //used to make sure the lift doesn't get stuck at the bottom
    private PIDCore pidUp; //the pid used for going upward
    private double kpUp = 0.005; //k: constant
    private double kiUp = 0.04;
    private double kdUp = 0.009;
    private PIDCore pidDown; //the pid used for going downward
    private double kpDown = 0.008;
    private double kiDown = 0.04;
    private double kdDown = 0.009;
    private PIDCore pidVP;
    private double kp = 0.011;
    private double ki = 0;
    private double kd = 0;
    private PIDCore pidVV;
    private double kpv = 0.02;
    private double kiv = 0;
    private double kdv = 0;
    private double kv = 0.3;
    private double power = 0;
    private double angularVelocity = 0;
    private int finalPosition;
    private double tickToAngleConversion = Math.PI * 2 / 753.2; //tick: encoder tick
    public boolean runToPosition = false;
    private int encoderI; //I: initial
    private int encoderF; //F: final
    public boolean test = false;
    private int uncertainty = 3;
    private double m = 0.3;
    private double c = 0.1;
    private double downThreshold = -0.3;
    private volatile double intervalValue = 0;
    private double cascadeOutput = 0;

    public double vel = 0;

    private InterpLUT interpLUT = new InterpLUT();
    private LUT<Integer, Double> lut = new LUT<>();

    private PIDCore cascadePID;
    private double cascadeKp = (double) 1/750;
    private double cascadeKi = 0;
    private double cascadeKd = 0.0;
    private double cascadeKpVel = (double) 1/2000;
    private double cascadeKiVel = 0;
    private double cascadeKdVel = 0;
    private PIDCore testLiftPID = new PIDCore(1, 1, 1);
    public boolean testBoolean = false; //ignore delete later lol

    public enum MultiMotorType {
        dualMotor,
        threeMotor
    }

    public void setVPPIDConstants(double kp, double ki, double kd){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        pidVP.setConstant(kp, kd, ki);
    }

    public void setVVPIDConstants(double kp, double ki, double kd){
        this.kpv = kp;
        this.kiv = ki;
        this.kdv = kd;
        pidVV.setConstant(kpv, kdv, kiv);
    }

    public void setPidConstants(double kpUp, double kiUp, double kdUp, double kpDown, double kiDown, double kdDown){
        this.kpUp = kpUp;
        this.kiUp = kiUp;
        this.kdUp = kdUp;
        this.kpDown = kpDown;
        this.kiDown = kiDown;
        this.kdDown = kdDown;
        pidUp.setConstant(kpUp, kdUp, kiUp);
        pidDown.setConstant(kpDown, kdDown, kiDown);
    }

    public void cascadeSetConstants(double kpp, double kpi, double kpd, double kvp, double kvi, double kvd){
        cascadeKp = kpp;
        cascadeKi = kpi;
        cascadeKd = kpd;
        cascadeKpVel = kvp;
        cascadeKiVel = kvi;
        cascadeKdVel = kvd;
        cascadePID.setCascadeConstant(kpp, kpi, kpd, kvp, kvi, kvd);
    }

    public void setPidConstants(double kpUp, double kiUp, double kdUp, double kpDown, double kiDown, double kdDown, double kv){
        this.kpUp = kpUp;
        this.kiUp = kiUp;
        this.kdUp = kdUp;
        this.kpDown = kpDown;
        this.kiDown = kiDown;
        this.kdDown = kdDown;
        this.kv = kv;
        pidUp.setConstant(kpUp, kdUp, kiUp);
        pidDown.setConstant(kpDown, kdDown, kiDown);
    }

    //for three motor
    public MultiMotorSubsystem(HardwareMap hardwareMap, boolean reset, MultiMotorType type) {
        if(type == MultiMotorType.threeMotor) {
            main = hardwareMap.get(DcMotorEx.class, EXTENSION_MOTOR_MAIN);
            aux1 = hardwareMap.get(DcMotor.class, EXTENSION_MOTOR_AUX1);
            aux2 = hardwareMap.get(DcMotor.class, "");
            main.setDirection(DcMotorSimple.Direction.REVERSE);
            aux1.setDirection(DcMotorSimple.Direction.FORWARD);
            aux2.setDirection(DcMotorSimple.Direction.FORWARD);
            if (reset) {
                main.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            main.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            aux1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            aux2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            main.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            aux1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            aux2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            pidUp = new PIDCore(kpUp, kdUp, kiUp);
            pidDown = new PIDCore(kpDown, kdDown, kiDown);
            pidVP = new PIDCore(kp, kd, ki);
            pidVV = new PIDCore(kpv, kdv, kiv);
            mainTimer = new ElapsedTime();
            safetyTimer = new ElapsedTime();
            safetyTimer.reset();
            encoderI = getPosition();
        }
        else if(type == MultiMotorType.dualMotor){
            main = hardwareMap.get(DcMotorEx.class, EXTENSION_MOTOR_MAIN);
            aux1 = hardwareMap.get(DcMotor.class, EXTENSION_MOTOR_AUX1);



            main.setDirection(DcMotorSimple.Direction.FORWARD); //originally reverse
            aux1.setDirection(DcMotorSimple.Direction.REVERSE);

            if (reset) {
                main.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            //might change main depending on
            main.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            aux1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            main.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            aux1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            pidUp = new PIDCore(kpUp, kdUp, kiUp);
            pidDown = new PIDCore(kpDown, kdDown, kiDown);
            pidVP = new PIDCore(kp, kd, ki);
            pidVV = new PIDCore(kpv, kdv, kiv);
            cascadePID = new PIDCore(cascadeKp, cascadeKd, cascadeKi, cascadeKpVel, cascadeKdVel, cascadeKiVel, 0);

            mainTimer = new ElapsedTime();
            encoderI = getPosition();

            //create lut
            //motor position, power needed to maintain
            lut.add(0, 0.0);

        }
    }

    //two motor methods

    //theres literally no difference between internal and external encoder process
    public void internalEncoderProcessTwoMotorInterPLUT(){
        if(runToPosition) {
            if(finalPosition >= getPosition()){
                //or create lut table for setting pidUP values
                power = pidUp.outputPositional(finalPosition, getPosition()) + lut.getClosest(getPosition());
            }
            else if(Math.abs(getPosition() - finalPosition) < uncertainty){
                power = lut.getClosest(getPosition());
            }
            else{
                //or create lut table for setting pidDown values (more likely for this)
                power = 0.3*pidDown.outputPositional(finalPosition, getPosition()) + lut.getClosest(getPosition());
            }
        }
        else { //if run to position is false
            safetyTimer.reset(); //reset safety timer
        }
        if (power > 1) { //set power = 1 if greater than 1
            power = 1;
        } else if (power < -0.4){ //set power = -0.6 if less than -0.6
            power = -0.4;
        }
        main.setPower(power); //set power
        aux1.setPower(getMainPower());
    }

    public void moveLift(double power){
        main.setPower(power);
        aux1.setPower(getMainPower());
        testLiftPID.outputPositional(3900, getPosition());
    }

    public void externalEncoderProcessTwoMotor(){
        test = true;
        mainTimer.reset(); //reset timer
        encoderF = getPosition(); //get final encoder value
        angularVelocity = (encoderF - encoderI) * tickToAngleConversion / mainTimer.time(); //calculate angular velocity
        encoderI = encoderF; //set initial to target encoder value
        if (runToPosition){ //if run to position is true
            if (finalPosition >= getPosition()){ //if target is higher than current position
                power = pidUp.outputPositional(finalPosition, encoderF) + m/1050 * encoderF + c; //calculate power up
            } else if (Math.abs(getPosition()-finalPosition) < uncertainty){ //if target is less than current and the difference is less than 750
//                if(getPosition() < uncertainty){
//                    power = 0;
//                }
//                else {
                power = m / 1050 * encoderF + c; //maintain position with power
//                }
            } else { //move down, if either no input / target is lower than current position
                power = 0.3*pidDown.outputPositional(finalPosition, encoderF) + m/1050 * encoderF + c; //calculate power down
            }
//            if (finalPosition < uncertainty){ //if final position is bottom
//                if (safetyTimer.time() > 2){
//                    reset(); //reset encoder if never reached bottom in 2 seconds
//                }
//            } else {
//                safetyTimer.reset();
//            }
        } else { //if run to position is false
            safetyTimer.reset(); //reset safety timer
        }
        if (power > 1) { //set power = 1 if greater than 1
            power = 1;
        } else if (power < -0.4){ //set power = -0.6 if less than -0.6
            power = -0.4;
        }
        main.setPower(power); //set power
        aux1.setPower(getMainPower());
    }

    //three motor methods

    //async process (in a thread)
    public void externalEncoderProcess(){
        test = true;
        mainTimer.reset(); //reset timer
        encoderF = getPosition(); //get final encoder value
        angularVelocity = (encoderF - encoderI) * tickToAngleConversion / mainTimer.time(); //calculate angular velocity
        encoderI = encoderF; //set initial to target encoder value
        if (runToPosition){ //if run to position is true
            if (finalPosition >= getPosition()){ //if target is higher than current position
                power = pidUp.outputPositional(finalPosition, encoderF) + m/1050 * encoderF + c; //calculate power up
            } else if (Math.abs(getPosition()-finalPosition) < uncertainty){ //if target is less than current and the difference is less than 750
//                if(getPosition() < uncertainty){
//                    power = 0;
//                }
//                else {
                power = m / 1050 * encoderF + c; //maintain position with power
//                }
            } else { //move down, if either no input / target is lower than current position
                power = 0.3*pidDown.outputPositional(finalPosition, encoderF) + m/1050 * encoderF + c; //calculate power down
            }
//            if (finalPosition < uncertainty){ //if final position is bottom
//                if (safetyTimer.time() > 2){
//                    reset(); //reset encoder if never reached bottom in 2 seconds
//                }
//            } else {
//                safetyTimer.reset();
//            }
        } else { //if run to position is false
            safetyTimer.reset(); //reset safety timer
        }
        if (power > 1) { //set power = 1 if greater than 1
            power = 1;
        } else if (power < -0.4){ //set power = -0.6 if less than -0.6
            power = -0.4;
        }
        main.setPower(power); //set power
        aux1.setPower(getMainPower());
        aux2.setPower(getMainPower());
    }

    public void internalEncoderProcess(){
        test = true;
        mainTimer.reset(); //reset timer
        encoderF = getPosition(); //get final encoder value
        angularVelocity = -main.getVelocity(AngleUnit.RADIANS);
        encoderI = encoderF; //set initial to target encoder value
        if (runToPosition){ //if run to position is true
            vel = pidVP.outputPositional(finalPosition, getPosition());
            if(vel < -3){
                vel = -3;
//                main.setVelocity(-0.45);
            } /*else {
                main.setVelocity(velocity, AngleUnit.RADIANS);
            }*/
            power = pidVV.outputVelocity(vel, angularVelocity, power);
            if (power > 1) power = 1;
            if (power < -1) power = -1;
        } /*else { //if run to position is false*/
        main.setPower(power);
//        }
        aux1.setPower(getMainPower());
        aux2.setPower(getMainPower());
    }

    public void internalEncoderDecelerationProcess(){
        test = true;
        mainTimer.reset(); //reset timer
        encoderF = getPosition(); //get final encoder value
        angularVelocity = -main.getVelocity(AngleUnit.RADIANS);
        encoderI = encoderF; //set initial to target encoder value
        if (runToPosition){ //if run to position is true
            if (finalPosition < 5 && getPosition() < 5) {
                power = 0;
            } else if (finalPosition >= getPosition()){ //if target is higher than current position
                power = pidUp.outputPositionalIntegral(finalPosition, encoderF) + m/1050 * encoderF + c; //calculate power up
            } else if (Math.abs(getPosition()-finalPosition) < uncertainty){ //if target is less than current and the difference is less than 750
                power = m / 1050 * encoderF + c; //maintain position with power
            } else if (/*finalPosition < 5 && */getPosition() < 150) {
                power = Math.min(-0.4, -0.4 * getPosition() / 40);
            } else if (finalPosition < getPosition() - 100) {
                power = 0.3*pidDown.outputPositionalIntegral(finalPosition, encoderF) - Math.max(kv * getAngularVelocity() * getPosition() / 300, downThreshold) + m/1050 * encoderF + c; //calculate power down
            } else { //move down, if either no input / target is lower than current position
                power = 0.3*pidDown.outputPositionalIntegral(finalPosition, encoderF) + m/1050 * encoderF + c; //calculate power down
            }
        }
        if (power > 1) { //set power = 1 if greater than 1
            power = 1;
        } else if (power < -0.4 - (double) getPosition() / 9250){ //set power = -0.6 if less than -0.6
            power = -0.4 - (double) getPosition() / 9250;
        }
//        if (down){
//            main.setVelocity(-Math.sqrt(4*97.6/2/2*((getPosition() - 40)/145.1*Math.PI*2)*1.9)/1.9);
//        } else {
            main.setPower(power); //set power
//        }
        aux1.setPower(getMainPower());
        aux2.setPower(getMainPower());
    }

    public void LiftPositionalProcess(double targetPos){
        runToPosition = true;
        power = pidUp.outputPositional(targetPos, getPosition());
        moveLift(power);
    }

    public void LiftCascadeProcess(double targetPos, Interval... interval){
        runToPosition = true;
        IntervalControl velocityInterval = new IntervalControl(interval);
//        angularVelocity = -main.getVelocity(AngleUnit.RADIANS);

        //only for up
        intervalValue = velocityInterval.getOutput(getPosition());
        cascadeOutput = -cascadePID.cascadeOutput(targetPos, getPosition(), intervalValue, getDerivativeValue());
        moveLift(Math.abs(getCascadeVelocity()) < 0.1 && intervalValue == 0 ? 0 : cascadeOutput); //basically if power is at idle
        testBoolean = Math.abs(getCascadeVelocity()) < 0.1 && intervalValue == 0;
    }


    public void motorTurn(boolean run, int position){
        if (run){
            main.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.finalPosition = position;
            runToPosition = true;
        }
    }

    public void motorTurnPower(boolean run, double power){
        if (run){
            main.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            runToPosition = false;
            this.power = power + 0.2/1050*getPosition();
        }
    }

    public void stop(boolean run){
        if (run){
            main.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            runToPosition = false;
            power = 0;
        }
    }

    //reset encoder
    public void reset(){
        main.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        main.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        aux1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aux1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean isBusy() {
        return main.isBusy() || aux1.isBusy() || aux2.isBusy();
    }

    public int getPosition() {
        return main.getCurrentPosition();
    }

    public int getAuxPos(){
        return aux1.getCurrentPosition();
    }

    public double getMainPower(){
        return main.getPower();
    }

    public double getAux1Power(){
        return aux1.getPower();
    }

    public double getAux2Power(){
        return aux2.getPower();
    }

    public double getAngularVelocity(){
        return angularVelocity;
    }
    public boolean isPositionReached(){
        return Math.abs(finalPosition - getPosition()) < uncertainty;
    }
    public double getDerivativeValue(){
        return -testLiftPID.getDerivative();
    }
    public double getLastErrorValue(){
        return testLiftPID.getLastError();
    }

    public double getErrorValue(){
        return testLiftPID.getError();
    }

    public double getMainCurrent(){
        return main.getCurrent(CurrentUnit.AMPS);
    }

    public double getKvTerm(){
        return kv * getAngularVelocity() * getPosition() / 300;
    }

    public double getUpIntegralSum(){
        return pidUp.getIntegralSum();
    }

    public double getDownIntegralSum(){
        return pidDown.getIntegralSum();
    }

    public int getFinalPosition() {
        return finalPosition;
    }
    public double getIntervalValue() {
        return intervalValue;
    }
    public double getCascadeOutput(){
        return cascadeOutput;
    }
    public double getCascadePositional(){
        return cascadePID.getOutputPositionalValue();
    }
    public double getCascadeVelocity(){
        return cascadePID.getOutputVelocityValue();
    }

    public PIDCore getPidUp() {
        return pidUp;
    }
}
