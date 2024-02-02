package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Specifications;

public class MecanumSubsystem extends Specifications{

    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private DcMotorEx leftForward;
    private DcMotorEx rightForward;
    private final double SCALE = 0.9;

    //rf: right front/forward
    //rb: right back
    //lb: left back
    //lf: left front/forward
    //vel: velocity
    //Main: driver controlled or main program
    //Adjustment: async process controlled

    public double rf = 0;
    public double rb = 0;
    public double lb = 0;
    public double lf = 0;

    public double lfvel = 0;
    public double lbvel = 0;
    public double rfvel = 0;
    public double rbvel = 0;
    public double lfvelMain = 0;
    public double lbvelMain = 0;
    public double rfvelMain = 0;
    public double rbvelMain = 0;
    public double lfvelAdjustment1 = 0;
    public double lbvelAdjustment1 = 0;
    public double rfvelAdjustment1 = 0;
    public double rbvelAdjustment1 = 0;
    public double lfvelAdjustment2 = 0;
    public double lbvelAdjustment2 = 0;
    public double rfvelAdjustment2 = 0;
    public double rbvelAdjustment2 = 0;

    //async process for motor control
    public void motorProcess(){
        if (Math.abs(lfvelMain + lfvelAdjustment1 + lfvelAdjustment2) > MAX_ANGULAR_VEL || Math.abs(lbvelMain + lbvelAdjustment1 + lbvelAdjustment2) > MAX_ANGULAR_VEL || Math.abs(rfvelMain + rfvelAdjustment1 + rfvelAdjustment2) > MAX_ANGULAR_VEL || Math.abs(rbvelMain + rbvelAdjustment1 + rbvelAdjustment2) > MAX_ANGULAR_VEL){
            lfvel = (lfvelMain + lfvelAdjustment1 + lfvelAdjustment2);
            lbvel = (lbvelMain + lbvelAdjustment1 + lbvelAdjustment2);
            rfvel = (rfvelMain + rfvelAdjustment1 + rfvelAdjustment2);
            rbvel = (rbvelMain + rbvelAdjustment1 + rbvelAdjustment2);
            double max = Math.max(Math.abs(lfvel), Math.max(Math.abs(lbvel), Math.max(Math.abs(rfvel), Math.abs(rbvel))));
            lfvel = lfvel/max*MAX_ANGULAR_VEL;
            lbvel = lbvel/max*MAX_ANGULAR_VEL;
            rfvel = rfvel/max*MAX_ANGULAR_VEL;
            rbvel = rbvel/max*MAX_ANGULAR_VEL;
        } else {
            lfvel = lfvelMain + lfvelAdjustment1 + lfvelAdjustment2;
            lbvel = lbvelMain + lbvelAdjustment1 + lbvelAdjustment2;
            rfvel = rfvelMain + rfvelAdjustment1 + rfvelAdjustment2;
            rbvel = rbvelMain + rbvelAdjustment1 + rbvelAdjustment2;
        }
        rightForward.setVelocity(rfvel, AngleUnit.RADIANS);
        leftBack.setVelocity(lbvel, AngleUnit.RADIANS);
        rightBack.setVelocity(rbvel, AngleUnit.RADIANS);
        leftForward.setVelocity(lfvel, AngleUnit.RADIANS);
    }

    public void motorProcessTeleOp(){
        if (Math.abs(lfvelMain + lfvelAdjustment1 + lfvelAdjustment2) > MAX_ANGULAR_VEL
                || Math.abs(lbvelMain + lbvelAdjustment1 + lbvelAdjustment2) > MAX_ANGULAR_VEL
                || Math.abs(rfvelMain + rfvelAdjustment1 + rfvelAdjustment2) > MAX_ANGULAR_VEL
                || Math.abs(rbvelMain + rbvelAdjustment1 + rbvelAdjustment2) > MAX_ANGULAR_VEL){
            lfvel = (lfvelMain + lfvelAdjustment1 + lfvelAdjustment2);
            lbvel = (lbvelMain + lbvelAdjustment1 + lbvelAdjustment2);
            rfvel = (rfvelMain + rfvelAdjustment1 + rfvelAdjustment2);
            rbvel = (rbvelMain + rbvelAdjustment1 + rbvelAdjustment2);
            double max = Math.max(Math.abs(lfvel), Math.max(Math.abs(lbvel), Math.max(Math.abs(rfvel), Math.abs(rbvel))));
            lfvel = lfvel/max;
            lbvel = lbvel/max;
            rfvel = rfvel/max;
            rbvel = rbvel/max;
        } else {
            lfvel = lfvelMain + lfvelAdjustment1 + lfvelAdjustment2;
            lbvel = lbvelMain + lbvelAdjustment1 + lbvelAdjustment2;
            rfvel = rfvelMain + rfvelAdjustment1 + rfvelAdjustment2;
            rbvel = rbvelMain + rbvelAdjustment1 + rbvelAdjustment2;
        }
        rightForward.setPower(rfvel);
        leftBack.setPower(lbvel);
        rightBack.setPower(rbvel);
        leftForward.setPower(lfvel);
    }

    public MecanumSubsystem(HardwareMap hardwareMap) {

        leftBack = hardwareMap.get(DcMotorEx.class, BKLF_MOTOR);
        rightBack = hardwareMap.get(DcMotorEx.class, BKRT_MOTOR);
        leftForward = hardwareMap.get(DcMotorEx.class, FTLF_MOTOR);
        rightForward = hardwareMap.get(DcMotorEx.class, FTRT_MOTOR);

        rightForward.setDirection(DcMotorSimple.Direction.REVERSE);
        leftForward.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        turnOffInternalPID();
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setPower(0);
        leftForward.setPower(0);
        rightBack.setPower(0);
        rightForward.setPower(0);
    }
    public void turnOffInternalPID() {
        rightForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void forward (double power) {
        leftBack.setPower(power);
        leftForward.setPower(power);
        rightBack.setPower(power);
        rightForward.setPower(power);
    }

    public void move(boolean run, double vertical, double horizontal, double rotational){
        if (run){
            rb = (vertical * Math.cos(Math.toRadians(45)) + horizontal * Math.sin(Math.toRadians(45)) + rotational * Math.sin(Math.toRadians(45)))*(1.41421356237);
            rf = (-horizontal * Math.cos(Math.toRadians(45)) + vertical * Math.sin(Math.toRadians(45)) + rotational * Math.sin(Math.toRadians(45)))*(1.41421356237);
            lf = (vertical * Math.cos(Math.toRadians(45)) + horizontal * Math.sin(Math.toRadians(45)) - rotational * Math.sin(Math.toRadians(45)))*(1.41421356237);
            lb = (-horizontal * Math.cos(Math.toRadians(45)) + vertical * Math.sin(Math.toRadians(45)) - rotational * Math.sin(Math.toRadians(45)))*(1.41421356237);

            rightForward.setPower(rf);
            leftBack.setPower(lb);
            rightBack.setPower(rb);
            leftForward.setPower(lf);
        }
    }

    public void moveVelocity(boolean run, double vertical, double horizontal, double rotational){
        if (run){
            rb = (vertical * Math.cos(Math.toRadians(45)) + horizontal * Math.sin(Math.toRadians(45)) + rotational * Math.sin(Math.toRadians(45)))*(1.41421356237);
            rf = (-horizontal * Math.cos(Math.toRadians(45)) + vertical * Math.sin(Math.toRadians(45)) + rotational * Math.sin(Math.toRadians(45)))*(1.41421356237);
            lf = (vertical * Math.cos(Math.toRadians(45)) + horizontal * Math.sin(Math.toRadians(45)) - rotational * Math.sin(Math.toRadians(45)))*(1.41421356237);
            lb = (-horizontal * Math.cos(Math.toRadians(45)) + vertical * Math.sin(Math.toRadians(45)) - rotational * Math.sin(Math.toRadians(45)))*(1.41421356237);

            rightForward.setVelocity(rf);
            leftBack.setVelocity(lb);
            rightBack.setVelocity(rb);
            leftForward.setVelocity(lf);
        }
    }

    //used with motorProcess or motorProcessTeleop
    public void partialMove(boolean run, double verticalVel, double horizontalVel, double rotationalVel){
//        verticalVel = 0;
//        horizontalVel = 0;
        if (run){
            rbvelMain = (verticalVel * Math.cos(Math.toRadians(45)) + horizontalVel * Math.sin(Math.toRadians(45)) + rotationalVel * Math.sin(Math.toRadians(45)))*(1.41421356237);
            rfvelMain = (-horizontalVel * Math.cos(Math.toRadians(45)) + verticalVel * Math.sin(Math.toRadians(45)) + rotationalVel * Math.sin(Math.toRadians(45)))*(1.41421356237);
            lfvelMain = (verticalVel * Math.cos(Math.toRadians(45)) + horizontalVel * Math.sin(Math.toRadians(45)) - rotationalVel * Math.sin(Math.toRadians(45)))*(1.41421356237);
            lbvelMain = (-horizontalVel * Math.cos(Math.toRadians(45)) + verticalVel * Math.sin(Math.toRadians(45)) - rotationalVel * Math.sin(Math.toRadians(45)))*(1.41421356237);
        }
    }

    public void partialMoveAdjustment1(boolean run, double verticalVel, double horizontalVel, double rotationalVel){
        if (run){
            rbvelAdjustment1 = (verticalVel * Math.cos(Math.toRadians(45)) + horizontalVel * Math.sin(Math.toRadians(45)) + rotationalVel * Math.sin(Math.toRadians(45)))*(1.41421356237);
            rfvelAdjustment1 = (-horizontalVel * Math.cos(Math.toRadians(45)) + verticalVel * Math.sin(Math.toRadians(45)) + rotationalVel * Math.sin(Math.toRadians(45)))*(1.41421356237);
            lfvelAdjustment1 = (verticalVel * Math.cos(Math.toRadians(45)) + horizontalVel * Math.sin(Math.toRadians(45)) - rotationalVel * Math.sin(Math.toRadians(45)))*(1.41421356237);
            lbvelAdjustment1 = (-horizontalVel * Math.cos(Math.toRadians(45)) + verticalVel * Math.sin(Math.toRadians(45)) - rotationalVel * Math.sin(Math.toRadians(45)))*(1.41421356237);
        }
    }

    public void partialMoveAdjustment2(boolean run, double verticalVel, double horizontalVel, double rotationalVel){
        if (run){
            rbvelAdjustment2 = (verticalVel * Math.cos(Math.toRadians(45)) + horizontalVel * Math.sin(Math.toRadians(45)) + rotationalVel * Math.sin(Math.toRadians(45)))*(1.41421356237);
            rfvelAdjustment2 = (-horizontalVel * Math.cos(Math.toRadians(45)) + verticalVel * Math.sin(Math.toRadians(45)) + rotationalVel * Math.sin(Math.toRadians(45)))*(1.41421356237);
            lfvelAdjustment2 = (verticalVel * Math.cos(Math.toRadians(45)) + horizontalVel * Math.sin(Math.toRadians(45)) - rotationalVel * Math.sin(Math.toRadians(45)))*(1.41421356237);
            lbvelAdjustment2 = (-horizontalVel * Math.cos(Math.toRadians(45)) + verticalVel * Math.sin(Math.toRadians(45)) - rotationalVel * Math.sin(Math.toRadians(45)))*(1.41421356237);
        }
    }

    //TeleOpMove (tank drive)
    public void move(boolean run, double leftVertical, double leftHorizontal, double rightVertical, double rightHorizontal){
        if (run){
            rf = rightHorizontal+rightVertical;
            lb = leftVertical+leftHorizontal;
            rb = rightVertical-rightHorizontal;
            lf = leftVertical-leftHorizontal;

            rightForward.setPower(rf);
            leftBack.setPower(lb);
            rightBack.setPower(rb);
            leftForward.setPower(lf);
        }
    }

    // Look forward but move at angle
    public void moveAngle(boolean run, double power, double degree){
        if (run){
            double y1 = power * Math.sin(Math.toRadians(degree)) * Math.cos(Math.toRadians(45)) - power * Math.cos(Math.toRadians(degree)) * Math.sin(Math.toRadians(45));
            double x1 = power * Math.cos(Math.toRadians(degree)) * Math.cos(Math.toRadians(45)) + power * Math.sin(Math.toRadians(degree)) * Math.sin(Math.toRadians(45));
            double y2 = power * Math.sin(Math.toRadians(degree)) * Math.cos(Math.toRadians(45)) - power * Math.cos(Math.toRadians(degree)) * Math.sin(Math.toRadians(45));
            double x2 = power * Math.cos(Math.toRadians(degree)) * Math.cos(Math.toRadians(45)) + power * Math.sin(Math.toRadians(degree)) * Math.sin(Math.toRadians(45));
            rightForward.setPower(x1);
            leftBack.setPower(x2);
            rightBack.setPower(y1);
            leftForward.setPower(y2);
        }
    }

    //Limits angle to 0-2Pi, and converts it to degrees

    // getBearing() takes the x and y coordinates of the final location as inputs, and returns
    // the degrees the robot should turn to have a straight path to its location

//    public double getBearing(double xFinal, double yFinal){
//
//        // Calculates the final bearing from a position of zero degrees
//        double bearingFromZero = Math.atan2(yFinal - y, xFinal - x);
//        if(bearingFromZero < 0) {
//            bearingFromZero += 2*Math.PI;
//        }
//
//        // Calculates current bearing from zero degrees
//        double currentBearing = convertBearing(Theta);
//
//        double bearingCorrected = Math.toDegrees(bearingFromZero) - currentBearing;
//        if (bearingCorrected > 180) {
//            bearingCorrected -= 360;
//        }
//        if (bearingCorrected < -180) {
//            bearingCorrected += 360;
//        }
//
//        //Uncomment this if you want to return bearing from current angle rather than from zero
//        //return bearingCorrected;
//        return Math.toDegrees(bearingFromZero);
//    }

    //Determines the distance between the current location, and the final specified location
//    public double calculateDistance(double xFinal, double yFinal) {
//
//        return Math.hypot(xFinal - x, yFinal - y);
//    }

    //Returns distance travelled

//    public double distanceTravelled(double curX, double curY) {
//
//        double xTravelled = Math.abs(x - curX);
//        double yTravelled = Math.abs(y - curY);
//        return (Math.sqrt((xTravelled*xTravelled) + (yTravelled*yTravelled)));
//    }

    //Moves the robot forwards, counting the distance that it has moved

//    public double linearMotion(double pow) {
//
//        OdometryProcess();
//        double initialDis = rightEncoder();
//        rightForward.setPower(pow);
//        rightBack.setPower(pow);
//        leftForward.setPower(pow);
//        leftBack.setPower(pow);
//        OdometryProcess();
//        double finalDis = rightEncoder();
//        return (odometryCir*(Math.abs((finalDis-initialDis))/odometryTick));
//    }

    //Don't use this move
    public void fieldOrientedMove(double x, double y, double z, double theta) {

        // translate the field relative movement (joystick) into the robot relative movement

        //changed all 3 lines below



        //drive gears code
//        double angle = Math.PI/2 - theta;
//        //Drive Gears
//        double newX =  y * Math.sin(theta) + x * Math.sin(angle);
//        double newY = y * Math.cos(theta) - x * Math.cos(angle);

        double newX = x * Math.cos(theta) + y * Math.sin(theta);
        double newY = -x * Math.sin(theta) + y * Math.cos(theta);



        //TODO: code below is after edward change


       // double newX = y * Math.cos(theta) + x * Math.cos(Math.PI/2 + theta);
       // double newY = y * Math.sin(theta) + x * Math.sin(Math.PI/2 + theta);


//        double newY = y * Math.cos(theta) + x * Math.sin(theta);
//        double newX = - y * Math.sin(theta) + x * Math.cos(theta);


       // double newX = x * Math.cos(theta) + y * Math.cos(Math.PI/2 + theta);
       // double newY = x * Math.sin(theta) + y * Math.sin(Math.PI/2 + theta);

//        double newX = x * Math.cos(theta) + y * Math.cos(Math.PI/2 + theta);
//        double newY = x * Math.sin(theta) + y * Math.sin(Math.PI/2 + theta);



        double frontRightPow = - newY + newX - z;
        double frontLeftPow = newY + newX + z;
        double backRightPow = newY + newX -  z;
        double backLeftPow = - newY + newX + z;

//        frontRightPow = - newX + newY - z;
//        frontLeftPow = newX + newY + z;
//        backRightPow = newX + newY - z;
//        backLeftPow = - newX + newY + z;



        double largest = Math.max(
                Math.max(Math.abs(frontRightPow), Math.abs(frontLeftPow)),
                Math.max(Math.abs(backRightPow), Math.abs(backLeftPow)));
        if (largest > 1) {
            frontRightPow /= largest;
            frontLeftPow /= largest;
            backRightPow /= largest;
            backLeftPow /= largest;
        }

        rightForward.setPower(frontRightPow * SCALE);
        leftForward.setPower(frontLeftPow * SCALE);
        rightBack.setPower(backRightPow * SCALE);
        leftBack.setPower(backLeftPow * SCALE);
    }



    public void moveToPosition(boolean run, double power, double degree, int position){
        if (run){
            double y1 = power * Math.sin(Math.toRadians(degree)) * Math.cos(Math.toRadians(45)) - power * Math.cos(Math.toRadians(degree)) * Math.sin(Math.toRadians(45));
            double x1 = power * Math.cos(Math.toRadians(degree)) * Math.cos(Math.toRadians(45)) + power * Math.sin(Math.toRadians(degree)) * Math.sin(Math.toRadians(45));
            double y2 = power * Math.sin(Math.toRadians(degree)) * Math.cos(Math.toRadians(45)) - power * Math.cos(Math.toRadians(degree)) * Math.sin(Math.toRadians(45));
            double x2 = power * Math.cos(Math.toRadians(degree)) * Math.cos(Math.toRadians(45)) + power * Math.sin(Math.toRadians(degree)) * Math.sin(Math.toRadians(45));
            while (rightForward.getCurrentPosition()<position){
                rightForward.setPower(x1);
                leftBack.setPower(x2);
                rightBack.setPower(y1);
                leftForward.setPower(y2);
            }
            rightForward.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            leftForward.setPower(0);
        }
    }

    public void stop(boolean run){
        if (run){
            rightForward.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            leftForward.setPower(0);
        }
    }

    public void reset(){
        rightForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightForward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftForward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

//    public void ResetHeadingChange() { headingChange = 0; }
//
//    public void setHeadingChange(double change) { headingChange += change; }

    public double rightForwardPow() {
        return rightForward.getPower();
    }

    public double rightBackPow() {
        return rightBack.getPower();
    }

    public double leftForwardPow() {
        return leftForward.getPower();
    }

    public double leftBackPow() {
        return leftBack.getPower();
    }

    public DcMotorEx getLeftBack() { return leftBack; }

    public DcMotorEx getRightBack() { return rightBack; }

    public DcMotorEx getLeftForward() { return leftForward; }

    public DcMotorEx getRightForward() { return rightForward; }

    public double getLeftBackVelocity() { return leftBack.getVelocity(AngleUnit.RADIANS); }

    public double getRightBackVelocity() { return rightBack.getVelocity(AngleUnit.RADIANS); }

    public double getLeftForwardVelocity() { return leftForward.getVelocity(AngleUnit.RADIANS); }

    public double getRightForwardVelocity() { return rightForward.getVelocity(AngleUnit.RADIANS); }

    public double rightForwardPos() {
        return rightForward.getCurrentPosition();
    }

    public double rightBackPos() {
        return rightBack.getCurrentPosition();
    }

    public double leftForwardPos() {
        return leftForward.getCurrentPosition();
    }

    public double leftBackPos() {
        return leftBack.getCurrentPosition();
    }
}
