package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Specifications;

@TeleOp (name = "Hanging Test")
public class HangingTesting extends LinearOpMode {
    private Servo hangingServoL;
    private Servo hangingServoR;
    private DcMotor hangingMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        //INITIALIZES THE HANGING SERVO
        hangingServoL = hardwareMap.get(Servo.class, Specifications.HANGING_SERVO_L);
        hangingServoL.setPosition(0.35);

        hangingServoR = hardwareMap.get(Servo.class, Specifications.HANGING_SERVO_R);
        hangingServoR.setPosition(0.35);
////
////    INITIALIZES THE HANGING MOTOR
        hangingMotor = hardwareMap.dcMotor.get(Specifications.HANGING_MOTOR);
        hangingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangingMotor.setPower(0);

        boolean hangingArmInPlace = false;
        boolean robotIsHanging = false;

        waitForStart();

        hangingServoL.setPosition(0.4);
        hangingServoR.setPosition(0.4);

        while(opModeIsActive()) {
            //hangingServo toggle
            if (gamepad1.dpad_up) {
                hangingServoL.setPosition(0.95);
                hangingServoR.setPosition(0.95);
            }
            //start button is for turning on the hanging motor
            if (gamepad1.dpad_right) {
                hangingMotor.setPower(0.7);
                /*while (timer.milliseconds() < 5000) {
                    hangingMotor.setPower(0.7);
                 */
            } else {
                hangingMotor.setPower(0);
                //Dpad up for unhanging
            }
            if (gamepad1.dpad_down) {
                //motor will spin in the opposite direction until it reaches the end ground
                hangingMotor.setPower(-0.7);
                /*while (timer.milliseconds() < 5000) {
                    hangingMotor.setPower(-0.7);
                }*/
                telemetry.addLine("hanging reversed");
            }

            if (gamepad1.a){
                setHangPosition(0.2);
            } else if (gamepad1.x){
                setHangPosition(0.4);
            } else if (gamepad1.y){
                setHangPosition(0.6);
            } else if (gamepad1.b){
                setHangPosition(0.8);
            }
            telemetry.addData("Right pos:", hangingServoR.getPosition());
            telemetry.addData("Left pos:", hangingServoL.getPosition());
            telemetry.update();
        }
    }

    private void setHangPosition(double position){
        hangingServoL.setPosition(position);
        hangingServoR.setPosition(position);
    }
}
