package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Specifications;

public class IntakeSubsystem extends Specifications {

        private DcMotor intakeMotor;
        private Servo intakeServo;

        public IntakeSubsystem(HardwareMap hardwareMap) {
            intakeMotor = hardwareMap.get(DcMotor.class, Specifications.INTAKE_MOTOR);
            intakeServo = hardwareMap.get(Servo.class, Specifications.INTAKE_SERVO);

            intakeMotor.setDirection(DcMotor.Direction.FORWARD);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            intakeServo.setDirection(Servo.Direction.FORWARD);

        }

        public void setIntakePower(double power) {
            intakeMotor.setPower(power);
        }

        public void stop() {
            intakeMotor.setPower(0);
        }

        public void setIntakePosition(double position) {
            //need to tune these
            intakeServo.setPosition(position);
        }

}
