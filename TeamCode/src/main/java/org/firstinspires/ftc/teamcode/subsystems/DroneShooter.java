package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.*;

public class DroneShooter {
    //Servo Variable
    private CRServo continuousServo;

    public DroneShooter (HardwareMap hardwareMap) {
        continuousServo = hardwareMap.get(CRServo.class, "continuousServo");
        continuousServo.setDirection(DcMotorSimple.Direction.FORWARD); // Spin direction
    }

    public void setContinuousServoPower(double power) {
        continuousServo.setPower(power);
    }
}
