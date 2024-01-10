package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.util.Specifications;

public class DroneShooter {
    //Servo Variable
    private CRServo droneServo;

    public DroneShooter (HardwareMap hardwareMap) {
        droneServo = hardwareMap.get(CRServo.class, Specifications.DRONE_SHOOTER);
        droneServo.setDirection(DcMotorSimple.Direction.FORWARD); // Spin direction
    }

    public void setContinuousServoPower(double power) {
        droneServo.setPower(power);
    }
}
