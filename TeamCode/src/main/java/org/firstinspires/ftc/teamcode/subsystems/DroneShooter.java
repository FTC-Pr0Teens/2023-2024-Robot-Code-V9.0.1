package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.util.Specifications;

public class DroneShooter {
    //Servo Variable
    private Servo droneServo;

    public DroneShooter (HardwareMap hardwareMap) {
        droneServo = hardwareMap.get(Servo.class, Specifications.DRONE_SHOOTER);
        // Spin direction
    }

    public void launch (){
        droneServo.setPosition(1);
    }
}
