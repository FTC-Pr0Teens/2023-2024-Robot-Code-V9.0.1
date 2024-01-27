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

    //0.03 close
    //0.115 open

    public void setPos(double pos){ droneServo.setPosition(pos); }

    public void lock(){
        droneServo.setPosition(0.6);
    }

    public void launch (){
        droneServo.setPosition(0.05);
    }

}
