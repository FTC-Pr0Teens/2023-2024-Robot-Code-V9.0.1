package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.List;

/**
 * Class to deal with expanded hardware-level interface methods. Such as literally breaking your control hub
 * or giving it steroids. I don't know, I'm using it for better sensor reads and LED color only. -Leo
 */
public class ExpandedHardware {
    List<LynxModule> allHubs;

    public ExpandedHardware(HardwareMap hardwareMap){
        allHubs = hardwareMap.getAll(LynxModule.class);
    }

    /**
     * This is optional mode, but can enhance software loop times. See for more info
     * https://gm0.org/en/latest/docs/software/tutorials/bulk-reads.html
     */
    public void enableBulkCache(){
        for(LynxModule hub : allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    /**
     * PLEASE read before you use this.
     * https://gm0.org/en/latest/docs/software/tutorials/bulk-reads.html
     */
    public void enableManualBulkCache(){
        for(LynxModule hub : allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    /**
     * This is default mode. See for more info
     * https://gm0.org/en/latest/docs/software/tutorials/bulk-reads.html
     */
    public void disableBulkCache(){
        for(LynxModule hub : allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
        }
    }

    /**
     * haha funny (i havent figured out how it actually works, like is it overridden by error codes?)
     * @param color to set to ALL hubs (add a second argument for individual hubs)
     *
     */
    public void setLedColor(int color){
        for(LynxModule hub : allHubs){
            hub.setConstant(color);

        }
    }

    /**
     * haha funny (i havent figured out how it actually works, like is it overridden by error codes?)

     * @param color to set
     * @param hubNumber hub number (starts at 0 for controlHub, 1 for expansion hub)
     */
    public void setLedColor(int color, int hubNumber){
        allHubs.get(hubNumber).setConstant(color);
    }

}
