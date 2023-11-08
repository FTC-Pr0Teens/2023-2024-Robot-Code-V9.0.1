package org.firstinspires.ftc.teamcode.command;

import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;
import org.firstinspires.ftc.teamcode.util.Interval;

public class MultiMotorCommand {
    private MultiMotorSubsystem multiMotorSubsystem;
    private int level = -1;

    public MultiMotorCommand(MultiMotorSubsystem multiMotorSubsystem){
        this.multiMotorSubsystem = multiMotorSubsystem;
    }

    public void LiftUp(boolean run, int level){
        this.level = level;
        switch(level){
            case 4:
                Interval interval1 = new Interval(0, 2400, -2500);
                Interval interval2 = new Interval(2400, 2800, -2000);
                Interval interval3 = new Interval(2800, 3000, -1000);
                Interval interval4 = new Interval(3000, 3100, 0);
                if(run) {
                    multiMotorSubsystem.LiftCascadeProcess(3100, interval1, interval2, interval3, interval4);
                }

//                Intervals for Position 4100
//                Interval interval1 = new Interval(0, 3700, -2500);
//                Interval interval2 = new Interval(3400, 3700, -2000);
//                Interval interval3 = new Interval(3700, 4000, -1000);
//                Interval interval4 = new Interval(4000, 4500, 0);
//                if(run) {
//                    multiMotorSubsystem.LiftCascadeProcess(4100, interval1, interval2, interval3, interval4);
//                }
                break;
        }
    }

    public int getLevel() {
        return level;
    }
}
