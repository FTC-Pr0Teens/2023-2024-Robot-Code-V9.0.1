package org.firstinspires.ftc.teamcode.command;

import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;
import org.firstinspires.ftc.teamcode.util.Interval;

public class MultiMotorCommand {
    private MultiMotorSubsystem multiMotorSubsystem;
    private int level = -1;

    public MultiMotorCommand(MultiMotorSubsystem multiMotorSubsystem){
        this.multiMotorSubsystem = multiMotorSubsystem;
    }

    public void LiftUpPositional(int level){
        switch(level){
            case 0:
                multiMotorSubsystem.LiftPositionalProcess(0);
                break;
            case 1:
                multiMotorSubsystem.LiftPositionalProcess(450);
                break;
            case 2:
                multiMotorSubsystem.LiftPositionalProcess(1200);
                break;
            case 3:
                multiMotorSubsystem.LiftPositionalProcess(1800);
                break;
            case 4:
                multiMotorSubsystem.LiftPositionalProcess(2200);
                break;
            case 5:
                multiMotorSubsystem.LiftPositionalProcess(900);
                break;
        }
    }
    public void LiftUp(boolean run, int level){
        this.level = level;
        Interval interval1;
        Interval interval2;
        Interval interval3;
        Interval interval4;

        //in case 0, 1, and 2 start and end values are reversed (neg becomes pos, pos becomes neg)
        //start and end values are switched (greatest to least now)


        switch(level){
            case 0:
                interval1 = new Interval(-3400, -1000, 2000);
                interval2 = new Interval(-1000, -500, 1500);
                interval3 = new Interval(-500, -18, 500);
                interval4 = new Interval(-18, 400, 0);
                if(run) {
                    multiMotorSubsystem.LiftCascadeProcess(0, interval1, interval2, interval3, interval4);
                }
                break;
            case 1:
                interval1 = new Interval(-300, 400, -1000);
                interval2 = new Interval(-400, -300, -400);
                interval3 = new Interval(-2000, -250, 0);
                //TODO: deceleration intervals
                if(run) {
                    multiMotorSubsystem.LiftCascadeProcess(-1100, interval1, interval2, interval3);
                }
                break;
            case 2:
                interval1 = new Interval(-900, 400, -2000);
                interval2 = new Interval(-1075, -900, -1700);
                interval3 = new Interval(-1150, -1075, -900);
                interval4 = new Interval(-1350-2000, -1150, 0);
                if(run) {
                    multiMotorSubsystem.LiftCascadeProcess(-2020, interval1, interval2, interval3, interval4);
                }
                break;
            case 3:
                interval1 = new Interval(-400, 1950, -2000);
                interval2 = new Interval(1950, 2100, -1700);
                interval3 = new Interval(2100, 2200, -1000);
                interval4 = new Interval(2200, 3000, 0);
                if(run) {
                    multiMotorSubsystem.LiftCascadeProcess(-2317, interval1, interval2, interval3, interval4);
                }
                break;
            case 4:
                interval1 = new Interval(-400, 2700, -2000);
                interval2 = new Interval(2700, 2800, -1700);
                interval3 = new Interval(2800, 3000, -1000);
                interval4 = new Interval(3000, 3400, 0);
                if(run) {
                    multiMotorSubsystem.LiftCascadeProcess(-2700, interval1, interval2, interval3, interval4);
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
            default:
//                multiMotorSubsystem.moveLift(0);
                break;
        }
    }

    public int getLevel() {
        return level;
    }
}
