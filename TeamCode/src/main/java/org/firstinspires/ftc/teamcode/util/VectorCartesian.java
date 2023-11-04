package org.firstinspires.ftc.teamcode.util;

public class VectorCartesian {
    public double x;
    public double y;
    public double theta;

    public VectorCartesian normalize(){
        double magnitude = Math.sqrt(x*x + y*y);
        return new VectorCartesian(x/magnitude, y/magnitude, theta);
    }
    public void scale(double scaleFactor){
        this.x *= scaleFactor;
        this.y *= scaleFactor;
    }
    public VectorCartesian(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public void setXY(double x, double y){
        this.x = x;
        this.y = y;
    }
    public double getHeading() {
        if (x == 0){
            if (y > 0){
                return Math.PI/2;
            } else {
                return -Math.PI/2;
            }
        } else if (x > 0){
            return Math.atan(y/x);
        } else {
            if (y > 0){
                return Math.atan(y / x) + Math.PI;
            } else {
                return Math.atan(y / x) - Math.PI;
            }
        }
    }


    public double getHeading(double x, double y) {
        if (x == 0){
            if (y > 0){
                return Math.PI/2;
            } else {
                return -Math.PI/2;
            }
        } else if (x > 0){
            return Math.atan(y/x);
        } else {
            if (y > 0){
                return Math.atan(y / x) + Math.PI;
            } else {
                return Math.atan(y / x) - Math.PI;
            }
        }
    }
    public int returnStickPosition(double x, double y){
        double heading = getHeading(x, y);
        if (heading < -Math.PI*7/8){ //left
            return 7;
        } else if (heading < -Math.PI*5/8){ //down left
            return 6;
        } else if (heading < -Math.PI*3/8){ //down
            return 5;
        } else if (heading < -Math.PI/8){ //down right
            return 4;
        } else if (heading < Math.PI/8){ //right
            return 3;
        } else if (heading < Math.PI*3/8){ //up right
            return 2;
        } else if (heading < Math.PI*5/8){ //up
            return 1;
        } else if (heading < Math.PI*7/8){ //up left
            return 8;
        } else { //left (again)
            return 7;
        }
    }

    public double getMagnitude(){
        return Math.sqrt(x * x + y * y);
    }
}