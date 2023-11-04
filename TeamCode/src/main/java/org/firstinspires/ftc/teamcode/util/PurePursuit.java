package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;

//All coordinates are in normal cartesian coordinates (x as horizontal, y as vertical)


//All coordinates are in normal cartesian coordinates (x as horizontal, y as vertical)
public class PurePursuit {
    private ArrayList<Waypoint> points;
    public ArrayList<Waypoint> finalPoints = new ArrayList<>();
    public int currentFinalTargetPoint = 0;
    private ArrayList<Double> slopes = new ArrayList<>();
    private double lookAhead;
    private int currentTargetPoint = 1;
    boolean isIntersection;

    public PurePursuit(ArrayList<Waypoint> points, double lookAhead){
        this.lookAhead = lookAhead;
        this.points = points;
        for (int i = 0; i < points.size() - 1; i++){
            if (points.get(i + 1).x - points.get(i).x != 0){
                slopes.add((points.get(i + 1).y - points.get(i).y) / (points.get(i + 1).x - points.get(i).x));
            } else {
                slopes.add(null);
            }
            if (points.get(i + 1).finalPoint){
                finalPoints.add(points.get(i + 1));
            }
        }
    }

    public void addPoint(Waypoint point) {
        points.add(point);
        slopes.add((points.get(points.size() - 1).y-points.get(points.size() - 2).y) / (points.get(points.size() - 1).x-points.get(points.size() - 2).x));
    }

    public void nextPoint(){
        currentTargetPoint++;
        currentFinalTargetPoint++;
    }

    //return x as vertical, y as horizontal
    public VectorCartesian getTarget(double normalCurrentX, double normalCurrentY){
        double x;
        double y;
        double nextSegmentPerpendicularSlope;
        double distanceToNextSegment;
        if (!points.get(currentTargetPoint).finalPoint){
            if (slopes.get(currentTargetPoint) == null) {
                distanceToNextSegment = Math.abs(points.get(currentTargetPoint).x - normalCurrentX);
                double maxY = Math.max(points.get(currentTargetPoint).y, points.get(currentTargetPoint + 1).y);
                double minY = Math.min(points.get(currentTargetPoint).y, points.get(currentTargetPoint + 1).y);
                if (normalCurrentY > maxY) {
                    distanceToNextSegment = Math.sqrt(Math.pow(points.get(currentTargetPoint).x - normalCurrentX, 2) + Math.pow(maxY - normalCurrentY, 2));
                } else if (normalCurrentY < minY) {
                    distanceToNextSegment = Math.sqrt(Math.pow(points.get(currentTargetPoint).x - normalCurrentX, 2) + Math.pow(minY - normalCurrentY, 2));
                }
            } else if (slopes.get(currentTargetPoint) == 0) {
                distanceToNextSegment = Math.abs(points.get(currentTargetPoint).y - normalCurrentY);
                double maxX = Math.max(points.get(currentTargetPoint).x, points.get(currentTargetPoint + 1).x);
                double minX = Math.min(points.get(currentTargetPoint).x, points.get(currentTargetPoint + 1).x);
                if (normalCurrentX > maxX) {
                    distanceToNextSegment = Math.sqrt(Math.pow(maxX - normalCurrentX, 2) + Math.pow(points.get(currentTargetPoint).y - normalCurrentY, 2));
                } else if (normalCurrentX < minX) {
                    distanceToNextSegment = Math.sqrt(Math.pow(minX - normalCurrentX, 2) + Math.pow(points.get(currentTargetPoint).y - normalCurrentY, 2));
                }
            } else {
                double nextSegmentIntersectX;
                double nextSegmentIntersectY;
                nextSegmentPerpendicularSlope = -1 / slopes.get(currentTargetPoint);
                nextSegmentIntersectX = (points.get(currentTargetPoint).y - slopes.get(currentTargetPoint) * points.get(currentTargetPoint).x - normalCurrentY + nextSegmentPerpendicularSlope * normalCurrentX) / (nextSegmentPerpendicularSlope - slopes.get(currentTargetPoint));
                nextSegmentIntersectY = slopes.get(currentTargetPoint) * nextSegmentIntersectX + points.get(currentTargetPoint).y - slopes.get(currentTargetPoint) * points.get(currentTargetPoint).x;
                distanceToNextSegment = Math.sqrt(Math.pow(nextSegmentIntersectX - normalCurrentX, 2) + Math.pow(nextSegmentIntersectY - normalCurrentY, 2));
                if (slopes.get(currentTargetPoint) > 0) {
                    if (points.get(currentTargetPoint).x < points.get(currentTargetPoint + 1).x) {
                        if (nextSegmentIntersectY <= nextSegmentPerpendicularSlope * nextSegmentIntersectX + points.get(currentTargetPoint).y - nextSegmentPerpendicularSlope * points.get(currentTargetPoint).x) {
                            distanceToNextSegment = Math.sqrt(Math.pow(points.get(currentTargetPoint).x - normalCurrentX, 2) + Math.pow(points.get(currentTargetPoint).y - normalCurrentY, 2));
                        } else if (nextSegmentIntersectY >= nextSegmentPerpendicularSlope * nextSegmentIntersectX + points.get(currentTargetPoint + 1).y - nextSegmentPerpendicularSlope * points.get(currentTargetPoint + 1).x) {
                            distanceToNextSegment = Math.sqrt(Math.pow(points.get(currentTargetPoint + 1).x - normalCurrentX, 2) + Math.pow(points.get(currentTargetPoint + 1).y - normalCurrentY, 2));
                        }
                    } else {
                        if (nextSegmentIntersectY <= nextSegmentPerpendicularSlope * nextSegmentIntersectX + points.get(currentTargetPoint + 1).y - nextSegmentPerpendicularSlope * points.get(currentTargetPoint + 1).x) {
                            distanceToNextSegment = Math.sqrt(Math.pow(points.get(currentTargetPoint + 1).x - normalCurrentX, 2) + Math.pow(points.get(currentTargetPoint + 1).y - normalCurrentY, 2));
                        } else if (nextSegmentIntersectY >= nextSegmentPerpendicularSlope * nextSegmentIntersectX + points.get(currentTargetPoint).y - nextSegmentPerpendicularSlope * points.get(currentTargetPoint).x) {
                            distanceToNextSegment = Math.sqrt(Math.pow(points.get(currentTargetPoint).x - normalCurrentX, 2) + Math.pow(points.get(currentTargetPoint).y - normalCurrentY, 2));
                        }
                    }
                } else {
                    if (points.get(currentTargetPoint).x < points.get(currentTargetPoint + 1).x) {
                        if (nextSegmentIntersectY >= nextSegmentPerpendicularSlope * nextSegmentIntersectX + points.get(currentTargetPoint).y - nextSegmentPerpendicularSlope * points.get(currentTargetPoint).x) {
                            distanceToNextSegment = Math.sqrt(Math.pow(points.get(currentTargetPoint).x - normalCurrentX, 2) + Math.pow(points.get(currentTargetPoint).y - normalCurrentY, 2));
                        } else if (nextSegmentIntersectY <= nextSegmentPerpendicularSlope * nextSegmentIntersectX + points.get(currentTargetPoint + 1).y - nextSegmentPerpendicularSlope * points.get(currentTargetPoint + 1).x) {
                            distanceToNextSegment = Math.sqrt(Math.pow(points.get(currentTargetPoint + 1).x - normalCurrentX, 2) + Math.pow(points.get(currentTargetPoint + 1).y - normalCurrentY, 2));
                        }
                    } else {
                        if (nextSegmentIntersectY >= nextSegmentPerpendicularSlope * nextSegmentIntersectX + points.get(currentTargetPoint + 1).y - nextSegmentPerpendicularSlope * points.get(currentTargetPoint + 1).x) {
                            distanceToNextSegment = Math.sqrt(Math.pow(points.get(currentTargetPoint + 1).x - normalCurrentX, 2) + Math.pow(points.get(currentTargetPoint + 1).y - normalCurrentY, 2));
                        } else if (nextSegmentIntersectY <= nextSegmentPerpendicularSlope * nextSegmentIntersectX + points.get(currentTargetPoint).y - nextSegmentPerpendicularSlope * points.get(currentTargetPoint).x) {
                            distanceToNextSegment = Math.sqrt(Math.pow(points.get(currentTargetPoint).x - normalCurrentX, 2) + Math.pow(points.get(currentTargetPoint).y - normalCurrentY, 2));
                        }
                    }
                }
            }
            if (distanceToNextSegment < lookAhead) {
                currentTargetPoint++;
            }
        }
        if (slopes.get(currentTargetPoint - 1) == null) {
            x = points.get(currentTargetPoint).x;
            if (lookAhead * lookAhead - (x - normalCurrentX) * (x - normalCurrentX) >= 0) {
                if (points.get(currentTargetPoint).y > normalCurrentY) {
                    y = Math.sqrt(lookAhead * lookAhead - (x - normalCurrentX) * (x - normalCurrentX)) + normalCurrentY;
                } else if (points.get(currentTargetPoint).y < normalCurrentY) {
                    y = -Math.sqrt(lookAhead * lookAhead - (x - normalCurrentX) * (x - normalCurrentX)) + normalCurrentY;
                } else {
                    y = normalCurrentY;
                }
            } else {
                y = normalCurrentY;
            }
            double maxTargetY = Math.max(points.get(currentTargetPoint).y, points.get(currentTargetPoint - 1).y);
            double minTargetY = Math.min(points.get(currentTargetPoint).y, points.get(currentTargetPoint - 1).y);
            if (y > maxTargetY){
                y = maxTargetY;
            } else if (y < minTargetY) {
                y = minTargetY;
            }
        } else if(slopes.get(currentTargetPoint - 1) == 0) {
            y = points.get(currentTargetPoint).y;
            if (lookAhead * lookAhead - (y - normalCurrentY) * (y - normalCurrentY) >= 0) {
                if (points.get(currentTargetPoint).x > normalCurrentX) {
                    x = Math.sqrt(lookAhead * lookAhead - (y - normalCurrentY) * (y - normalCurrentY)) + normalCurrentX;
                } else if (points.get(currentTargetPoint).x < normalCurrentX) {
                    x = -Math.sqrt(lookAhead * lookAhead - (y - normalCurrentY) * (y - normalCurrentY)) + normalCurrentX;
                } else {
                    x = normalCurrentX;
                }
            } else {
                x = normalCurrentX;
            }
            double maxTargetX = Math.max(points.get(currentTargetPoint).x, points.get(currentTargetPoint - 1).x);
            double minTargetX = Math.min(points.get(currentTargetPoint).x, points.get(currentTargetPoint - 1).x);
            if (x > maxTargetX){
                x = maxTargetX;
            } else if (x < minTargetX) {
                x = minTargetX;
            }
        } else {
            double a = slopes.get(currentTargetPoint - 1) * slopes.get(currentTargetPoint - 1) + 1;
            double b = 2 * slopes.get(currentTargetPoint - 1) * points.get(currentTargetPoint - 1).y - 2 * slopes.get(currentTargetPoint - 1) * slopes.get(currentTargetPoint - 1) * points.get(currentTargetPoint - 1).x - 2 * slopes.get(currentTargetPoint - 1) * normalCurrentY - 2 * normalCurrentX;
            double c = points.get(currentTargetPoint - 1).y * points.get(currentTargetPoint - 1).y - 2 * slopes.get(currentTargetPoint - 1) * points.get(currentTargetPoint - 1).y * points.get(currentTargetPoint - 1).x + slopes.get(currentTargetPoint - 1) * slopes.get(currentTargetPoint - 1) * points.get(currentTargetPoint - 1).x * points.get(currentTargetPoint - 1).x - lookAhead * lookAhead + normalCurrentY * normalCurrentY + normalCurrentX * normalCurrentX + 2 * normalCurrentY * slopes.get(currentTargetPoint - 1) * points.get(currentTargetPoint - 1).x - 2 * normalCurrentY * points.get(currentTargetPoint - 1).y;
            double sqrt = b * b - 4 * a * c;
            double perpendicularSlope = -1 / slopes.get(currentTargetPoint - 1);
            if (sqrt >= 0){
                sqrt = Math.sqrt(sqrt);
                if (points.get(currentTargetPoint).x > normalCurrentX) {
                    x = (-b + sqrt) / (2 * a);
                    y = slopes.get(currentTargetPoint - 1) * x + points.get(currentTargetPoint - 1).y - slopes.get(currentTargetPoint - 1) * points.get(currentTargetPoint - 1).x;
                } else if (points.get(currentTargetPoint).x < normalCurrentX) {
                    x = (-b - sqrt) / (2 * a);
                    y = slopes.get(currentTargetPoint - 1) * x + points.get(currentTargetPoint - 1).y - slopes.get(currentTargetPoint - 1) * points.get(currentTargetPoint - 1).x;
                } else {
                    x = (points.get(currentTargetPoint - 1).y - slopes.get(currentTargetPoint - 1) * points.get(currentTargetPoint - 1).x + normalCurrentY - perpendicularSlope * normalCurrentX) / (perpendicularSlope - slopes.get(currentTargetPoint - 1));
                    y = perpendicularSlope * x + normalCurrentY - perpendicularSlope * normalCurrentX;
                }
            } else {
                x = (points.get(currentTargetPoint - 1).y - slopes.get(currentTargetPoint - 1) * points.get(currentTargetPoint - 1).x + normalCurrentY - perpendicularSlope * normalCurrentX) / (perpendicularSlope - slopes.get(currentTargetPoint - 1));
                y = perpendicularSlope * x + normalCurrentY - perpendicularSlope * normalCurrentX;
            }
            if (slopes.get(currentTargetPoint - 1) > 0){
                if (points.get(currentTargetPoint).x > points.get(currentTargetPoint - 1).x){
                    if (y <= perpendicularSlope * x + points.get(currentTargetPoint - 1).y - perpendicularSlope * points.get(currentTargetPoint - 1).x) {
                        x = points.get(currentTargetPoint - 1).x;
                        y = points.get(currentTargetPoint - 1).y;
                    } else if (y >= perpendicularSlope * x + points.get(currentTargetPoint).y - perpendicularSlope * points.get(currentTargetPoint).x) {
                        x = points.get(currentTargetPoint).x;
                        y = points.get(currentTargetPoint).y;
                    }
                } else {
                    if (y <= perpendicularSlope * x + points.get(currentTargetPoint).y - perpendicularSlope * points.get(currentTargetPoint).x) {
                        x = points.get(currentTargetPoint).x;
                        y = points.get(currentTargetPoint).y;
                    } else if (y >= perpendicularSlope * x + points.get(currentTargetPoint - 1).y - perpendicularSlope * points.get(currentTargetPoint - 1).x) {
                        x = points.get(currentTargetPoint - 1).x;
                        y = points.get(currentTargetPoint - 1).y;
                    }
                }
            } else {
                if (points.get(currentTargetPoint).x > points.get(currentTargetPoint - 1).x){
                    if (y >= perpendicularSlope * x + points.get(currentTargetPoint - 1).y - perpendicularSlope * points.get(currentTargetPoint - 1).x) {
                        x = points.get(currentTargetPoint - 1).x;
                        y = points.get(currentTargetPoint - 1).y;
                    } else if (y <= perpendicularSlope * x + points.get(currentTargetPoint).y - perpendicularSlope * points.get(currentTargetPoint).x) {
                        x = points.get(currentTargetPoint).x;
                        y = points.get(currentTargetPoint).y;
                    }
                } else {
                    if (y >= perpendicularSlope * x + points.get(currentTargetPoint).y - perpendicularSlope * points.get(currentTargetPoint).x) {
                        x = points.get(currentTargetPoint).x;
                        y = points.get(currentTargetPoint).y;
                    } else if (y <= perpendicularSlope * x + points.get(currentTargetPoint - 1).y - perpendicularSlope * points.get(currentTargetPoint - 1).x) {
                        x = points.get(currentTargetPoint - 1).x;
                        y = points.get(currentTargetPoint - 1).y;
                    }
                }
            }
        }
        return new VectorCartesian(x, y, 0);
    }

    public static class Waypoint {
        private double x;
        private double y;
        private boolean finalPoint;
        public Waypoint(double x, double y, boolean finalPoint) {
            this.x = x;
            this.y = y;
            this.finalPoint = finalPoint;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }

        public boolean isFinalPoint() {
            return finalPoint;
        }
    }

    public ArrayList<Waypoint> getPoints() {
        return points;
    }

    public ArrayList<Double> getSlopes(){
        return slopes;
    }

    public int getCurrentTargetPoint(){
        return currentTargetPoint;
    }

    //jason pure pursuit

    public void findIntersection(double x, double y, double x1, double y1, double x2, double y2, double lookAhead){
        isIntersection = false;
        //a = x1 b = y1 c = x2 d = y2
        double realx1 = x1 - x;
        double realx2 = x2 - x;
        double realy1 = y1 - y;
        double realy2 = y2 - y;
        double slope = (y2 - y1)/(x2 - x1);
        double discriminant = Math.pow(2*slope*y1-2*Math.pow(slope, 2)*x1, 2) - (4*Math.pow(slope, 2) + 1);
        double solutionx1 = (-(2*slope*y1 - 2*Math.pow(slope, 2)*x1) + discriminant)/(2*y2);
        double solutionx2 = (-(2*slope*y1 - 2*Math.pow(slope, 2)*x1) - discriminant)/(2*y2);
    }
}
