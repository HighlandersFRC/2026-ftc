package org.firstinspires.ftc.teamcode.Commands.Pathing.PurePursuit;

public class PathPoint {
    double x;
    double y;
    double w;
    double curvature;
    double distance;
    public PathPoint() {

    }
    public PathPoint(double x, double y, double w, double curvature, double distance) {
        this.x = x;
        this.y = y;
        this.w = w;
        this.curvature = curvature;
        this.distance = distance;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getW() {
        return w;
    }

    public double getCurvature() {
        return curvature;
    }

    public double getDistance() {
        return distance;
    }

    public void setNewX(double newX) {
        this.x = newX;
    }

    public void setNewY(double newY) {
        this.y = newY;
    }

    public void setNewW(double newW) {
        this.w = newW;
    }

    public void setNewDistance(double newDistance) {
        this.distance = newDistance;
    }

    public void setNewCurvature(double newCurvature) {
        this.curvature = newCurvature;
    }

}
