package org.firstinspires.ftc.teamcode.navigation;

/**
 * From IsaacArrayDemo 2/6/21
 */
public class MyPath{

    //declare variables
    public double originX = 0;
    public double originY = 0;
    public double targetX = 0;
    public double targetY = 0;
    public double initHeading = 0;
    public double newHeading;
    public double oppHeading;
    public String originName, targetName, s;
    public double deltaX;
    public double deltaY;
    public double heading;
    public double finalHeading;
    public double leastRot;
    public double direction;
    public double distance;
    public double rot_L_fwd, rot_R_fwd, rot_L_rev, rot_R_rev;

    //constructor
    public MyPath() { }
    public MyPath(Destination origin, Destination target) {
        this(origin, target, 0);
    }
    public MyPath(Destination origin, Destination target, double initHeading) {
        this(origin.getX(), origin.getY(), target.getX(), target.getY(), initHeading);
    }
    public MyPath(double originX,double originY,double targetX,double targetY,double initHeading)  {
        this.originX = originX;
        this.originY = originY;
        this.targetX = targetX;
        this.targetY = targetY;
        this.initHeading = initHeading;
        calcMyPath(originX, originY, targetX, targetY, initHeading);
    }
    // methods
    public double calcMyPath(Destination origin, Destination target, double initHeading) {
        return calcMyPath(origin.getX(), origin.getY(), target.getX(), target.getY(), initHeading);
    }

    public double calcMyPath(double originX, double originY, double targetX, double targetY, double initHeading){

        deltaX = targetX - originX;
        deltaY = targetY - originY;
        newHeading = Math.atan2(deltaY,deltaX);
        newHeading = java.lang.Math.toDegrees(newHeading) + 90;
        if(newHeading < 180){
            oppHeading = newHeading + 180;
        }
        else {
            oppHeading = newHeading - 180;
        }


        distance = Math.sqrt(deltaX*deltaX + deltaY*deltaY);
        rot_L_fwd = (360 - initHeading + newHeading) % 360;
        rot_R_fwd = -(360 - newHeading + initHeading) % 360;
        rot_L_rev = (360 - initHeading + oppHeading) % 360;
        rot_R_rev = -(360 - oppHeading + initHeading) % 360;
        leastRot = rot_L_fwd;
        direction = 1;
        if (Math.abs(rot_R_fwd)<Math.abs(leastRot)){
            leastRot = rot_R_fwd;
        }
        if(Math.abs(rot_L_rev)<Math.abs(leastRot)){
            leastRot = rot_L_rev;
            direction = -1;
        }
        if (Math.abs(rot_R_rev)<Math.abs(leastRot)){
            leastRot = rot_R_rev;
            direction = -1;
        }
        finalHeading = initHeading + leastRot;
        return 1;
    }
    public double getHeading(double originX, double originY, double targetX, double targetY, double initHeading){
        calcMyPath(originX,originY,targetX,targetY,initHeading);
        return finalHeading;
    }
    public double getDistance(double originX, double originY, double targetX, double targetY, double initHeading){
        calcMyPath(originX,originY,targetX,targetY,initHeading);
        return distance;
    }
    public double getRotation(double originX, double originY, double targetX, double targetY, double initHeading){
        calcMyPath(originX,originY,targetX,targetY,initHeading);
        return leastRot;
    }
    public double getDirection(double originX, double originY, double targetX, double targetY, double initHeading){
        calcMyPath(originX,originY,targetX,targetY,initHeading);
        return direction;
    }
} // END:  class myPath

