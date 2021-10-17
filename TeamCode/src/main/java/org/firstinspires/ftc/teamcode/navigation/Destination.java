package org.firstinspires.ftc.teamcode.navigation;

import java.util.Arrays;

/**
 * From IsaacArrayDemo 2/6/21
 */
public class Destination {

    //declare variables
    public String destName;
    private float[] coordinate;

    //constructors of all shapes and sizes.
    public Destination() {
        coordinate = new float[2];
        Arrays.fill(coordinate, 0);
    }
    public Destination(String name, float destX, float destY){
        this();
        destName = name;
        coordinate[0] = destX;
        coordinate[1] = destY;
    }


    //methods
    public String getName(){
        return destName;
    }
    public void setName(String name) {
        destName = name;
    }
    public float getX() {
        return coordinate[0];
    }
    public float getY() {
        return coordinate[1];
    }

    public void setX(float x) {
        coordinate[0] = x;
    }
    public void setY(float y) {
        coordinate[1] = y;
    }

} //END:  class Destination
