package org.firstinspires.ftc.teamcode;

public class MasterFunctions {
    
    public static int inchesToTicks(double inches) {
        return (int) ((960/(4*Math.PI)) * inches);
    }



}
