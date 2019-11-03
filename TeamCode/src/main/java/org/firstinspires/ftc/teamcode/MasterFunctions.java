package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.lang.Math;

public class MasterFunctions {

    DcMotor leftWheel;
    DcMotor rightWheel;

    public static int inchesToTicks(double inches) {
        return (int) ((960/(4*Math.PI)) * inches);
    }



}
