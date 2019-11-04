package org.firstinspires.ftc.teamcode;

import android.media.AudioManager;
import android.media.SoundPool;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="TeleOpControllerTankDrive", group="TeleOp")


public class TeleOpControllerTankDrive extends OpMode {

    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor lift;

    Servo claw;

    public SoundPool mySound;
    public int beepID;
    boolean leftBumper;

    double leftWheelPower;
    double rightWheelPower;
    double liftPower;
    double servoPosition;

    @Override
    public void init() {

        leftWheel = hardwareMap.dcMotor.get("leftWheel");
        rightWheel = hardwareMap.dcMotor.get("rightWheel");

        lift = hardwareMap.dcMotor.get("liftMotor");

        claw = hardwareMap.servo.get("clawServo");

        rightWheel.setDirection(DcMotor.Direction.FORWARD);
        leftWheel.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.REVERSE);

        liftPower = 0.75;

        mySound = new SoundPool(1, AudioManager.STREAM_MUSIC, 0); // PSM
        beepID = mySound.load(hardwareMap.appContext, R.raw.defaultdance, 1); // PSM

        leftBumper = false;

    }

    @Override
    public void loop() {

        // Tank Drive Using Gamepad 1 Left and Right Stick
        leftWheelPower = gamepad1.left_stick_y;
        rightWheelPower = gamepad1.right_stick_y;

        leftWheel.setPower(leftWheelPower*0.8);
        rightWheel.setPower(rightWheelPower*0.8);

        // Lift and Claw Using Gamepad 2's Left Stick and Right Stick

        liftPower = gamepad2.left_stick_y;
        servoPosition = gamepad2.right_stick_y;

        lift.setPower(liftPower);
        claw.setPosition(Math.abs(servoPosition));

        // Joke Default Dance Sound - either driver can access through left bumper
        if(gamepad1.left_trigger > 0 || gamepad2.left_trigger > 0)
        {
            leftBumper = true;
        }
        else
            {
                leftBumper = false;
            }
        if(leftBumper)
        {
            mySound.play(beepID,1,1,1,0,1);
        }


        /*
        boolean servoOnOrOff;

        servoOnOrOff = false;

        if (gamepad2.x)
        {
            servoOnOrOff = true;
        }
        else
            {
                servoOnOrOff = false;
            }

        if (servoOnOrOff == true)
        {
            claw.setPosition(0.5);
        }
        else if(servoOnOrOff == false)
        {
            claw.setPosition(0);
        }
        */

    }
}
