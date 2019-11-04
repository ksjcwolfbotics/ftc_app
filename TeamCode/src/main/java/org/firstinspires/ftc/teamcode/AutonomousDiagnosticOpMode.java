package org.firstinspires.ftc.teamcode;

import android.media.AudioManager;
import android.media.SoundPool;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.MasterFunctions.inchesToTicks;


@Autonomous(name="AutonomousDiagnosticOpMode", group = "Autonomous")
public class AutonomousDiagnosticOpMode extends LinearOpMode {

    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor lift;

    Servo claw;

    public SoundPool mySound;

    public int beepID;

    double tiles = 22.75;
    double drivePower = 0.8;
    double liftPower = 0.75;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Instantiate out all motors/servos

        leftWheel = hardwareMap.dcMotor.get("leftWheel");
        rightWheel = hardwareMap.dcMotor.get("rightWheel");

        lift = hardwareMap.dcMotor.get("liftMotor");

        claw = hardwareMap.servo.get("clawServo");

        // Initialize Motors and Servos
        rightWheel.setDirection(DcMotor.Direction.FORWARD);
        leftWheel.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.REVERSE);

        leftWheel.setPower(drivePower);
        rightWheel.setPower(drivePower);

        mySound = new SoundPool(1, AudioManager.STREAM_MUSIC, 0); // PSM
        beepID = mySound.load(hardwareMap.appContext, R.raw.beep, 1); // PSM

        mySound.play(beepID,1,1,1,0,1);

        waitForStart();

        //------------------------------------------------------------------------------------------

        //Instructions After Init. and Start.

        // Move for a second
        leftWheel.setPower(drivePower);
        rightWheel.setPower(drivePower);
        sleep(1000);
        leftWheel.setPower(0);
        rightWheel.setPower(0);

        //Beep boop
        sleep(1000);
        mySound.play(beepID,1,1,1,0,1);
        sleep(1000);

        //Lift for 1 second
        lift.setPower(liftPower);
        sleep(1000);
        lift.setPower(0);

        //Lift for 1 second
        lift.setPower(-liftPower);
        sleep(1000);
        lift.setPower(0);

        //Beep boop
        sleep(1000);
        mySound.play(beepID,1,1,1,0,1);
        sleep(1000);

        //Opens claw for 1 sec
        claw.setPosition(1);
        sleep(1000);
        claw.setPosition(0);

        //Beep
        sleep(1000);
        mySound.play(beepID,1,1,1,0,1);
        sleep(1000);

        resetEncoders();

        //Move 1 inch
        leftWheel.setTargetPosition(inchesToTicks(1));
        rightWheel.setTargetPosition(inchesToTicks(1));

        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait command while both
        while(leftWheel.isBusy() || rightWheel.isBusy())
        {
            sleep(0);
            //Wait until position reached
        }

        //Beep
        sleep(1000);
        mySound.play(beepID,1,1,1,0,1);
        sleep(1000);

        //Move 1 tile
        leftWheel.setTargetPosition(inchesToTicks(tiles));
        rightWheel.setTargetPosition(inchesToTicks(tiles));

        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait command while both
        while(leftWheel.isBusy() || rightWheel.isBusy())
        {
            sleep(0);
            //Wait until position reached
        }

        //Beep
        sleep(1000);
        mySound.play(beepID,1,1,1,10,1);
        sleep(1000);



    }

    private void resetEncoders() {

        // Reset Encoders and Set Wheels to use Encoders

        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    }
