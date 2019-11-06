package org.firstinspires.ftc.teamcode;

import android.media.AudioManager;
import android.media.SoundPool;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.MasterFunctions.inchesToTicks;


@Autonomous(name="AutonomousInchesToTicksTesting", group = "Autonomous Testing")
public class AutonomousInchesToTicksTesting extends LinearOpMode {

    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor lift;

    Servo claw;

    public SoundPool beep;
    public int beepID;

    public SoundPool dance;
    public int danceID;

    double tiles = 22.75;
    double drivePower = 0.9;
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


        beep = new SoundPool(1, AudioManager.STREAM_MUSIC, 0); // PSM
        beepID = beep.load(hardwareMap.appContext, R.raw.beep, 1); // PSM

        dance = new SoundPool(1, AudioManager.STREAM_MUSIC, 0); // PSM
        danceID = dance.load(hardwareMap.appContext, R.raw.defaultdance, 1); // PSM

        beep.play(beepID,1,1,1,0,1);

        waitForStart();

        //------------------------------------------------------------------------------------------

        //Instructions After Init. and Start.

        //Beep
        sleep(1000);
        beep.play(beepID,1,1,1,0,1);
        sleep(1000);

        resetEncoders();

        leftWheel.setPower(drivePower);
        rightWheel.setPower(drivePower);

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

        resetEncoders();

        //Beep
        sleep(1000);
        beep.play(beepID,1,1,1,1,1);
        sleep(1000);

        //Default
        dance.play(danceID,1,1,1,0,1);



    }

    private void resetEncoders() {

        // Reset Encoders and Set Wheels to use Encoders

        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
