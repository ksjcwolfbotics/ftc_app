package org.firstinspires.ftc.teamcode;


import android.media.AudioManager;
import android.media.SoundPool;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.MasterFunctions.inchesToTicks;

@Autonomous(name="AutoRobotPresentation", group = "Autonomous Testing")
public class JudgePresentation extends LinearOpMode {

    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor lift;

    Servo claw;

    double tiles = 22.75;
    double drivePower = 0.5;
    double liftPower = -0.75;

    int leftWheelTargetPosition;
    int rightWheelTargetPosition;

    double robotLength = 15.625;

    //Define all noises
    // Sound variables
    public SoundPool intro;
    public int introID;

    public SoundPool claws;
    public int clawID;

    public SoundPool lifts;
    public int liftID;

    public SoundPool finish;
    public int finishID;


    @Override
    public void runOpMode() throws InterruptedException {

        // Instantiate out all motors

        leftWheel = hardwareMap.dcMotor.get("leftWheel");
        rightWheel = hardwareMap.dcMotor.get("rightWheel");

        lift = hardwareMap.dcMotor.get("liftMotor");

        claw = hardwareMap.servo.get("clawServo");

        // Initialize Motors and Servos
        rightWheel.setDirection(DcMotor.Direction.REVERSE);
        leftWheel.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.REVERSE);

        claw.setPosition(0);

        leftWheelTargetPosition = inchesToTicks(0);
        rightWheelTargetPosition = inchesToTicks((Math.PI * robotLength)/2);

        // Instantiate noise
        intro = new SoundPool(1, AudioManager.STREAM_MUSIC, 0); // PSM
        introID = intro.load(hardwareMap.appContext, R.raw.intro, 1); // PSM

        claws = new SoundPool(1, AudioManager.STREAM_MUSIC, 0); // PSM
        clawID = claws.load(hardwareMap.appContext, R.raw.claws, 1); // PSM

        lifts = new SoundPool(1, AudioManager.STREAM_MUSIC, 0); // PSM
        liftID = lifts.load(hardwareMap.appContext, R.raw.lift, 1); // PSM

        finish = new SoundPool(1, AudioManager.STREAM_MUSIC, 0); // PSM
        finishID = finish.load(hardwareMap.appContext, R.raw.finish, 1); // PSM


        //------------------------------------------------------------------------

        waitForStart();

        intro.play(introID,1,1,1,0,1);

        sleep(3000);

        claws.play(clawID,1,1,1,0,1);

        claw.setPosition(0.5);
        sleep(1500);
        claw.setPosition(0);

        sleep(2500);

        lifts.play(liftID,1,1,1,0,1);

        lift.setPower(1);
        sleep(700);
        lift.setPower(0);

        sleep(1000);

        lift.setPower(-1);
        sleep(700);
        lift.setPower(0);

        sleep(3000);

        finish.play(finishID,1,1,1,0,1);

        sleep(3000);

    }
}
