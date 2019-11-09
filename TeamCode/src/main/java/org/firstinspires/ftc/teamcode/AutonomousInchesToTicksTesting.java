package org.firstinspires.ftc.teamcode;

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
    double drivePower = 0.5;
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
        rightWheel.setDirection(DcMotor.Direction.REVERSE);
        leftWheel.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);


        /*
        beep = new SoundPool(1, AudioManager.STREAM_MUSIC, 0); // PSM
        beepID = beep.load(hardwareMap.appContext, R.raw.beep, 1); // PSM

        dance = new SoundPool(1, AudioManager.STREAM_MUSIC, 0); // PSM
        danceID = dance.load(hardwareMap.appContext, R.raw.defaultdance, 1); // PSM

        beep.play(beepID,1,1,1,0,1);



         */
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        waitForStart();

        //------------------------------------------------------------------------------------------

        //Instructions After Init. and Start.

        moveForward(inchesToTicks(5), inchesToTicks(5), 0.1);

    }

    private void resetEncoders() {

        // Reset Encoders and Set Wheels to use Encoders

        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveForward( int leftTargetPosition, int rightTargetPosition, double drivePower) {
        // make sure to use positive values to move forward since set power is negative.

        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftWheel.setTargetPosition(leftTargetPosition);
        rightWheel.setTargetPosition(rightTargetPosition);

        telemetry.addData("Left Wheel Target Position", leftWheel.getCurrentPosition());
        telemetry.addData("Right Wheel Target Position", rightWheel.getCurrentPosition());
        telemetry.update();

        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftWheel.setPower(-drivePower);
        rightWheel.setPower(-drivePower);

        while (rightWheel.isBusy() || leftWheel.isBusy()) {
            telemetry.addData("Left Wheel Target Position", leftWheel.getCurrentPosition());
            telemetry.addData("Right Wheel Target Position", rightWheel.getCurrentPosition());
            telemetry.update();
        }

        leftWheel.setPower(0);
        rightWheel.setPower(0);


    }
}
