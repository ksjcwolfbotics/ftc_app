package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.MasterFunctions.inchesToTicks;


@Autonomous(name="AutonomousEncoderOpMode", group = "Autonomous")
public class AutonomousEncoderOpMode extends LinearOpMode {

    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor lift;

    Servo claw;

    double tiles = 22.75;
    double drivePower = 0.8;
    double liftPower = 0.75;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Instantiate out all motors

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

        claw.setPosition(0);

        // Reset Encoders and Set Wheels to use Encoders
        resetEncoders();

        //------------------------------------------------------------------------------------------
        waitForStart();

        //Instructions After Init. and Start.

        // Move Forward 2 Tiles

        // Set target position to 2 tiles using inchesToTicks function
        leftWheel.setTargetPosition(inchesToTicks(2*tiles));
        rightWheel.setTargetPosition(inchesToTicks(2*tiles));

        // Move to set 2 tile position.
        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait command while both
        while(leftWheel.isBusy() || rightWheel.isBusy())
        {
            sleep(0);
            //Wait until position reached
        }

        // Reset Encoders and Set Wheels to use Encoders
        resetEncoders();

        //------------------------------------------------------------------------------------------

        // Lift goes up, Claw opens, Lift goes down, Claw closes, Lift goes up

        // Lift goes up for 1 sec and stops.
        lift.setPower(liftPower);
        sleep(1000);
        lift.setPower(0);

        //Open claw.
        claw.setPosition(0.5);

        // Lift goes down for 1 second and stops.
        lift.setPower(-liftPower);
        sleep(1000);
        lift.setPower(0);

        // Close claw.
        claw.setPosition(0);

        // Lift goes up for 0.5 sec and stops.
        lift.setPower(liftPower);
        sleep(500);
        lift.setPower(0);

        //------------------------------------------------------------------------------------------

        // Move 1 tile back.

        // Set Up Target Position to 1 tile backwards.

        leftWheel.setTargetPosition(inchesToTicks(-1*tiles));
        rightWheel.setTargetPosition(inchesToTicks(-1*tiles));

        // Run To 1 tile back position.
        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait command while both
        while(leftWheel.isBusy() || rightWheel.isBusy())
        {
            sleep(0);
            //Wait until position reached
        }

        //------------------------------------------------------------------------------------------

        // Reset Encoders and Set Wheels to use Encoders
        resetEncoders();

        //------------------------------------------------------------------------------------------

        // Turn left.
        leftWheel.setTargetPosition(-500);
        rightWheel.setTargetPosition(1000);

        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait command while both
        while(leftWheel.isBusy() || rightWheel.isBusy())
        {
            sleep(0);
            //Wait until position reached
        }

        //------------------------------------------------------------------------------------------

        // Reset Encoders and Set Wheels to use Encoders
        resetEncoders();

        //------------------------------------------------------------------------------------------

        // Move forward 2 tiles.

        // Set Up Target Position to 2 tiles forward..
        leftWheel.setTargetPosition(inchesToTicks(2*tiles));
        rightWheel.setTargetPosition(inchesToTicks(2*tiles));

        // Run To 2 tile forward position.
        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait command while both
        while(leftWheel.isBusy() || rightWheel.isBusy())
        {
            sleep(0);
            //Wait until position reached
        }

        //------------------------------------------------------------------------------------------

        // Reset Encoders and Set Wheels to use Encoders
        resetEncoders();

        //------------------------------------------------------------------------------------------

        // Open Claw for 1 seconds

        claw.setPosition(0.5);
        sleep(1000);
        claw.setPosition(0);
        sleep(500);

        //------------------------------------------------------------------------------------------


        // Move 1 tile back.

        // Set Up Target Position to 1 tile backwards.
        leftWheel.setTargetPosition(inchesToTicks(-1*tiles));
        rightWheel.setTargetPosition(inchesToTicks(-1*tiles));

        // Run To 1 tile back position.
        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait command while both
        while(leftWheel.isBusy() || rightWheel.isBusy())
        {
            sleep(0);
            //Wait until position reached
        }


    }

    private void resetEncoders()
    {
        // Reset Encoders and Set Wheels to use Encoders

        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

}
