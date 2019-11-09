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
    double drivePower = 0.9;
    double liftPower = -0.75;

    int leftWheelTargetPosition;
    int rightWheelTargetPosition;

    double robotLength = 15.625;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Instantiate out all motors

        leftWheel = hardwareMap.dcMotor.get("leftWheel");
        rightWheel = hardwareMap.dcMotor.get("rightWheel");

        lift = hardwareMap.dcMotor.get("liftMotor");

        claw = hardwareMap.servo.get("clawServo");

        // Initialize Motors and Servos
        rightWheel.setDirection(DcMotor.Direction.REVERSE);
        leftWheel.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);

        claw.setPosition(0);

        leftWheelTargetPosition = inchesToTicks((Math.PI * robotLength)/2);
        rightWheelTargetPosition = inchesToTicks(0);

        // Reset Encoders and Set Wheels to use Encoders
        resetEncoders();

        //------------------------------------------------------------------------------------------
        //Instructions After Init. and Start.
        waitForStart();

        //Open claw.
        claw.setPosition(0.5);

        // Move Forward 1.25 tiles

        moveForward(inchesToTicks(1.25*tiles), inchesToTicks(1.25*tiles), 0.5);

        //------------------------------------------------------------------------------------------

        //  Claw opens, Lift goes down, Claw closes, Lift goes up

        // Close claw.
        claw.setPosition(0);

        //------------------------------------------------------------------------------------------

        // Move 1 tile back.

        // Set Up Target Position to 1 tile backwards.

        // DO WE NEED TO MAKE DRIVE POWER NEGATIVE?

        moveForward(inchesToTicks(-1*tiles), inchesToTicks(-1*tiles), 0.5);

        //------------------------------------------------------------------------------------------

        // Turn right.
        moveForward(leftWheelTargetPosition, rightWheelTargetPosition, 0.5);

        //------------------------------------------------------------------------------------------

        // Move forward 1.5 tiles.

        moveForward(inchesToTicks(1.5*tiles), inchesToTicks(1.5*tiles), 0.5);

        //------------------------------------------------------------------------------------------

        // Open Claw

        claw.setPosition(0.5);

        //------------------------------------------------------------------------------------------

        // Move 1.25 tiles back.

        moveForward(inchesToTicks(-1.5*tiles), inchesToTicks(-1.5*tiles), 0.5);

        //turn left (inverse the right and left wheel targets)
        moveForward(rightWheelTargetPosition, leftWheelTargetPosition, 0.5);

        //move forward 1
        moveForward(inchesToTicks(0.5*tiles), inchesToTicks(0.5*tiles), 0.5);

        // close claw

        claw.setPosition(0);

        // move back 1 tile

        moveForward(inchesToTicks(-1*tiles), inchesToTicks(-1*tiles), 0.5);

        // Turn right.
        moveForward(leftWheelTargetPosition, rightWheelTargetPosition, 0.5);

        //move forward 1
        moveForward(inchesToTicks(1*tiles), inchesToTicks(1*tiles), 0.5);

        // open claw

        claw.setPosition(0.5);

        // move back 0.5 tiles

        moveForward(inchesToTicks(-0.75*tiles), inchesToTicks(-0.75*tiles), 0.5);

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

    private void resetEncoders()
    {
        // Reset Encoders and Set Wheels to use Encoders

        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



    }

}
