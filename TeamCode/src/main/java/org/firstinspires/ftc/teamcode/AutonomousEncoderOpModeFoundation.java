package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.MasterFunctions.inchesToTicks;

@Autonomous(name="AutonomousEncoderOpModeFoundation", group = "Autonomous")
public class AutonomousEncoderOpModeFoundation extends LinearOpMode {

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

        claw.setPosition(0.5);

        leftWheelTargetPosition = inchesToTicks(0);
        rightWheelTargetPosition = inchesToTicks((Math.PI * robotLength)/2);

        waitForStart();

        lift.setPower(1);
        sleep(800);
        lift.setPower(0);

        moveForward(inchesToTicks(1.5*tiles), inchesToTicks(1.5*tiles), 0.1);

        lift.setPower(-1);
        sleep(800);
        lift.setPower(0);

        moveForward(inchesToTicks(-1.5*tiles), inchesToTicks(-1.5*tiles), 0.3);





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


