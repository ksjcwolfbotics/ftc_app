package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.MasterFunctions.inchesToTicks;


@Autonomous(name="AutonomousEncoderTurningTesting", group = "Autonomous")
public class AutonomousEncoderTurningTesting extends LinearOpMode {

    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor lift;

    Servo claw;

    double drivePower = 0.8;

    double robotLength;

    int rightWheelTargetPosition;
    int leftWheelTargetPosition;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Instantiate out all motors

        robotLength = 16;

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

        double robotLength = 0;
        rightWheelTargetPosition = inchesToTicks((Math.PI * robotLength)/2);
        leftWheelTargetPosition = inchesToTicks(0);

        //------------------------------------------------------------------------------------------
        waitForStart();

        //Instructions After Init. and Start.

        leftWheel.setPower(drivePower);
        rightWheel.setPower(drivePower);

        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(5000);

        leftWheel.setPower(0);
        rightWheel.setPower(0);


        telemetry.addLine("Done.");
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
