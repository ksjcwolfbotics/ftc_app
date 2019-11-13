package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.MasterFunctions.inchesToTicks;


@Autonomous(name="Move Forward 2 Auto Bracket Sources - Wait 20 sec, move", group = "Autonomous Testing")
public class MoveForward2AutonomousRealSauceLittleBracketSources extends LinearOpMode {

    //WolfboticsHardwareMap robot   = new WolfboticsHardwareMap();

    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor lift;

    Servo claw;

    double tiles = 22.75;
    double drivePower = 0.3;
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

        waitForStart();

        //------------------------------------------------------------------------------------------

        //Instructions After Init. and Start.

        sleep(20000);

        moveForward(inchesToTicks(5), inchesToTicks(5), 0.2);


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


    private void resetEncoders() {

        // Reset Encoders and Set Wheels to use Encoders

        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
/*
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftWheel.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightWheel.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftWheel.setTargetPosition(newLeftTarget);
            robot.rightWheel.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftWheel.setPower(Math.abs(speed));
            robot.rightWheel.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftWheel.isBusy() && robot.rightWheel.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftWheel.getCurrentPosition(),
                        robot.rightWheel.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftWheel.setPower(0);
            robot.rightWheel.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

 */
}
