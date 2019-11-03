package org.firstinspires.ftc.teamcode.TutorialCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="AutonomousOpMode", group = "AutoMode")
public class AutonomousOpMode extends LinearOpMode {

    DcMotor leftWheel;
    DcMotor rightWheel;

    Servo claw;

    double leftWheelPower;
    double rightWheelPower;

    @Override
    public void runOpMode() throws InterruptedException {

        leftWheel = hardwareMap.dcMotor.get("leftWheel");
        rightWheel = hardwareMap.dcMotor.get("rightWheel");

        claw = hardwareMap.servo.get("clawServo");

        leftWheel.setDirection(DcMotor.Direction.REVERSE);

        claw.setPosition(0);

        waitForStart();

        // Move Forward
        leftWheel.setPower(1);
        rightWheel.setPower(1);

        //Wait for 1 sec.
        sleep(1000);

        //Turn Off Motors
        leftWheel.setPower(0);
        rightWheel.setPower(0);

        //Turn Right
        leftWheel.setPower(0.5);
        rightWheel.setPower(-0.5);

        sleep(700);

        //Turn Off Motors
        leftWheel.setPower(0);
        rightWheel.setPower(0);

        //Open Claw
        claw.setPosition(1);

        //Wait for 1 sec
        sleep(1000);

        //Close claw.
        claw.setPosition(0);


    }
}
