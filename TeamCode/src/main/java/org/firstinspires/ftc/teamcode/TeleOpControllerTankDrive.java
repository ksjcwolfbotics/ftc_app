package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="TeleOpControllerTankDrive", group="TeleOp")


public class TeleOpControllerTankDrive extends OpMode {

    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor lift;

    Servo claw;

    double leftWheelPower;
    double rightWheelPower;
    double liftPower;
    double servoPosition;

    @Override
    public void init() {

        leftWheel = hardwareMap.dcMotor.get("leftWheel");
        rightWheel = hardwareMap.dcMotor.get("rightWheel");

        lift = hardwareMap.dcMotor.get("liftMotor");

        claw = hardwareMap.servo.get("clawServo");

        rightWheel.setDirection(DcMotor.Direction.FORWARD);
        leftWheel.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.REVERSE);

        liftPower = 0.75;

    }

    @Override
    public void loop() {

        // Tank Drive Using Gamepad 1 Left and Right Stick
        leftWheelPower = gamepad1.left_stick_y;
        rightWheelPower = gamepad1.right_stick_y;

        leftWheel.setPower(leftWheelPower);
        rightWheel.setPower(rightWheelPower);

        // Lift and Claw Using Gamepad 2's Left Stick and Right Stick

        liftPower = gamepad2.left_stick_y;
        servoPosition = gamepad2.right_stick_y;

        lift.setPower(liftPower);
        claw.setPosition(Math.abs(servoPosition));

    }
}
