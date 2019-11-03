import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="MoveForwardOpMode", group = "TeleOp")
// @Disabled


public class MoveForwardOpMode extends OpMode {

    DcMotor leftWheel;
    DcMotor rightWheel;
    double drivePower = 0.5;

    @Override
    public void init()
    {
        leftWheel = hardwareMap.dcMotor.get("leftWheel");
        rightWheel = hardwareMap.dcMotor.get("rightWheel");

        //Will tell motor to go in reverse b/c motor moved 180 to get on both sides of robot
        rightWheel.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void start(){}

    @Override
    public void loop()
    {
        // Power can be any value from 0 - 1
        leftWheel.setPower(drivePower);
        // rightWheel can be set negative because one must be turned 180 to get it on both sides.
        // However, the setDirection method will be used.
        rightWheel.setPower(drivePower);

    }

    @Override
    public void stop(){}
}
