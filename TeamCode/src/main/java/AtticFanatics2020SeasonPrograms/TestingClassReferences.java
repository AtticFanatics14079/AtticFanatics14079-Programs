package AtticFanatics2020SeasonPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "TestingClassReferences")
public class TestingClassReferences extends LinearOpMode{

    boolean Configured = false;

    //The next line instantiates a supplementary class. The "Vera" variable can be any name as long as it is referenced by the same name later, but we're keepin it this way for now gang cause Vera carried this one.
    //RobotMecanum vera = new RobotMecanum();
    RobotNormalDrive jon = new RobotNormalDrive();

    @Override
    public void runOpMode() throws InterruptedException {

        //vera.SetMotorPower(.5);

        telemetry.addLine("(1) About to enter MoveEncoderTicks in RobotNormalDrive");
        Configured = jon.MoveEncoderTicks(20, Configured);

        //vera.SidewaysMovement(30, Configured);

        Configured = jon.TurnUsingIMU(80, Configured); //TURNING RIGHT IS POSITIVE!!!
    }
}