package AtticFanatics2020SeasonPrograms;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TestingClassReferences extends LinearOpMode{

    private DcMotor Motor1 = null;
    boolean Configured = false;

    @Override
    public void runOpMode() throws InterruptedException {

        //The next line instantiates a supplementary class. The "Vera" variable can be any name as long as it is referenced by the same name later, but we're keepin it this way for now gang cause Vera carried this one.
        RobotMecanum vera = new RobotMecanum();

        vera.SetMotorPower(.5);

        vera.MoveEncoderTicks(20, Configured);

        vera.SidewaysMovement(30, Configured);

        vera.TurnUsingIMU(80, Configured);
    }
}