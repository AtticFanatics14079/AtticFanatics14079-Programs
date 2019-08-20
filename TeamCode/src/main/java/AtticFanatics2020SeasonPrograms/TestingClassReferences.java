package AtticFanatics2020SeasonPrograms;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TestingClassReferences extends LinearOpMode{

    private DcMotor Motor1 = null;
    private DcMotor Motor2 = null;
    private DcMotor Motor3 = null;
    private DcMotor Motor4 = null;
    public static boolean Configured = false;

    @Override
    public void runOpMode() throws InterruptedException {

        //The next line instantiates a supplementary class. The "Vera" variable can be any name as long as it is referenced by the same name later, but we're keepin it this way for now gang cause Vera carried this one.
        RobotMecanum vera = new RobotMecanum();

        vera.SetMotorPower(Motor1, .5);
    }
}