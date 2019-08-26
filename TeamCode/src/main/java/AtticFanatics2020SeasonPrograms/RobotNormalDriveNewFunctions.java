package AtticFanatics2020SeasonPrograms;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RobotNormalDriveNewFunctions extends LinearOpMode{

    BNO055IMU imu;

    DcMotor[] Motors;

    //Orientation angles;
    Acceleration gravity;

    ConfigureRobot Config = new ConfigureRobot();

    public void runOpMode() throws InterruptedException {
    }

    public boolean MoveEncoderTicks(double NumbCM, boolean Configured) {

        telemetry.addLine("(2) About to check for configured");
        if (!Configured) {
            telemetry.addLine("(3) About to configure");
            Config.Configure(Motors);
            Configured = true;
        }

        telemetry.addLine("(6) Configured, resetting motor encoders");
        Config.ResetMotorEncoders(Motors[1], Motors[2], Motors[3], Motors[4]);

        double HeadingAdjust = Config.angles.firstAngle;

        double TurnAmount;

        //Mess with numbers, as different circumference.
        double Ticks = 36.1275 * NumbCM;

        telemetry.addLine("(8) About to run");
        if (NumbCM > 0) {
            Motors[1].setPower(1);
            Motors[2].setPower(1);
            Motors[3].setPower(1);
            Motors[4].setPower(1);
        } else {
            Motors[1].setPower(-1);
            Motors[2].setPower(-1);
            Motors[3].setPower(-1);
            Motors[4].setPower(-1);
        }

        //The part of the next section that includes "Motors[1].getCurrentPosition" is highly experimental and prob won't work.
        while (Motors[1].isBusy() || Motors[2].isBusy() || Motors[3].isBusy() || Motors[4].isBusy()) {
            Config.telemetry.update();
            TurnAmount = Config.angles.firstAngle - HeadingAdjust;
            if (TurnAmount > .3 && Motors[1].getPower() > 0) {
                Motors[2].setPower(1);
                Motors[4].setPower(1);
                Motors[1].setPower(.9);
                Motors[3].setPower(.9);
            } else if (TurnAmount > .3 && Motors[1].getPower() < 0) {
                Motors[2].setPower(-.9);
                Motors[4].setPower(-.9);
                Motors[1].setPower(-1);
                Motors[3].setPower(-1);
            } else if (TurnAmount < -.3 && Motors[1].getPower() > 0) {
                Motors[1].setPower(1);
                Motors[3].setPower(1);
                Motors[2].setPower(.9);
                Motors[4].setPower(.9);
            } else if (TurnAmount < -.3 && Motors[1].getPower() < 0) {
                Motors[1].setPower(-.9);
                Motors[3].setPower(-.9);
                Motors[2].setPower(-1);
                Motors[4].setPower(-1);
            } else if (Motors[1].getPower() > 0 && TurnAmount < .2 && TurnAmount > -.2) {
                Motors[1].setPower(1);
                Motors[2].setPower(1);
                Motors[3].setPower(1);
                Motors[4].setPower(1);
            } else if (Motors[1].getPower() < 0 && TurnAmount < .2 && TurnAmount > -.2) {
                Motors[1].setPower(-1);
                Motors[2].setPower(-1);
                Motors[3].setPower(-1);
                Motors[4].setPower(-1);
            }

            //This is the experimental part. What it should do is check if a motor has gone the intended distance, then stop. After this, if 1+ motor(s) still needs to move farther, the stopped motor will move only enough to keep the robot straight until all motors are at the right position.
            for (int Counter = 1; Counter <= 4; Counter++) {
                if (Motors[Counter].getCurrentPosition() >= Ticks)
                    Motors[Counter].setPower(0);
            }
        }

        telemetry.addLine("(8) Robot should have moved, then stopped");

        return Configured;
    }

    public boolean TurnEncoderTicks(double Degrees, boolean Configured) //TURNING RIGHT IS POSITIVE!!! Also this is NOT using IMU, test both to compare.
    {
        if (!Configured)
        {
            Config.Configure(Motors);
            Configured = true;
        }

        double Ticks = Degrees * 19.8;

        if (Degrees < 0)
        {
            Motors[1].setPower(-1);
            Motors[2].setPower(1);
            Motors[3].setPower(-1);
            Motors[4].setPower(1);

            while (Motors[1].isBusy() || Motors[2].isBusy() || Motors[3].isBusy() || Motors[4].isBusy())
            {
                for(int Counter = 1; Counter <= 4; Counter++)
                {
                    Ticks = -Ticks;
                    if (Motors[Counter].getCurrentPosition() >= Ticks)
                        Motors[Counter].setPower(0);
                }
            }
        }
        else {

            Motors[1].setPower(1);
            Motors[2].setPower(-1);
            Motors[3].setPower(1);
            Motors[4].setPower(-1);

            while (Motors[1].isBusy() || Motors[2].isBusy() || Motors[3].isBusy() || Motors[4].isBusy())
            {
                for(int Counter = 1; Counter <= 4; Counter++)
                {
                    if (Motors[Counter].getCurrentPosition() >= Ticks)
                        Motors[Counter].setPower(0);
                    Ticks = -Ticks;
                }
            }
        }

        return Configured;
    }
}
