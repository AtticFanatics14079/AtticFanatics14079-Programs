package AtticFanatics2020SeasonPrograms;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

public class RobotNormalDrive extends LinearOpMode{

    BNO055IMU imu;

    DcMotor[] Motors;

    Orientation angles;
    Acceleration gravity;

    ConfigureRobot Config = new ConfigureRobot();

    public void runOpMode() throws InterruptedException {
    }

    public boolean MoveEncoderTicks(double NumbCM, boolean Configured) {

        telemetry.addLine("(2) About to check for configured");
        if (!Configured)
        {
            telemetry.addLine("(3) About to configure");
            Motors = Config.Configure(Motors);
            Configured = true;
        }

        telemetry.addLine("(6) Configured, resetting motor encoders");
        Config.ResetMotorEncoders(Motors[1], Motors[2], Motors[3], Motors[4]);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double HeadingAdjust = angles.firstAngle;

        double TurnAmount;

        telemetry.addLine("(7) About to set motors to Run_To_Position");
        Motors[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motors[2].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motors[3].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motors[4].setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Mess with numbers, as different circumference.
        double Ticks = 36.1275 * NumbCM;

        Motors[1].setTargetPosition((int) Ticks);
        Motors[2].setTargetPosition((int) Ticks);
        Motors[3].setTargetPosition((int) Ticks);
        Motors[4].setTargetPosition((int) Ticks);

        telemetry.addLine("(8) About to run");
        if (Motors[1].getTargetPosition() > 0) {
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

        while (Motors[1].isBusy() || Motors[2].isBusy() || Motors[3].isBusy() || Motors[4].isBusy()) {
            telemetry.update();
            TurnAmount = angles.firstAngle - HeadingAdjust;
            telemetry.addLine("This is about to use IMU, I doubt the IMU will work");
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
            } else if (Motors[1].getPower() > 0) {
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
        }

        Motors[1].setPower(0);
        Motors[2].setPower(0);
        Motors[3].setPower(0);
        Motors[4].setPower(0);

        telemetry.addLine("(8) Robot should have moved");

        return Configured;
    }

    public boolean TurnUsingIMU(int Degrees, boolean Configured) //DO NOT TURN CLOSE TO A 180; INSTEAD JUST TURN UP TO 90 AND GO SIDEWAYS OR BACKWARDS
    {
        //TURNING RIGHT IS POSITIVE!!!

        if (!Configured)
        {
            Motors = Config.Configure(Motors);
            Configured = true;
        }

        Config.ResetMotorEncoders(Motors[1], Motors[2], Motors[3], Motors[4]);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double HeadingAdjust = angles.firstAngle;

        //Mess with numbers, as different circumference.
        double Ticks = Degrees * 19.8;

        Motors[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motors[2].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motors[3].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motors[4].setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Motors[2].setTargetPosition((int) (-1 * Ticks));
        Motors[1].setTargetPosition((int) Ticks);
        Motors[4].setTargetPosition((int) (-1 * Ticks));
        Motors[3].setTargetPosition((int) Ticks);

        double TurnAmount;

        if (Motors[1].getTargetPosition() < 0) {
            Motors[1].setPower(-1);
            Motors[2].setPower(1);
            Motors[3].setPower(-1);
            Motors[4].setPower(1);
        } else {
            Motors[1].setPower(1);
            Motors[2].setPower(-1);
            Motors[3].setPower(1);
            Motors[4].setPower(-1);
        }

        while (Motors[1].isBusy() || Motors[2].isBusy() || Motors[3].isBusy() || Motors[4].isBusy()) {
            telemetry.update();
        }

        while (opModeIsActive()) {

            telemetry.update();

            TurnAmount = angles.firstAngle - HeadingAdjust;

            if ((Degrees - TurnAmount > -1) && (Degrees - TurnAmount < 1)) {

                Motors[1].setPower(0);
                Motors[2].setPower(0);
                Motors[3].setPower(0);
                Motors[4].setPower(0);

                break;

            }
            else if ((Degrees - TurnAmount >= 1) && (TurnAmount >= 0)) {

                Config.ResetMotorEncoders(Motors[1], Motors[2], Motors[3], Motors[4]);

                Motors[2].setTargetPosition((int) (-.3 * (Degrees - TurnAmount)));
                Motors[4].setTargetPosition((int) (-.3 * (Degrees - TurnAmount)));
                Motors[1].setTargetPosition((int) (.3 * (Degrees - TurnAmount)));
                Motors[3].setTargetPosition((int) (.3 * (Degrees - TurnAmount)));

                Motors[2].setPower(-.2);
                Motors[1].setPower(.2);
                Motors[4].setPower(-.2);
                Motors[3].setPower(.2);
            }
            else if (Degrees - TurnAmount <= -1) {

                Config.ResetMotorEncoders(Motors[1], Motors[2], Motors[3], Motors[4]);

                Motors[2].setTargetPosition((int) (.3 * (Degrees - TurnAmount)));
                Motors[4].setTargetPosition((int) (.3 * (Degrees - TurnAmount)));
                Motors[1].setTargetPosition((int) (-.3 * (Degrees - TurnAmount)));
                Motors[3].setTargetPosition((int) (-.3 * (Degrees - TurnAmount)));

                Motors[2].setPower(.2);
                Motors[1].setPower(-.2);
                Motors[4].setPower(.2);
                Motors[3].setPower(-.2);
            }
            else if (-1 * Degrees + TurnAmount <= -1) {

                Config.ResetMotorEncoders(Motors[1], Motors[2], Motors[3], Motors[4]);

                Motors[2].setTargetPosition((int) (-.3 * (Degrees - TurnAmount))); //Numbers off, fix using math.
                Motors[4].setTargetPosition((int) (-.3 * (Degrees - TurnAmount)));
                Motors[1].setTargetPosition((int) (.3 * (Degrees - TurnAmount)));
                Motors[3].setTargetPosition((int) (.3 * (Degrees - TurnAmount)));

                Motors[2].setPower(-.2);
                Motors[1].setPower(.2);
                Motors[4].setPower(-.2);
                Motors[3].setPower(.2);
            }
            else {

                Config.ResetMotorEncoders(Motors[1], Motors[1], Motors[1], Motors[1]);

                Motors[2].setTargetPosition((int) (.3 * (Degrees - TurnAmount)));
                Motors[4].setTargetPosition((int) (.3 * (Degrees - TurnAmount)));
                Motors[1].setTargetPosition((int) (-.3 * (Degrees - TurnAmount)));
                Motors[3].setTargetPosition((int) (-.3 * (Degrees - TurnAmount)));

                Motors[2].setPower(.2);
                Motors[1].setPower(-.2);
                Motors[4].setPower(.2);
                Motors[3].setPower(-.2);
            }
        }

        return Configured;
    }
}
