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

    DcMotor Motor1;
    DcMotor Motor2;
    DcMotor Motor3;
    DcMotor Motor4;
    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    ConfigureRobot Config = new ConfigureRobot();

    public void runOpMode() throws InterruptedException {
    }

    public void MoveEncoderTicks(double NumbCM, boolean Configured) {

        if (!Configured)
        {
            Config.Configure(Motor1, Motor2, Motor3, Motor4);
            Configured = true;
        }

        Config.ResetMotorEncoders(Motor1, Motor2, Motor3, Motor4);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double HeadingAdjust = angles.firstAngle;

        double TurnAmount;

        Motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Mess with numbers, as different circumference.
        double Ticks = 36.1275 * NumbCM;

        Motor1.setTargetPosition((int) Ticks);
        Motor2.setTargetPosition((int) Ticks);
        Motor3.setTargetPosition((int) Ticks);
        Motor4.setTargetPosition((int) Ticks);

        if (Motor1.getTargetPosition() > 0) {
            Motor1.setPower(1);
            Motor2.setPower(1);
            Motor3.setPower(1);
            Motor4.setPower(1);
        } else {
            Motor1.setPower(-1);
            Motor2.setPower(-1);
            Motor3.setPower(-1);
            Motor4.setPower(-1);
        }

        while (Motor1.isBusy() || Motor2.isBusy() || Motor3.isBusy() || Motor4.isBusy()) {
            telemetry.update();
            TurnAmount = angles.firstAngle - HeadingAdjust;
            if (TurnAmount > .3 && Motor1.getPower() > 0) {
                Motor2.setPower(1);
                Motor4.setPower(1);
                Motor1.setPower(.9);
                Motor3.setPower(.9);
            } else if (TurnAmount > .3 && Motor1.getPower() < 0) {
                Motor2.setPower(-.9);
                Motor4.setPower(-.9);
                Motor1.setPower(-1);
                Motor3.setPower(-1);
            } else if (TurnAmount < -.3 && Motor1.getPower() > 0) {
                Motor1.setPower(1);
                Motor3.setPower(1);
                Motor2.setPower(.9);
                Motor4.setPower(.9);
            } else if (TurnAmount < -.3 && Motor1.getPower() < 0) {
                Motor1.setPower(-.9);
                Motor3.setPower(-.9);
                Motor2.setPower(-1);
                Motor4.setPower(-1);
            } else if (Motor1.getPower() > 0) {
                Motor1.setPower(1);
                Motor2.setPower(1);
                Motor3.setPower(1);
                Motor4.setPower(1);
            } else {
                Motor1.setPower(-1);
                Motor2.setPower(-1);
                Motor3.setPower(-1);
                Motor4.setPower(-1);
            }
        }

        Motor1.setPower(0);
        Motor2.setPower(0);
        Motor3.setPower(0);
        Motor4.setPower(0);
    }

    public void TurnUsingIMU(int Degrees, boolean Configured) //DO NOT TURN CLOSE TO A 180; INSTEAD JUST TURN UP TO 90 AND GO SIDEWAYS OR BACKWARDS
    {

        if (!Configured)
        {
            Config.Configure(Motor1, Motor2, Motor3, Motor4);
            Configured = true;
        }

        Config.ResetMotorEncoders(Motor1, Motor2, Motor3, Motor4);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double HeadingAdjust = angles.firstAngle;

        //Mess with numbers, as different circumference.
        double Ticks = Degrees * 19.8;

        Motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Motor2.setTargetPosition((int) (-1 * Ticks));
        Motor1.setTargetPosition((int) Ticks);
        Motor4.setTargetPosition((int) (-1 * Ticks));
        Motor3.setTargetPosition((int) Ticks);

        double TurnAmount;

        if (Motor1.getTargetPosition() < 0) {
            Motor1.setPower(-1);
            Motor2.setPower(1);
            Motor3.setPower(-1);
            Motor4.setPower(1);
        } else {
            Motor1.setPower(1);
            Motor2.setPower(-1);
            Motor3.setPower(1);
            Motor4.setPower(-1);
        }

        while (Motor1.isBusy() || Motor2.isBusy() || Motor3.isBusy() || Motor4.isBusy()) {
            telemetry.update();
        }

        while (opModeIsActive()) {

            telemetry.update();

            TurnAmount = angles.firstAngle - HeadingAdjust;

            if ((Degrees - TurnAmount > -1) && (Degrees - TurnAmount < 1)) {

                Motor1.setPower(0);
                Motor2.setPower(0);
                Motor3.setPower(0);
                Motor4.setPower(0);

                break;

            }
            else if ((Degrees - TurnAmount >= 1) && (TurnAmount >= 0)) {

                Config.ResetMotorEncoders(Motor1, Motor2, Motor3, Motor4);

                Motor2.setTargetPosition((int) (-.3 * (Degrees - TurnAmount)));
                Motor4.setTargetPosition((int) (-.3 * (Degrees - TurnAmount)));
                Motor1.setTargetPosition((int) (.3 * (Degrees - TurnAmount)));
                Motor3.setTargetPosition((int) (.3 * (Degrees - TurnAmount)));

                Motor2.setPower(-.2);
                Motor1.setPower(.2);
                Motor4.setPower(-.2);
                Motor3.setPower(.2);
            }
            else if (Degrees - TurnAmount <= -1) {

                Config.ResetMotorEncoders(Motor1, Motor2, Motor3, Motor4);

                Motor2.setTargetPosition((int) (.3 * (Degrees - TurnAmount)));
                Motor4.setTargetPosition((int) (.3 * (Degrees - TurnAmount)));
                Motor1.setTargetPosition((int) (-.3 * (Degrees - TurnAmount)));
                Motor3.setTargetPosition((int) (-.3 * (Degrees - TurnAmount)));

                Motor2.setPower(.2);
                Motor1.setPower(-.2);
                Motor4.setPower(.2);
                Motor3.setPower(-.2);
            }
            else if (-1 * Degrees + TurnAmount <= -1) {

                Config.ResetMotorEncoders(Motor1, Motor2, Motor3, Motor4);

                Motor2.setTargetPosition((int) (-.3 * (Degrees - TurnAmount))); //Numbers off, fix using math.
                Motor4.setTargetPosition((int) (-.3 * (Degrees - TurnAmount)));
                Motor1.setTargetPosition((int) (.3 * (Degrees - TurnAmount)));
                Motor3.setTargetPosition((int) (.3 * (Degrees - TurnAmount)));

                Motor2.setPower(-.2);
                Motor1.setPower(.2);
                Motor4.setPower(-.2);
                Motor3.setPower(.2);
            }
            else {

                Config.ResetMotorEncoders(Motor1, Motor2, Motor3, Motor4);

                Motor2.setTargetPosition((int) (.3 * (Degrees - TurnAmount)));
                Motor4.setTargetPosition((int) (.3 * (Degrees - TurnAmount)));
                Motor1.setTargetPosition((int) (-.3 * (Degrees - TurnAmount)));
                Motor3.setTargetPosition((int) (-.3 * (Degrees - TurnAmount)));

                Motor2.setPower(.2);
                Motor1.setPower(-.2);
                Motor4.setPower(.2);
                Motor3.setPower(-.2);
            }
        }
    }
}
