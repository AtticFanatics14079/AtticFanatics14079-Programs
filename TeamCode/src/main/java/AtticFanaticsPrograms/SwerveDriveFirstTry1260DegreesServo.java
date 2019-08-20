package AtticFanaticsPrograms;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;
import java.util.concurrent.TimeUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="SwerveDriveFirstTry1260DegreesServo", group="Iterative Opmode")
//@Disabled
public class SwerveDriveFirstTry1260DegreesServo extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Motor1;
    private DcMotor Motor2;
    private DcMotor Motor3;
    private DcMotor Motor4;
    private Servo Servo1;
    private Servo Servo2;
    private Servo Servo3;
    private Servo Servo4;
    private double HeadingAdjust;
    private boolean OutOfPosition;

    private BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;


    @Override
    public void init() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        telemetry.addData("Status", "Initialized");
        Motor1 = hardwareMap.get(DcMotor.class, "motor_1");
        Motor2 = hardwareMap.get(DcMotor.class, "motor_2");
        Motor3 = hardwareMap.get(DcMotor.class, "motor_3");
        Motor4 = hardwareMap.get(DcMotor.class, "motor_4");
        Servo1 = hardwareMap.get(Servo.class, "servo_1");
        Servo2 = hardwareMap.get(Servo.class, "servo_2");
        Servo3 = hardwareMap.get(Servo.class, "servo_3");
        Servo4 = hardwareMap.get(Servo.class, "servo_4");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        Motor2.setDirection(DcMotor.Direction.REVERSE);
        Motor4.setDirection(DcMotor.Direction.REVERSE);

        Motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Playing");

        composeTelemetry();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (gamepad1.start)
        {
            HeadingAdjust = angles.firstAngle;
        }

        if (gamepad1.right_trigger != 0)
        {
            if (!OutOfPosition)
            {
                Servo1.setPosition(Servo1.getPosition() + 0.07); //Replace position modifier with whatever is 90 degrees
                Servo4.setPosition(Servo1.getPosition() + 0.07); //Replace position modifier with whatever is 90 degrees
                OutOfPosition = true;
            }

            if (angles.firstAngle - HeadingAdjust <= 90 && angles.firstAngle - HeadingAdjust >= 0)
            {
                Servo1.setPosition(Servo1.getPosition() + 0.07); //Replace position modifier with whatever is 90 degrees
                Motor1.setPower(1);
                Servo2.setPosition(Servo1.getPosition() + 0.07); //Replace position modifier with whatever is 90 degrees
                Motor2.setPower(-gamepad1.left_stick_y);
                Servo4.setPosition(Servo1.getPosition() + 0.07); //Replace position modifier with whatever is 90 degrees
                Motor4.setPower(1);
                Servo3.setPosition(Servo1.getPosition() + 0.07); //Replace position modifier with whatever is 90 degrees
                Motor3.setPower(gamepad1.left_stick_y);
                while (angles.firstAngle - HeadingAdjust <= 90 && angles.firstAngle - HeadingAdjust >= 0)
                {
                    Servo1.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                    Servo2.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                    Servo3.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                    Servo4.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                }
            }
            else if (angles.firstAngle - HeadingAdjust <= 180 && angles.firstAngle - HeadingAdjust >= 90)
            {
                Servo2.setPosition(Servo1.getPosition() + 0.07); //Replace position modifier with whatever is 90 degrees
                Motor2.setPower(1);
                Servo1.setPosition(Servo1.getPosition() + 0.07); //Replace position modifier with whatever is 90 degrees
                Motor1.setPower(-gamepad1.left_stick_y);
                Servo3.setPosition(Servo1.getPosition() + 0.07); //Replace position modifier with whatever is 90 degrees
                Motor3.setPower(1);
                Servo4.setPosition(Servo1.getPosition() + 0.07); //Replace position modifier with whatever is 90 degrees
                Motor4.setPower(gamepad1.left_stick_y);
                while (angles.firstAngle - HeadingAdjust <= 180 && angles.firstAngle - HeadingAdjust >= 90)
                {
                    Servo1.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                    Servo2.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                    Servo3.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                    Servo4.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                }
            }
            else if (angles.firstAngle - HeadingAdjust <= -90 && angles.firstAngle - HeadingAdjust >= -180)
            {
                Servo1.setPosition(Servo1.getPosition() + 0.07); //Replace position modifier with whatever is 90 degrees
                Motor1.setPower(1);
                Servo2.setPosition(Servo1.getPosition() + 0.07); //Replace position modifier with whatever is 90 degrees
                Motor2.setPower(gamepad1.left_stick_y);
                Servo4.setPosition(Servo1.getPosition() + 0.07); //Replace position modifier with whatever is 90 degrees
                Motor4.setPower(1);
                Servo3.setPosition(Servo1.getPosition() + 0.07); //Replace position modifier with whatever is 90 degrees
                Motor3.setPower(-gamepad1.left_stick_y);
                while (angles.firstAngle - HeadingAdjust <= -90 && angles.firstAngle - HeadingAdjust >= -180)
                {
                    Servo1.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                    Servo2.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                    Servo3.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                    Servo4.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                }
            }
            else if (angles.firstAngle - HeadingAdjust <= 0 && angles.firstAngle - HeadingAdjust >= -90)
            {
                Servo2.setPosition(Servo1.getPosition() + 0.07); //Replace position modifier with whatever is 90 degrees
                Motor2.setPower(1);
                Servo1.setPosition(Servo1.getPosition() + 0.07); //Replace position modifier with whatever is 90 degrees
                Motor1.setPower(gamepad1.left_stick_y);
                Servo3.setPosition(Servo1.getPosition() + 0.07); //Replace position modifier with whatever is 90 degrees
                Motor3.setPower(1);
                Servo4.setPosition(Servo1.getPosition() + 0.07); //Replace position modifier with whatever is 90 degrees
                Motor4.setPower(-gamepad1.left_stick_y);
                while (angles.firstAngle - HeadingAdjust <= 0 && angles.firstAngle - HeadingAdjust >= -90)
                {
                    Servo1.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                    Servo2.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                    Servo3.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                    Servo4.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                }
            }
        }

        else if (gamepad1.left_trigger != 0)
        {
            if (!OutOfPosition)
            {
                Servo1.setPosition(Servo1.getPosition() + 0.07); //Replace position modifier with whatever is 90 degrees
                Servo4.setPosition(Servo1.getPosition() + 0.07); //Replace position modifier with whatever is 90 degrees
                OutOfPosition = true;
            }

            if (angles.firstAngle - HeadingAdjust <= 0 && angles.firstAngle - HeadingAdjust >= -90)
            {
                Servo1.setPosition(Servo1.getPosition() - 0.07); //Replace position modifier with whatever is 90 degrees
                Motor1.setPower(-1);
                Servo2.setPosition(Servo1.getPosition() - 0.07); //Replace position modifier with whatever is 90 degrees
                Motor2.setPower(-gamepad1.left_stick_y);
                Servo4.setPosition(Servo1.getPosition() - 0.07); //Replace position modifier with whatever is 90 degrees
                Motor4.setPower(-1);
                Servo3.setPosition(Servo1.getPosition() - 0.07); //Replace position modifier with whatever is 90 degrees
                Motor3.setPower(gamepad1.left_stick_y);
                while (angles.firstAngle - HeadingAdjust <= 0 && angles.firstAngle - HeadingAdjust >= -90)
                {
                    Servo1.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                    Servo2.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                    Servo3.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                    Servo4.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                }
            }
            else if (angles.firstAngle - HeadingAdjust <= -90 && angles.firstAngle - HeadingAdjust >= -180)
            {
                Servo2.setPosition(Servo1.getPosition() - 0.07); //Replace position modifier with whatever is 90 degrees
                Motor2.setPower(-1);
                Servo1.setPosition(Servo1.getPosition() - 0.07); //Replace position modifier with whatever is 90 degrees
                Motor1.setPower(-gamepad1.left_stick_y);
                Servo3.setPosition(Servo1.getPosition() - 0.07); //Replace position modifier with whatever is 90 degrees
                Motor3.setPower(-1);
                Servo4.setPosition(Servo1.getPosition() - 0.07); //Replace position modifier with whatever is 90 degrees
                Motor4.setPower(gamepad1.left_stick_y);
                while (angles.firstAngle - HeadingAdjust <= -90 && angles.firstAngle - HeadingAdjust >= -180)
                {
                    Servo1.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                    Servo2.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                    Servo3.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                    Servo4.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                }
            }
            else if (angles.firstAngle - HeadingAdjust <= 180 && angles.firstAngle - HeadingAdjust >= 90)
            {
                Servo1.setPosition(Servo1.getPosition() - 0.07); //Replace position modifier with whatever is 90 degrees
                Motor1.setPower(-1);
                Servo2.setPosition(Servo1.getPosition() - 0.07); //Replace position modifier with whatever is 90 degrees
                Motor2.setPower(gamepad1.left_stick_y);
                Servo4.setPosition(Servo1.getPosition() - 0.07); //Replace position modifier with whatever is 90 degrees
                Motor4.setPower(-1);
                Servo3.setPosition(Servo1.getPosition() - 0.07); //Replace position modifier with whatever is 90 degrees
                Motor3.setPower(-gamepad1.left_stick_y);
                while (angles.firstAngle - HeadingAdjust <= 180 && angles.firstAngle - HeadingAdjust >= 90)
                {
                    Servo1.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                    Servo2.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                    Servo3.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                    Servo4.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                }
            }
            else if (angles.firstAngle - HeadingAdjust <= 90 && angles.firstAngle - HeadingAdjust >= 0)
            {
                Servo2.setPosition(Servo1.getPosition() - 0.07); //Replace position modifier with whatever is 90 degrees
                Motor2.setPower(-1);
                Servo1.setPosition(Servo1.getPosition() - 0.07); //Replace position modifier with whatever is 90 degrees
                Motor1.setPower(gamepad1.left_stick_y);
                Servo3.setPosition(Servo1.getPosition() - 0.07); //Replace position modifier with whatever is 90 degrees
                Motor3.setPower(-1);
                Servo4.setPosition(Servo1.getPosition() - 0.07); //Replace position modifier with whatever is 90 degrees
                Motor4.setPower(-gamepad1.left_stick_y);
                while (angles.firstAngle - HeadingAdjust <= 90 && angles.firstAngle - HeadingAdjust >= 0)
                {
                    Servo1.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                    Servo2.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                    Servo3.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                    Servo4.setPosition(Servo1.getPosition() - 0.0007777 * (angles.firstAngle - HeadingAdjust));
                }
            }
        }
        else {
            if (OutOfPosition)
            {
                ResetServos();
                OutOfPosition = false;
            }

            Motor1.setPower(gamepad1.left_stick_y);
            Motor2.setPower(gamepad1.left_stick_y);
            Motor3.setPower(gamepad1.left_stick_y);
            Motor4.setPower(gamepad1.left_stick_y);

            Servo1.setPosition(0.0007777 * gamepad1.right_stick_x + 0.07);
            Servo2.setPosition(0.0007777 * gamepad1.right_stick_x + 0.07);
            Servo3.setPosition(0.0007777 * gamepad1.right_stick_x + 0.07);
            Servo4.setPosition(0.0007777 * gamepad1.right_stick_x + 0.07);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    void ResetServos ()
    {
        Servo2.setPosition(Servo1.getPosition());
        Servo3.setPosition(Servo1.getPosition());
        Servo4.setPosition(Servo1.getPosition());
    }
    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle); //I believe this is what we want starting with angles.angleUnit
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle); //I believe this is what we want starting with angles.angleUnit
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle); //I believe this is what we want starting with angles.angleUnit
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString(); //Returns the gravity as the string value
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
