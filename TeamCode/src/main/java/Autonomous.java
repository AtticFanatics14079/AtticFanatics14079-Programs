/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * {@link Autonomous} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutonomousTreads", group = "Sensor")
//@Disabled
public class Autonomous extends LinearOpMode
    {

        private DcMotor left_motor;
        private DcMotor right_motor;
        private DcMotor lifter_lander;
        private DcMotor ingester;
        BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override public void runOpMode() {

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        left_motor = hardwareMap.get(DcMotor.class, "left motor");
        right_motor = hardwareMap.get(DcMotor.class, "right motor");
        lifter_lander = hardwareMap.get(DcMotor.class, "lifter");
        ingester = hardwareMap.get(DcMotor.class, "ingester");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        right_motor.setDirection(DcMotor.Direction.FORWARD);
        left_motor.setDirection(DcMotor.Direction.REVERSE);
        lifter_lander.setDirection(DcMotor.Direction.FORWARD);
        ingester.setDirection(DcMotor.Direction.FORWARD);

        left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter_lander.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ingester.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up our telemetry dashboard
        composeTelemetry();

        // Wait until we're told to go
        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Loop and update the dashboard
        TurnUsingIMU(-97);
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

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

    private void ResetMotorEncoders(){
        left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

        private void MoveEncoderTicks(int NumbCM)
        {

            ResetMotorEncoders();

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double TurnAmount;

            left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            int Ticks = NumbCM * 84;
            left_motor.setTargetPosition(Ticks);
            right_motor.setTargetPosition(Ticks);

            left_motor.setPower(1);
            right_motor.setPower(1);

            while (left_motor.isBusy() || right_motor.isBusy()) {
                telemetry.update();
                TurnAmount = angles.firstAngle;
                if (TurnAmount > .3) {
                    right_motor.setPower(.9);
                    left_motor.setPower(1);
                }
                else if (TurnAmount < -.3)
                {
                    left_motor.setPower(.9);
                    right_motor.setPower(1);
                }
                else {
                    left_motor.setPower(1);
                    right_motor.setPower(1);
                }
            }

            left_motor.setPower(0);
            right_motor.setPower(0);

        }

        private void TurnUsingIMU(int Degrees)
        {

            ResetMotorEncoders();

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double Ticks = Degrees * 32;

            left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            left_motor.setTargetPosition((int) (-1 * Ticks));
            right_motor.setTargetPosition((int) Ticks);

            double TurnAmount;

            left_motor.setPower(-1);
            right_motor.setPower(1);

            while (left_motor.isBusy() || right_motor.isBusy())
            {
                telemetry.update();
            }

            while (opModeIsActive()) {

                telemetry.update();
                TurnAmount = angles.firstAngle;
                if (Degrees - TurnAmount > -2 && Degrees - TurnAmount < 2) {

                    left_motor.setPower(0);
                    right_motor.setPower(0);
                    break;
                }
                else if ((Degrees - TurnAmount >= 2) && (TurnAmount >= 0)) {

                    ResetMotorEncoders();

                    left_motor.setTargetPosition((int) (-3 * (Degrees - TurnAmount)));
                    right_motor.setTargetPosition((int) (3 * (Degrees - TurnAmount)));

                    left_motor.setPower(-1);
                    right_motor.setPower(1);
                }
                else if (Degrees - TurnAmount <= -2){

                    ResetMotorEncoders();

                    left_motor.setTargetPosition((int) (3 * (Degrees - TurnAmount)));
                    right_motor.setTargetPosition((int) (-3 * (Degrees - TurnAmount)));

                    left_motor.setPower(1);
                    right_motor.setPower(-1);
                }
                else if (-1 * Degrees + TurnAmount <= -2){

                    ResetMotorEncoders();

                    left_motor.setTargetPosition((int) (-3 * (Degrees - TurnAmount)));
                    right_motor.setTargetPosition((int) (3 * (Degrees - TurnAmount)));

                    left_motor.setPower(-1);
                    right_motor.setPower(1);
                }
                else {

                    ResetMotorEncoders();

                    left_motor.setTargetPosition((int) (3 * (Degrees - TurnAmount)));
                    right_motor.setTargetPosition((int) (-3 * (Degrees - TurnAmount)));

                    left_motor.setPower(1);
                    right_motor.setPower(-1);
                }
            }

        }

}

