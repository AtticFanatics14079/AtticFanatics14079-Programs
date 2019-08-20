package AtticFanaticsRoverRuckusPrograms;/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
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

/**
 * {@link AutonomousOpenCVTest} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
//
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "DepotSideRotated", group = "Sensor")
@Disabled
public class AutonomousDogeCVDepotSideRotated extends LinearOpMode
{

    private DcMotor Motor1 = null;
    private DcMotor Motor2 = null;
    private DcMotor Motor3 = null;
    private DcMotor Motor4 = null;
    private DcMotor lifter_lander = null;
    //private DcMotor ingester = null;
    private Servo Claim;
    private BNO055IMU imu;
    private GoldAlignDetector detector;


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
        Motor1 = hardwareMap.get(DcMotor.class, "motor_1");
        Motor2 = hardwareMap.get(DcMotor.class, "motor_2");
        Motor3 = hardwareMap.get(DcMotor.class, "motor_3");
        Motor4 = hardwareMap.get(DcMotor.class, "motor_4");
        lifter_lander = hardwareMap.get(DcMotor.class, "lifter");
        //ingester = hardwareMap.get(DcMotor.class, "ingester");
        Claim = hardwareMap.get(Servo.class, "Claim");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        Motor1.setDirection(DcMotor.Direction.REVERSE);
        Motor3.setDirection(DcMotor.Direction.REVERSE);
        lifter_lander.setDirection(DcMotor.Direction.REVERSE);
        //ingester.setDirection(DcMotor.Direction.FORWARD);

        Motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter_lander.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //ingester.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up our telemetry dashboard
        composeTelemetry();

        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");

        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!

        while (! isStarted()) {
            Claim.setPosition(.6);
            telemetry.update();
        }

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Loop and update the dashboard
        //land:
        lifter_lander.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter_lander.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter_lander.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        lifter_lander.setTargetPosition(21690);
        lifter_lander.setPower(1);
        while(lifter_lander.isBusy()){
            lifter_lander.setPower(1);
            telemetry.update();
        }
        lifter_lander.setPower(0);
        //sideways towards samples
        SidewaysMovement(-2.5);

        while (opModeIsActive()) {

            telemetry.update();
            telemetry.addData("IsAligned", detector.getAligned()); // Is the bot aligned with the gold mineral?
            telemetry.addData("X Pos", detector.getXPosition()); // Gold X position.

            if (detector.getXPosition() > 450) {

                MoveEncoderTicks(-25);
                TurnUsingIMU(-42);
                MoveEncoderTicks(-82);
                TurnUsingIMU(68);
                MoveEncoderTicks(-65);
                Claim.setPosition(.1);
                MoveEncoderTicks(10);
                MoveEncoderTicks(55);

                break;
            }
            //turn to second position
            else if ((detector.getXPosition() < 115) && (detector.getXPosition() > 0)) {

                MoveEncoderTicks(-25);
                TurnUsingIMU(42);
                MoveEncoderTicks(-82);
                TurnUsingIMU(-68);
                MoveEncoderTicks(-65);
                Claim.setPosition(.1);
                MoveEncoderTicks(10);
                MoveEncoderTicks(55);

                break;
            }

            else if (detector.getAligned()) {

                MoveEncoderTicks(-140);
                Claim.setPosition(.1);
                MoveEncoderTicks(140);

                break;
            }


        }

        detector.disable();
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
        Motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void MoveEncoderTicks(double NumbCM) {

        ResetMotorEncoders();

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double HeadingAdjust = angles.firstAngle;

        double TurnAmount;

        Motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

    private void TurnUsingIMU(int Degrees) //DO NOT TURN CLOSE TO A 180; INSTEAD JUST TURN UP TO 90 AND GO SIDEWAYS OR BACKWARDS
    {

        ResetMotorEncoders();

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double HeadingAdjust = angles.firstAngle;

        double Ticks = Degrees * -27; //Numbers off, fix using math

        double TicksAdjust = Ticks * .5;

        Motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Motor2.setTargetPosition((int) (-1 * Ticks));
        Motor1.setTargetPosition((int) Ticks);
        Motor4.setTargetPosition((int) (-1 * TicksAdjust));
        Motor3.setTargetPosition((int) TicksAdjust);

        double TurnAmount;

        if (Motor1.getTargetPosition() < 0) {
            Motor1.setPower(-1);
            Motor2.setPower(1);
            Motor3.setPower(-.5);
            Motor4.setPower(.5);
        } else {
            Motor1.setPower(1);
            Motor2.setPower(-1);
            Motor3.setPower(.5);
            Motor4.setPower(-.5);
        }

        while (Motor1.isBusy() || Motor2.isBusy() || Motor3.isBusy() || Motor4.isBusy()) {
            telemetry.update();
        }

        /*while (opModeIsActive()) {

            telemetry.update();

            TurnAmount = angles.firstAngle - HeadingAdjust;

            if ((Degrees - TurnAmount > -3) && (Degrees - TurnAmount < 3)) {

                Motor1.setPower(0);
                Motor2.setPower(0);
                Motor3.setPower(0);
                Motor4.setPower(0);

                break;

            }
            else if ((Degrees - TurnAmount >= 3) && (TurnAmount >= 0)) {

                ResetMotorEncoders();

                Motor2.setTargetPosition((int) (-.6 * (Degrees - TurnAmount)));
                Motor4.setTargetPosition((int) (-.3 * (Degrees - TurnAmount)));
                Motor1.setTargetPosition((int) (.6 * (Degrees - TurnAmount)));
                Motor3.setTargetPosition((int) (.3 * (Degrees - TurnAmount)));

                Motor2.setPower(-.6);
                Motor1.setPower(.6);
                Motor4.setPower(-.3);
                Motor3.setPower(.3);
            }
            else if (Degrees - TurnAmount <= -3) {

                ResetMotorEncoders();

                Motor2.setTargetPosition((int) (.6 * (Degrees - TurnAmount)));
                Motor4.setTargetPosition((int) (.3 * (Degrees - TurnAmount)));
                Motor1.setTargetPosition((int) (-.6 * (Degrees - TurnAmount)));
                Motor3.setTargetPosition((int) (-.3 * (Degrees - TurnAmount)));

                Motor2.setPower(.6);
                Motor1.setPower(-.6);
                Motor4.setPower(.3);
                Motor3.setPower(-.3);
            }
            else if (-1 * Degrees + TurnAmount <= -3) {

                ResetMotorEncoders();

                Motor2.setTargetPosition((int) (-.6 * (Degrees - TurnAmount))); //Numbers off, fix using math.
                Motor4.setTargetPosition((int) (-.3 * (Degrees - TurnAmount)));
                Motor1.setTargetPosition((int) (.6 * (Degrees - TurnAmount)));
                Motor3.setTargetPosition((int) (.3 * (Degrees - TurnAmount)));

                Motor2.setPower(-.6);
                Motor1.setPower(.6);
                Motor4.setPower(-.3);
                Motor3.setPower(.3);
            }
            else {

                ResetMotorEncoders();

                Motor2.setTargetPosition((int) (.6 * (Degrees - TurnAmount)));
                Motor4.setTargetPosition((int) (.3 * (Degrees - TurnAmount)));
                Motor1.setTargetPosition((int) (-.6 * (Degrees - TurnAmount)));
                Motor3.setTargetPosition((int) (-.3 * (Degrees - TurnAmount)));

                Motor2.setPower(.6);
                Motor1.setPower(-.6);
                Motor4.setPower(.3);
                Motor3.setPower(-.3);
            }
        }
        */
    }

    private void SidewaysMovement(double NumbCM) {

        ResetMotorEncoders();

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double HeadingAdjust = angles.firstAngle;

        double TurnAmount;

        Motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double Ticks = 72 * NumbCM;

        double AdjustedTicks = .42 * Ticks;

        Motor1.setTargetPosition((int) Ticks);
        Motor4.setTargetPosition((int) AdjustedTicks);
        Motor2.setTargetPosition((int) (-1 * Ticks));
        Motor3.setTargetPosition((int) (-1 * AdjustedTicks));

        if (Motor2.getTargetPosition() < 0) {
            Motor1.setPower(.9);
            Motor2.setPower(-.9);
            Motor3.setPower(-.44);
            Motor4.setPower(.44);
        }
        else {
            Motor1.setPower(-.9);
            Motor2.setPower(.9);
            Motor3.setPower(.44);
            Motor4.setPower(-.44);
        }

        while (Motor1.isBusy() || Motor2.isBusy() || Motor3.isBusy() || Motor4.isBusy()) {
            telemetry.update();
            TurnAmount = angles.firstAngle - HeadingAdjust;
            if (TurnAmount > 2 && Motor1.getPower() > 0) {
                Motor3.setPower(.44);
                Motor1.setPower(-1);
                Motor4.setPower(-.35);
                Motor2.setPower(.9);
            } else if (TurnAmount > 2 && Motor1.getPower() < 0) {
                Motor3.setPower(-.44);
                Motor1.setPower(.8);
                Motor4.setPower(.55);
                Motor2.setPower(-.9);
            } else if (TurnAmount < -2 && Motor1.getPower() > 0) {
                Motor3.setPower(-.45);
                Motor1.setPower(.8);
                Motor4.setPower(.55);
                Motor2.setPower(-.9);
            } else if (TurnAmount < -2 && Motor1.getPower() < 0) {
                Motor3.setPower(.44);
                Motor1.setPower(-.8);
                Motor4.setPower(-.55);
                Motor2.setPower(.9);
            } else if (Motor2.getPower() > 0) {
                Motor1.setPower(-.9);
                Motor2.setPower(.9);
                Motor3.setPower(.44);
                Motor4.setPower(-.44);
            } else {
                Motor1.setPower(.9);
                Motor2.setPower(-.9);
                Motor3.setPower(-.44);
                Motor4.setPower(.44);
            }
        }

        Motor1.setPower(0);
        Motor2.setPower(0);
        Motor3.setPower(0);
        Motor4.setPower(0);
    }

}
