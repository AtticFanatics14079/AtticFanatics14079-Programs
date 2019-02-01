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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutonomousWithoutIMU", group="Linear Opmode")

@Disabled

public class AutonomousWithoutIMU extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left_motor = null;
    private DcMotor right_motor = null;
    private DcMotor lifter_lander = null;
    private DcMotor ingester = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        left_motor  = hardwareMap.get(DcMotor.class, "left motor");
        right_motor = hardwareMap.get(DcMotor.class, "right motor");
        lifter_lander = hardwareMap.get(DcMotor.class, "lifter");
        ingester = hardwareMap.get(DcMotor.class, "ingester");

        right_motor.setDirection(DcMotor.Direction.FORWARD);
        left_motor.setDirection(DcMotor.Direction.REVERSE);
        lifter_lander.setDirection(DcMotor.Direction.FORWARD);
        ingester.setDirection(DcMotor.Direction.FORWARD);

        left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter_lander.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ingester.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();

        if(opModeIsActive()) {
            telemetry.update();
            lifter_lander.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ingester.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            lifter_lander.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ingester.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            lifter_lander.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ingester.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            MoveEncoderTicks(20);

            MoveEncoderTicks(-20);

            TurnEncoderTicks(90);

            TurnEncoderTicks(90);
        }
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

        left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int Ticks = NumbCM * 84;
        left_motor.setTargetPosition(Ticks);
        right_motor.setTargetPosition(Ticks);

        left_motor.setPower(1);
        right_motor.setPower(1);

        while (left_motor.isBusy() || right_motor.isBusy()) {
            telemetry.update();
        }

        left_motor.setPower(0);
        right_motor.setPower(0);

    }

    private void TurnEncoderTicks(int Degrees)
    {
        ResetMotorEncoders();

        int Ticks = Degrees * 26;

        left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left_motor.setTargetPosition(Ticks);
        right_motor.setTargetPosition(-Ticks);

        left_motor.setPower(1);
        right_motor.setPower(1);

        while (left_motor.isBusy() || right_motor.isBusy()) {
            telemetry.update();
        }

        left_motor.setPower(0);
        right_motor.setPower(0);
    }

}
