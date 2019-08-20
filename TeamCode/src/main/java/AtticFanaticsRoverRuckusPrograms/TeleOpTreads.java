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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOpActual", group="Iterative Opmode")
//@Disabled Remove the comment to disable
public class TeleOpTreads extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left_motor = null;
    private DcMotor right_motor = null;
    private DcMotor lifter_lander = null;
    private DcMotor ingester = null;
    private Servo box1 = null;
    private Servo box2 = null;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        left_motor  = hardwareMap.get(DcMotor.class, "left motor");
        right_motor = hardwareMap.get(DcMotor.class, "right motor");
        lifter_lander = hardwareMap.get(DcMotor.class, "lifter");
        ingester = hardwareMap.get(DcMotor.class, "ingester");
        box1 = hardwareMap.get(Servo.class, "box1");
        box2 = hardwareMap.get(Servo.class, "box2");

        left_motor.setDirection(DcMotor.Direction.FORWARD);
        right_motor.setDirection(DcMotor.Direction.REVERSE);
        lifter_lander.setDirection(DcMotor.Direction.FORWARD);
        ingester.setDirection(DcMotor.Direction.FORWARD);

        left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter_lander.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ingester.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Playing");
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

        left_motor.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x);
        right_motor.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x);

        if (gamepad1.a)
            ingester.setPower(1);
        else if (gamepad1.b)
            ingester.setPower(-1);
        else ingester.setPower(0);

        if (gamepad1.x)
            lifter_lander.setPower(1);
        else if (gamepad1.y)
            lifter_lander.setPower(-1);
        else lifter_lander.setPower(0);

        if (gamepad1.left_trigger != 0) {
            box1.setPosition(1);
            box2.setPosition(0);
        }
        else {
            box1.setPosition(0.7);
            box2.setPosition(0.3);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
