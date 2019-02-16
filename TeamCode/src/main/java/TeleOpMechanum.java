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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOpMechanum", group="Iterative Opmode")
//@Disabled Remove the comment to disable
public class TeleOpMechanum extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Motor1 = null;
    private DcMotor Motor2 = null;
    private DcMotor Motor3 = null;
    private DcMotor Motor4 = null;
    private DcMotor lifter_lander = null;
    private DcMotor ingester = null;
     //private Servo box1 = null;
    //private Servo box2 = null;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        Motor1 = hardwareMap.get(DcMotor.class, "motor_1");
        Motor2 = hardwareMap.get(DcMotor.class, "motor_2");
        Motor3 = hardwareMap.get(DcMotor.class, "motor_3");
        Motor4 = hardwareMap.get(DcMotor.class, "motor_4");
        lifter_lander = hardwareMap.get(DcMotor.class, "lifter");
        ingester = hardwareMap.get(DcMotor.class, "ingester");
        //box1 = hardwareMap.get(Servo.class, "box1");
       // box2 = hardwareMap.get(Servo.class, "box2");

        Motor2.setDirection(DcMotor.Direction.REVERSE);
        Motor4.setDirection(DcMotor.Direction.REVERSE);
        lifter_lander.setDirection(DcMotor.Direction.FORWARD);
        ingester.setDirection(DcMotor.Direction.FORWARD);

        Motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        if (gamepad1.left_bumper) {
            if (gamepad1.right_stick_x != 0) {
                Motor1.setPower(-.3 * (gamepad1.right_stick_x));
                Motor4.setPower(-.3 * (gamepad1.right_stick_x));
                Motor3.setPower(.3 * (gamepad1.right_stick_x));
                Motor2.setPower(.3 * (gamepad1.right_stick_x));
            }
            else {
                Motor1.setPower(.3 * (gamepad1.left_stick_y));
                Motor2.setPower(.3 * (gamepad1.left_stick_y));
                Motor3.setPower(.3 * (gamepad1.left_stick_y));
                Motor4.setPower(.3 * (gamepad1.left_stick_y));
            }

        }
        else if (gamepad1.right_bumper){
            Motor1.setPower(-1 * (gamepad1.right_stick_x));
            Motor4.setPower(-1 * (gamepad1.right_stick_x));
            Motor3.setPower(1 * (gamepad1.right_stick_x));
            Motor2.setPower(1 * (gamepad1.right_stick_x));
        }
        else {
            Motor1.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x);
            Motor2.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x);
            Motor3.setPower(gamepad1. left_stick_y + gamepad1.right_stick_x);
            Motor4.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x);
        }

        if (gamepad1.a)
            ingester.setPower(.5);
        else if (gamepad1.b)
            ingester.setPower(-.5);
        else ingester.setPower(0);

        if (gamepad2.x)
            lifter_lander.setPower(1);
        else if (gamepad2.y)
            lifter_lander.setPower(-1);
        else lifter_lander.setPower(0);

       // if (gamepad1.left_trigger != 0) {
        //    box1.setPosition(1);
       //     box2.setPosition(0);
      //  }
        //else {
      //      box1.setPosition(0.7);
      //      box2.setPosition(0.3);
      //  }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
