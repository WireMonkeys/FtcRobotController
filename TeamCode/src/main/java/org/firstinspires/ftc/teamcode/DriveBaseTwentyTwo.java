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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DigitalChannel;



@TeleOp(name="DriveBaseTwentyTwo", group="DriveBase")

public class DriveBaseTwentyTwo extends OpMode
{
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotor eMotor = null;

    private Servo   wrist;
    private Servo   hand;


//    boolean rampUp = true;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        leftRear  = hardwareMap.get(DcMotor.class, "backLeft");
        rightRear = hardwareMap.get(DcMotor.class, "backRight");
        eMotor = hardwareMap.get(DcMotor.class, "eMotor");
        hand = hardwareMap.get(Servo.class, "hand");
        wrist = hardwareMap.get(Servo.class, "wrist");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);


        eMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        eMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addData("Starting at",  "%7d",
                eMotor.getCurrentPosition());
        telemetry.update();
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }
    double handStick = 0.3059;
    double wristStick = 0.3577;
    int eMotors = 0;
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        hand.setPosition(handStick);
        wrist.setPosition(wristStick);

        handStick = handStick + (-gamepad2.right_stick_x * 0.005);
        wristStick = wristStick + (-gamepad2.left_stick_x * 0.005);




        wristStick = Range.clip(wristStick, 0.0d, 1.0d);
        handStick = Range.clip(handStick, 0.3096d, 0.656d);

        telemetry.addData("hand pos",hand.getPosition());
        telemetry.addData("wrist pos",wrist.getPosition());

        if(gamepad2.a){
            eMotors = 100;
        }
        if(gamepad2.y){
            eMotors = 4088;
        }
        if(gamepad2.x){
            eMotors = 1610;
        }
        if(gamepad2.b){
            eMotors = 2849;
        }

        if(gamepad2.right_bumper){
            eMotors = eMotors - 10;
        }
        else if(gamepad2.left_bumper){
            eMotors = eMotors + 10;
        }
        eMotors = Range.clip(eMotors,0, 4088);

        eMotor.setTargetPosition(eMotors);

        eMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        eMotor.setPower(Math.abs(0.5));



        drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);



        telemetry.addData("Currently at",  " at %7d",
                eMotor.getCurrentPosition());
        telemetry.addData("Currently at",  " at %7d",
                leftRear.getCurrentPosition());

    }

    public void drive(double forward, double strafe, double turn) {
        double r = Math.hypot(strafe, forward);
        double robotAngle = Math.atan2(forward, strafe) - Math.PI / 4;
        double rightX = turn;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        leftFront.setPower(v1/1.5);
        rightFront.setPower(v2/1.5);
        leftRear.setPower(v3/1.5);
        rightRear.setPower(v4/1.5);
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", v1, v2);
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", v1, v2);



    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}