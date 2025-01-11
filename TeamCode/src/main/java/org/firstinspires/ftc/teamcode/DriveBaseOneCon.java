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



@TeleOp(name="Simon's Drive Base", group="DriveBase")

public class DriveBaseOneCon extends OpMode
{
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotor eMotor = null;
    private DcMotor pivot = null;
    private DcMotor reMotor = null;
    private DcMotor intake = null;

    private Servo   wrist;
    private Servo   hand;
    private Servo   plane;
    private Servo   elbow;
    private Servo   relbow;
    private Servo   output;


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
        reMotor = hardwareMap.get(DcMotor.class, "reMotor");
        pivot = hardwareMap.get(DcMotor.class, "pivot");
        intake = hardwareMap.get(DcMotor.class, "intake");
        hand = hardwareMap.get(Servo.class, "hand");
        wrist = hardwareMap.get(Servo.class, "wrist");
        plane = hardwareMap.get(Servo.class, "planes");
        elbow = hardwareMap.get(Servo.class, "elbow");
        relbow = hardwareMap.get(Servo.class, "relbow");
        output = hardwareMap.get(Servo.class, "output");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);


        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        eMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        eMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        reMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        reMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


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
    double handStick = 0.255;
    double wristStick = 0.86;
    double planes = 0.0;
    double elbowPad = 0.25;
    double place = 0.5;
    int eMotors = 0;
    int pivotMotor = 0;
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        hand.setPosition(handStick);
        wrist.setPosition(wristStick);
        plane.setPosition(planes);
        elbow.setPosition(elbowPad);
        relbow.setPosition(elbowPad);
        output.setPosition(place);


       if (gamepad1.a){
           handStick = 0.4401;
           wristStick = 0.46983;
       }
       if (gamepad1.b){
           wristStick = 0.855;
           handStick = 0.255;
       }
       if(gamepad1.y){
           handStick = 0.86345;
           wristStick = 0.025;
       }

        intake.setPower(gamepad1.left_trigger - gamepad1.right_trigger);



        if (gamepad1.left_bumper){
            place = 1.0;
        }
        else if (gamepad1.right_bumper){
            place = 0.0;
        }
        else {
            place = 0.5;
        }


        planes = planes + ((gamepad2.right_trigger * 0.005) - (gamepad2.left_trigger * 0.005));

        planes = Range.clip(planes, 0.0d,1.0d);
        wristStick = Range.clip(wristStick, 0.025d, 0.914d);
        handStick = Range.clip(handStick, 0.0d, 1.0d);
        elbowPad = Range.clip(elbowPad, 0.25d, 0.75d);

        telemetry.addData("hand pos",hand.getPosition());
        telemetry.addData("wrist pos",wrist.getPosition());
        telemetry.addData("elbow pos",elbow.getPosition());
        telemetry.addData("plane pos",planes);



        if(gamepad1.dpad_up){
            eMotors = eMotors + 20;
        }
        else if(gamepad1.dpad_down){
            eMotors = eMotors - 20;
        }

        eMotors = Range.clip(eMotors,5, 3360);

        eMotor.setTargetPosition(eMotors);
        reMotor.setTargetPosition(eMotors);

        eMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        reMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        eMotor.setPower(Math.abs(1.0));
        reMotor.setPower(Math.abs(1.0));


//        pivot.setTargetPosition(pivotMotor);
//
//        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        pivot.setPower(Math.abs(1.0));
//
//        pivotMotor = Range.clip(pivotMotor, 0, 11100);
//
//
//        if(gamepad1.dpad_up){
//            pivotMotor = pivotMotor + 20;
//        }
//        else if(gamepad1.dpad_down){
//            pivotMotor = pivotMotor - 20;
//        }


        if(gamepad1.dpad_left){
            elbowPad = elbowPad - 0.0025;
        }
        else if(gamepad1.dpad_right){
            elbowPad = elbowPad + 0.0025;
        }




        drive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x / 1.25);



        telemetry.addData("Currently at",  " at %7d",
                eMotor.getCurrentPosition());
        telemetry.addData("Currently at",  " at %7d",
                pivot.getCurrentPosition());
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

        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftRear.setPower(v3);
        rightRear.setPower(v4);
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