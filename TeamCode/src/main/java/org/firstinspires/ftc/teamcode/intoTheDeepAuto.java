/* Copyright (c) 2019 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;






/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "intoTheDeepAuto", group = "Autonomous")
public class intoTheDeepAuto extends LinearOpMode {

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */


    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotor slides = null;
    private DcMotor viper = null;
    private DcMotor swivel = null;

    private Servo intakeUno = null;
    private Servo intakeDos = null;
    private Servo drop = null;



    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */


    @Override
    public void runOpMode() {


        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.

        initRobot();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/




        /** Wait for the game to begin */

        waitForStart();

        /*
        vertical top is 4000

        swivel bottom is -650

        horizontal out is positive
         */


        pidDrive(0.0,0.25,0.3,100);
        pidDrive(0.0,0.0,0.01, 1);
        vert(4000);
        sleep(1750);
        drop.setPosition(0.355);
        sleep(1000);
        drop.setPosition(0.15);
        sleep(1000);
        vert(0);
        pidDrive(0.0,0.5,0.0,500);
        sleep(20000);





    }






    private void initRobot() {
        leftFront  = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        leftRear  = hardwareMap.get(DcMotor.class, "backLeft");
        rightRear = hardwareMap.get(DcMotor.class, "backRight");
        viper = hardwareMap.get(DcMotor.class, "reMotor");
        slides = hardwareMap.get(DcMotor.class, "eMotor");
        swivel = hardwareMap.get(DcMotor.class, "swivel");
        intakeUno = hardwareMap.get(Servo.class, "intakeUno");
        intakeDos = hardwareMap.get(Servo.class, "intakeDos");
        drop = hardwareMap.get(Servo.class, "drop");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        viper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        viper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        swivel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        swivel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        drop.setPosition(0.15);





    }




    private void pidDrive (double forward, double strafe, double turn, int target){
        /*

         * Proportional Integral Derivative Controller

         */

        double Kp = 0.5;
        double Ki = 0;
        double Kd = 0;

        int reference = leftFront.getCurrentPosition() + target;

        double integralSum = 0;

        int lastError = 0;

        boolean setPointIsNotReached = true;

// Elapsed timer class from SDK, please use it, it's epic
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && setPointIsNotReached) {


            // obtain the encoder position
            int encoderPosition = leftFront.getCurrentPosition();
            // calculate the error
            int error = reference - encoderPosition;

            // rate of change of the error
            double derivative = (error - lastError) / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());

            double out = (Kp * error); // + (Ki * integralSum) + (Kd * derivative);

            out = Range.clip(out, -1, 1);

            drive(forward * out,-strafe * out,-turn * out);

            lastError = error;

            telemetry.addData("encoderPos", leftFront.getCurrentPosition());
            telemetry.update();


            // reset the timer for next time
            timer.reset();

            setPointIsNotReached = Math.abs(error) > 10;

        }
        drive(0.0,0.0,0.0);

    }

    private void vert (int vertical){
        viper.setTargetPosition(vertical);

        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        viper.setPower(Math.abs(1.0));
    }

    private void horizon (int horizontal){
        slides.setTargetPosition(horizontal);

        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slides.setPower(Math.abs(1.0));
    }

    private void moveSwivel(int swivels){
        swivel.setTargetPosition(swivels);

        swivel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        swivel.setPower(Math.abs(1.0));
    }




    public void drive(double forward, double strafe, double turn) {
        double r = Math.hypot(strafe, forward);
        double robotAngle = Math.atan2(forward, strafe) - Math.PI / 4;
        double rightX = turn;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        leftFront.setPower(-v1);
        rightFront.setPower(-v2);
        leftRear.setPower(-v3);
        rightRear.setPower(-v4);
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", v1, v2);
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", v1, v2);







    }


}