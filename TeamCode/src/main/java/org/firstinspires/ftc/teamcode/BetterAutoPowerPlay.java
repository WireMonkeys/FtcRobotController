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

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

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
@Autonomous(name = "BetterAuto", group = "Autonomous")
public class BetterAutoPowerPlay extends LinearOpMode {

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotor eMotor = null;

    private Servo   wrist;
    private Servo   hand;



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
    private static final String VUFORIA_KEY =
            "ATxVdFX/////AAABmUqFC7h7TU83t9WFcVjmUiEzW+zOjHGZGAwJ33FVK38O8XGYLa/YyC3dwPGeIanbvCyJK4rZ8Is4/DidvCGHe4bcz0Co0jNoQUTg+bDU3jfaYBJjUeSIKwJGTAYOqHQQH4tvrxvyEh/zqAA6Jrw1V3qxFW7m2lZTnn6onm0lXVX2/io8Smll63aSFEBdtVA25j2IxC8it5RvxGIcpxbvaQ8bkQjXvOdlBVGgLZj4y2DH3CzNKLwGth4vR5nsQOiGO0OT9rV/2TbpDgMWkc1WY475sJ5AlH6L3mR0bFHrgyGGhW8lmJashJhDtdt5Axb9Z7l9Ix+BEU2GC3oBGT/MvfOG2OgqEYuKLoTLIiV/i1ot";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {



        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        initRobot();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        /** Wait for the game to begin */

        hand.setPosition(0.575);

        waitForStart();




        if (opModeIsActive()) {
            int object = 0;
            while (opModeIsActive() && object == 0) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            if(recognition.getLabel() == "1 Bolt"){
                                object = 1;
                            }

                            if(recognition.getLabel() == "2 Bulb"){
                                object = 2;
                            }

                            if(recognition.getLabel() == "3 Panel"){
                                object = 3;
                            }

                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );

                        }
                        telemetry.update();
                    }
                }


            }
            if(object == 1){

                drive(0.0,0.5,0.0);
                sleep(800);

                drive(0.0,0.0,0.0);

                moveArm(1800);
                sleep(1500);

                drive(0.25,0.0,0.0);
                sleep(650);

                drive(0.0,0.0,0.0);

                hand.setPosition(0.3096);
                sleep(1000);

                drive(-0.25,0.0,0.0);
                sleep(500);

                drive(0.0,-0.5,0.0);
                sleep(2800);

                drive(0.0,0.0,0.5);
                sleep(50);

                drive(0.5,0.0,0.0);
                sleep(1600);

                drive(0.0,0.25,0.25);
                sleep(1000);

                drive(0.25,0.0,0.0);
                sleep(700);

                drive(0.0,0.0,0.0);

                moveArm(850);
                sleep(900);

                hand.setPosition(0.656);
                sleep(500);

                drive(0.0,0.0,0.0);
                sleep(10000);



            }
            if(object == 2){

                hand.setPosition(0.6);
                pidDrive(0.0,0.75,-0.01,1000);
                moveArm(4088);
                pidDrive(0.75,0.0,0.0, -2000);
                pidDrive(0.0,0.0,0.25,290);
                pidDrive(0.5,0.0,0.0,-290);

                sleep(1000);

                hand.setPosition(0.3095);
                sleep(1000);

                pidDrive(0.5,0.0,0.0,175);

                moveArm(650);

                pidDrive(0.0,0.0,0.25,550);
                pidDrive(0.5,0.0,0.0025,-1875);

                hand.setPosition(0.6);
                sleep(500);

                moveArm(4088);
                sleep(1000);
//
//                pidDrive(0.5,0.0,0.0,1800);
//                pidDrive(0.0,0.0,0.25,-585);
//                pidDrive(0.5,0.0,0.0,-100);
//
//                hand.setPosition(0.3095);
//                sleep(1000);




            }
            if(object == 3){

                pidDrive(0.0,0.5,-0.025,-300);
                pidDrive(0.5,0.0,0.0,-2500);
                moveArm(-4000);
                pidDrive(0.5,0.0,0.0,250);
                pidDrive(0.0,0.0,0.25,-230);
                pidDrive(0.5,0.0,0.0,-250);
                sleep(3000);
                hand.setPosition(0.3095);
                sleep(3000);
                moveArm(0);
                pidDrive(0.5,0.0,0.0,275);
                pidDrive(0.0,0.0,0.25,400);
                pidDrive(0.0,0.5,0.0,1200);


            }
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.60f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
    private void initRobot() {
        leftFront  = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        leftRear  = hardwareMap.get(DcMotor.class, "backLeft");
        rightRear = hardwareMap.get(DcMotor.class, "backRight");
        eMotor = hardwareMap.get(DcMotor.class, "eMotor");
        hand = hardwareMap.get(Servo.class, "hand");
        wrist = hardwareMap.get(Servo.class, "wrist");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        eMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        eMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


//        eMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        eMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void moveArm(int eMotors) {


        eMotors = Range.clip(eMotors, -4088, 0);

        eMotor.setTargetPosition(eMotors);

        eMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        eMotor.setPower(Math.abs(0.75));
    }

    private void pidDrive (double forward, double strafe, double turn, int target){
        /*

         * Proportional Integral Derivative Controller

         */

       double Kp = -0.5;
       double Ki = 0;
       double Kd = 0;

       int reference = rightFront.getCurrentPosition() + target;

       double integralSum = 0;

       int lastError = 0;

       boolean setPointIsNotReached = true;

// Elapsed timer class from SDK, please use it, it's epic
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && setPointIsNotReached) {


            // obtain the encoder position
            int encoderPosition = rightFront.getCurrentPosition();
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

            telemetry.addData("encoderPos", rightFront.getCurrentPosition());
            telemetry.update();


            // reset the timer for next time
            timer.reset();

            setPointIsNotReached = Math.abs(error) > 10;

        }
        drive(0.0,0.0,0.0);

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



        telemetry.addData("frontMotors", "left (%.2f), right (%.2f)", v1, v2);
        telemetry.addData("backMotors", "left (%.2f), right (%.2f)", v3, v4);
        telemetry.update();

    }
}
