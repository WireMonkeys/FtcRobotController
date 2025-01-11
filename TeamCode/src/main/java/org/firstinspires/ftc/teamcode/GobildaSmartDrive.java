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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//These are needed for orientation
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//goBilba Smart Drive Version 3.0.1

@TeleOp(name="GoBilda Smart Drive", group="DriveBase")

public class GobildaSmartDrive extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotor carouselMotor = null;
    private DcMotor emotor = null;

    private Servo   shoulder;
    private Servo   elbow;
    private Servo   wrist;
    private Servo   hand;

    boolean rampUp = true;

    BNO055IMU     robotOrientation;
    Orientation   rotation;

    float initialRotationZ = 0;

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
        emotor = hardwareMap.get(DcMotor.class, "eMotor");
        carouselMotor = hardwareMap.get(DcMotor.class, "carouselMotor");
        shoulder = hardwareMap.get(Servo.class, "shoulder");
        elbow = hardwareMap.get(Servo.class, "elbow");
        wrist = hardwareMap.get(Servo.class, "wrist");
        hand = hardwareMap.get(Servo.class, "hand");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        carouselMotor.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        rotation = robotOrientation.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        telemetry.addData("Robot Rotation", rotation.thirdAngle * 180 / Math.PI + "\u00B0"); // the "\u00B0" adds the degree symbol

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        initialRotationZ = rotation.thirdAngle;
    }
    double carousel = 0.0;

    double shoulderStick = 0.0;
    double elbowStick = 0.46;
    double wristStick = 0.8;
    double handStick = 0.46;

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        float currentRotationZ = rotation.thirdAngle;
        telemetry.addData("Robot Rotation Since Play Hit", (currentRotationZ - initialRotationZ) * 180 / Math.PI + "\u00B0"); // the "\u00B0" adds the degree symbol


        shoulder.setPosition(shoulderStick);
        elbow.setPosition(elbowStick);
        wrist.setPosition(wristStick);
        hand.setPosition(handStick);


        shoulderStick = shoulderStick + (gamepad2.left_stick_y * 0.001);
        elbowStick = elbowStick + (gamepad2.right_stick_y * 0.001);
        wristStick = wristStick + (-gamepad2.left_stick_x * 0.001);
        handStick = handStick + (gamepad2.right_stick_x * 0.001);


        shoulderStick = Range.clip(shoulderStick, 0.0d, 1.0d);
        elbowStick = Range.clip(elbowStick, 0.0d, 1.0d);
        wristStick = Range.clip(wristStick, 0.0d, 1.0d);
        handStick = Range.clip(handStick, 0.0d, 1.0d);

        telemetry.addData("\nShoulder Servo:", shoulderStick);
        telemetry.addData("Elbow Servo:", elbowStick);
        telemetry.addData("Wrist Servo:", wristStick);
        telemetry.addData("Hand Servo:", handStick);


        carouselMotor.setPower(carousel);
        if(gamepad2.right_bumper){
            carousel = -1.0;
        }
        else if(gamepad2.left_bumper){
            carousel = 1.0;
        }
        else {
            carousel = 0.0;
        }

        emotor.setPower(gamepad2.left_trigger * 0.5 - gamepad2.right_trigger * 0.5);


        drive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, initialRotationZ, currentRotationZ);


//         double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
// double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
// double rightX = -gamepad1.right_stick_x;
// final double v1 = r * Math.cos(robotAngle) + rightX;
// final double v2 = r * Math.sin(robotAngle) - rightX;
// final double v3 = r * Math.sin(robotAngle) + rightX;
// final double v4 = r * Math.cos(robotAngle) - rightX;

// leftFront.setPower(v1);
// rightFront.setPower(v2);
// leftRear.setPower(v3);
// rightRear.setPower(v4);
        telemetry.addData("\nStatus", "Run Time: " + runtime.toString());

    }

    public void drive(double forward, double strafe, double turn, float initialRotationZ, float currentRotationZ) {
        double r = Math.hypot(strafe, forward);
        double robotAngle = Math.atan2(forward, strafe) - (Math.PI / 4) - (currentRotationZ - initialRotationZ);
        double rightX = turn;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftRear.setPower(v3);
        rightRear.setPower(v4);
        telemetry.addData("\nMotors", "left (%.2f), right (%.2f)", v1, v2);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}