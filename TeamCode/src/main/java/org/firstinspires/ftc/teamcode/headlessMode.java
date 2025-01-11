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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;


@TeleOp(name="HeadlessMode", group="DriveBase")

public class headlessMode extends OpMode
{
    // Declare OpMode members.
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

    BNO055IMU imu;


    Orientation angles;



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



//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);

        initIMU();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".



        // Set up our telemetry dashboard




    }
    private void initIMU(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
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


    double intakeOne = 0.5;
    double intakeTwo = 0.5;
    double dropping = 0.16;
    int eMotors = 0;
    int viperd = 0;
    Double slide = 0.0;
    int swivelI = 0;
    int mode = 1;


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        intakeDos.setPosition(intakeTwo);
        intakeUno.setPosition(intakeOne);
        drop.setPosition(dropping);
        dropping = Range.clip(dropping, 0.0d, 1.0d);

        if (gamepad1.a){
            dropping = 0.15;
        }
        else if (gamepad1.b){
            dropping = 0.355;
        }

        if (gamepad2.a){
            swivelI = -650;
        }
        else if (gamepad2.b){
            swivelI = -520;
        }
        else if (gamepad2.y){
            swivelI = -10;
        }




        if (gamepad2.right_bumper){
            intakeTwo = 1.0;
            intakeOne = 0.0;
        }
        else if (gamepad2.left_bumper){
            intakeTwo = 0.0;
            intakeOne = 1.0;
        } else{
            intakeOne = 0.5;
            intakeTwo = 0.5;
        }
        if (gamepad2.right_stick_button){
            viperd = 4000;
        }
        else if (gamepad2.left_stick_button){
            viperd = 0;
            dropping = 0.15;
        }




        if(gamepad2.dpad_left){
            slide = slide - 20;
        }
        else if(gamepad2.dpad_right){
            slide = slide + 20;
        }
        else if(gamepad2.x){
            slide = 0.0;
        }


        if(gamepad2.dpad_up){
            swivelI = swivelI + 35;
        }
        else if(gamepad2.dpad_down){
            swivelI = swivelI - 35;
        }






        slides.setTargetPosition(slide.intValue());

        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slides.setPower(Math.abs(1.0));

        slide = Range.clip(slide,0, 1600);


        viperd = Range.clip(viperd,0, 4000);

        viper.setTargetPosition(viperd);

        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        viper.setPower(Math.abs(1.0));


        swivel.setTargetPosition(swivelI);

        swivel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        swivel.setPower(Math.abs(1.0));


        telemetry.addData("Currently at",  " at %7d",
                slides.getCurrentPosition());

        telemetry.addData("Currently at", "at %7d",
        swivel.getCurrentPosition());

        telemetry.addData("Currently at",  " at %7d",
                eMotors);

        telemetry.addData("drop pos",drop.getPosition());

        if (gamepad1.start && gamepad1.back){
            mode = 2;
        }
        else if (gamepad2.start && gamepad2.back){
            initIMU();
            mode = 1;
        }



        // Get the current heading (yaw/z-axis rotation)
        double currentHeading = getHeading();


        if (mode == 1) {


            drive(gamepad1.left_stick_y * Math.cos(Math.toRadians(currentHeading)) + gamepad1.left_stick_x * Math.sin(Math.toRadians(currentHeading)), -(gamepad1.left_stick_x * Math.cos(Math.toRadians(currentHeading)) - gamepad1.left_stick_y * Math.sin(Math.toRadians(currentHeading))), gamepad1.right_stick_x / 1.25);
        } else {
            drive(gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);
        }




        // Calculate how much the Control Hub has turned relative to the start
        double turnAmount = currentHeading - initialHeading;

        // If the angle is less than -180 or greater than 180, adjust it to stay within -180 to 180 degrees
        if (turnAmount > 180) turnAmount -= 360;
        if (turnAmount < -180) turnAmount += 360;

        // Display the heading and the amount of turn
        telemetry.addData("Heading (Z-axis)", currentHeading);
        telemetry.addData("Turn Amount", turnAmount);
        telemetry.update();





    }

    double initialHeading = getHeading();

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




//        @SuppressWarnings("all")
        public double getHeading () {
        double heading = 0;

        // Attempt to get the angular orientation
           if(imu == null){
               return 0;
           }
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Check if angles is not null to avoid a NullPointerException
        if (angles != null) {
            heading = angles.firstAngle;
        } else {
            // If angles is null, provide a fallback value or take some action
            telemetry.addData("Error", "IMU data unavailable");
            telemetry.update();
        }


        return heading;
    }




    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


    }

