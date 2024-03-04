package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous
public class JoDadaBlue extends LinearOpMode {

    OpenCvWebcam webcam1 = null;
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
    boolean center = false;
    boolean right = false;
    boolean left = false;


    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST = 18;// Tag ID 18 from the 36h11 family

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {

        initRobot();
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam1");
        int cameraMonitorViewId= hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam1.setPipeline(new examplePipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                webcam1.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode){

            }
        });
        waitForStart();
        webcam1.closeCameraDevice();
        aprilTagInit();
        if (center){
            telemetry.addLine("center");
            pidDrive(0.5,0.0,0.0, -1400);
            sleep(100);
            intake.setPower(-0.2);
            sleep(1000);
            pidDrive(0.5,0.0,0.0,300);
            pidDrive(0.0,0.75,0.0,-200);
            pidDrive(0.0,0.0,0.5,-800);
            runAprilTag(3);
            sleep(1000);
            wrist.setPosition(0.68);
            sleep(1000);
            hand.setPosition(0.6);
            sleep(1000);
            pidDrive(0.0,1.0,0.0,1000);

        }
        else if (left){
            telemetry.addLine("Left");
            pidDrive(0.5,0.0,0.0, -800);
            sleep(100);
            pidDrive(0.0,0.0,0.5,-450);
            pidDrive(0.3,0.0,0.0,-425);
            intake.setPower(-0.2);
            sleep(1000);
            pidDrive(0.5,0.0,0.0,425);
            pidDrive(0.0,0.0,0.5,-400);
            runAprilTag(2);
            sleep(1000);
            wrist.setPosition(0.67);
            sleep(1000);
            hand.setPosition(0.61679);
            sleep(1000);
            pidDrive(0.0,1.0,0.0,-500);

        }
        else if (right){
            telemetry.addLine("right");
            pidDrive(1.0,0.0,0.0, -800);
            sleep(100);
            pidDrive(0.0,0.0,0.5,250);
            pidDrive(0.3,0.0,0.0,-400);
            intake.setPower(-0.2);
            sleep(2000);
            pidDrive(0.5,0.0,0.0,500);
            pidDrive(0.0,0.5,0.0,-200);
            pidDrive(0.0,0.0,0.5,-1400);
            runAprilTag(3);
            sleep(100);
            pidDrive(-0.05,0.75,0.0,300);
            sleep(1);
            wrist.setPosition(0.67);
            sleep(1000);
            hand.setPosition(0.61679);
            sleep(1000);
            pidDrive(0.0,1.0,0.0,200);

        }
    }






    class examplePipeline extends OpenCvPipeline {
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        double leftavgfin;
        double rightavgfin;
        Mat outPut = new Mat();
        Scalar rectColor= new Scalar(255.0, 0.0, 0.0);


        public Mat processFrame(Mat input){

            Imgproc.cvtColor(input,YCbCr,Imgproc.COLOR_RGB2YCrCb);

            Rect leftRect = new Rect(80, 200, 100, 50);
            Rect rightRect = new Rect(400, 190, 100, 50);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, leftRect, rectColor, 2);
            Imgproc.rectangle(outPut, rightRect, rectColor, 2);

            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);

            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgfin = leftavg.val[0];
            rightavgfin = rightavg.val[0];



            if (leftavgfin  > 140){
                telemetry.addLine("LEFT!!!");
                center = true;
                right = false;
                left = false;
            }
            else if (rightavgfin > 140){
                telemetry.addLine("RIGHT!!!");
                right = true;
                center = false;
                left = false;
            }
            else {
                telemetry.addLine("BOOOOOOO!!!");
                left = true;
                right = false;
                center = false;
            }
            telemetry.addData("Right",  "%f", rightavgfin);
            telemetry.addData("Left",  "%f", leftavgfin);


            return(outPut);

        }
    }
    public void drive(double forward, double strafe, double turn) {
        double r = Math.hypot(strafe, forward);
        double robotAngle = Math.atan2(forward, strafe) - Math.PI / 4;
        double rightX = turn;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        leftFront.setPower(v1 / -1.25);
        rightFront.setPower(v2 / -1.25);
        leftRear.setPower(v3 / -1.25);
        rightRear.setPower(v4 / -1.25);


    }

    private void runAprilTag (int desiredTag){
        boolean done = false;
        while (!isStopRequested() && !done)
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == desiredTag)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                    AprilTagPose pose = getFTCPose(tagOfInterest);
                    telemetry.addData("range", "%f", pose.range);
                    telemetry.addData("Bearing", "%f", pose.bearing);
                    done = aprilTagMove(0.5,0.2,0.175, 0.45, pose);
                }
                else {
                    drive(0.0,0.0,0.0);
                    telemetry.addLine("tag not found :(");
                }

            }

            else{
                drive(0.0,0.0,0.0);
                telemetry.addLine("no detections");
            }

            telemetry.update();
            sleep(20);
        }
    }

    private void aprilTagInit(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        webcam1.setPipeline(aprilTagDetectionPipeline);
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam1.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    private AprilTagPose getFTCPose (AprilTagDetection detection){
        if (detection.pose != null)   {
            AprilTagPose ftcPose = new AprilTagPose();

            ftcPose.x =  detection.pose.x;
            ftcPose.y =  detection.pose.z;
            ftcPose.z = -detection.pose.y;

            AngleUnit outputUnitsAngle = AngleUnit.DEGREES;
            Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, outputUnitsAngle);
            ftcPose.yaw = -rot.firstAngle;
            ftcPose.roll = rot.thirdAngle;
            ftcPose.pitch = rot.secondAngle;

            ftcPose.range = Math.hypot(ftcPose.x, ftcPose.y);
            ftcPose.bearing = outputUnitsAngle.fromUnit(AngleUnit.RADIANS, Math.atan2(-ftcPose.x, ftcPose.y));
            ftcPose.elevation = outputUnitsAngle.fromUnit(AngleUnit.RADIANS, Math.atan2(ftcPose.z, ftcPose.y));

            return ftcPose;
        }
        return null;
    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    private void initRobot(){
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

        while (setPointIsNotReached) {


            // obtain the encoder position
            int encoderPosition = rightFront.getCurrentPosition();
            // calculate the error
            int error = reference - encoderPosition;

            // rate of change of the error
            double derivative = (error - lastError) / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());

            double out = (Kp * error);  //+ (Ki * integralSum) + (Kd * derivative);

            out = Range.clip(out, -1, 1);

            drive(forward * out,-strafe * out,-turn * out);

            lastError = error;




            // reset the timer for next time
            timer.reset();

            setPointIsNotReached = Math.abs(error) > 10;


        }
        drive(0.0,0.0,0.0);
    }

    public boolean aprilTagMove (double speed, double turn, double strafe, double destRange,  AprilTagPose pose){
        if (destRange < pose.range){
            double scaledTurn = turn * Range.clip(pose.bearing/10, -1, 1);
            double scaledStrafe = strafe * Range.clip(pose.yaw/5,-1,1);
            drive(speed,scaledStrafe, -scaledTurn);
            return false;
        }
        else {
            drive(0.0,0.0,0.0);
            return true;
        }
    }

    public class AprilTagPose {
        public double x;
        public double y;
        public double z;
        public double yaw;
        public double pitch;
        public double roll;
        public double range;
        public double bearing;
        public double elevation;

        public AprilTagPose() {
        }
    }
}
