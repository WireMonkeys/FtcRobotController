package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.geometry.Pose2d;

@TeleOp public class Odometry extends LinearOpMode {

    private MotorEx leftEncoder, rightEncoder, perpEncoder;
    private HolonomicOdometry odometry;
    private Pose2d robotPose;

    public static final double TRACKWIDTH = 13;
    public static final double CENTER_WHEEL_OFFSET = 0.477;
    public static final double WHEEL_DIAMETER = 2.0;
    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 4092;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;



    @Override
    public void runOpMode() throws InterruptedException {
        leftEncoder = new MotorEx(hardwareMap, "backLeft");
        rightEncoder = new MotorEx(hardwareMap, "backRight");
        perpEncoder = new MotorEx(hardwareMap, "frontRight");

        leftEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        perpEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        odometry = new HolonomicOdometry(
                leftEncoder::getDistance,
                rightEncoder::getDistance,
                perpEncoder::getDistance,
                TRACKWIDTH,
                CENTER_WHEEL_OFFSET
        );

        robotPose = odometry.getPose();
        odometry.updatePose(robotPose);
        telemetry.addData("Robot Position at Init: ",robotPose);
        telemetry.update();

        waitForStart();

        while (!isStopRequested()) {
            // run autonomous

            // update positions
            odometry.updatePose();
            robotPose = odometry.getPose();
            telemetry.addData("Robot Position at Init: ",robotPose);
            telemetry.update();


        }
    }

}