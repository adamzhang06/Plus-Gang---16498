package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class AutoTest extends LinearOpMode {
    public static double DISTANCE = 60; // in

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory part1 = drive.trajectoryBuilder(new Pose2d())
                //40 = 25

                .lineTo(
                        new Vector2d(45.8, 0),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory part2 = drive.trajectoryBuilder(new Pose2d())
                .lineTo(
                        new Vector2d(-15, 0),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory part3 = drive.trajectoryBuilder(new Pose2d())
                .strafeTo(
                        new Vector2d(15, 50)
                )
                .build();

        Trajectory part4 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(
                        new Pose2d(0, 0, Math.toRadians(90))
                )
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("startX", poseEstimate.getX());
        telemetry.addData("startY", poseEstimate.getY());
        telemetry.addData("startHeading", poseEstimate.getHeading());
        telemetry.update();

        drive.followTrajectory(part1);
        drive.followTrajectory(part2);
        drive.followTrajectory(part3);
        drive.followTrajectory(part4);

        poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
