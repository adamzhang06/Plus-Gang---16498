package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Vector;


/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "RedRight_PixelCenter", group = "4RedRight")
public class RedRight_PixelCenter extends LinearOpMode {
    // initialize motors
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    private DcMotor armMotor;
    private Servo grabberServo;
    private Servo wristServo;

    // huskylens constants
    private HuskyLens huskyLens;

    //timer
    public ElapsedTime mClock = new ElapsedTime();
    String spikeZone = "";
    @Override
    public void runOpMode() throws InterruptedException {

        // define motors
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        grabberServo = hardwareMap.get(Servo.class, "grabberServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo sideWristServo = hardwareMap.get(Servo.class, "sideWristServo");
        Servo sideGrabberServo = hardwareMap.get(Servo.class, "sideGrabberServo");
        sideWristServo.setPosition(0);



        //define huskylens
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        telemetry.update();


        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        //calling tuned roadrunner
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //define trajectories

        int speed = 25; // inches per second
        int slow = 20;
        int armHeight = -5025; //TODO used to be -5150

        Pose2d startPose = new Pose2d(0,0,0);

        drive.setPoseEstimate(startPose);

        // TODO change forward and back to lines so it uses the PID

//spikeLeft
        TrajectorySequence spikeLeft = drive.trajectorySequenceBuilder()
                .lineToLinearHeading(
                        new Pose2d(19, -5, Math.toRadians(60)),
                        SampleMecanumDrive.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .forward(9,
                        SampleMecanumDrive.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .back(9,
                        SampleMecanumDrive.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .lineToLinearHeading(
                        new Pose2d(31, -33, Math.toRadians(-90)),
                        SampleMecanumDrive.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

//spikeLeftBackUp
        TrajectorySequence spikeLeftBackUp = drive.trajectorySequenceBuilder(spikeLeft.end())
                .back(3,
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

//spikeLeftPark
        TrajectorySequence spikeLeftPark = drive.trajectorySequenceBuilder(spikeLeftBackUp.end())
                .lineToConstantHeading(
                        new Vector2d(4, -33),
                        SampleMecanumDrive.getVelocityConstraint(slow, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

//spikeCenter
        TrajectorySequence spikeCenter = drive.trajectorySequenceBuilder()
                .lineToLinearHeading(
                        new Pose2d(27.5, 0, 0),
                        SampleMecanumDrive.getVelocityConstraint(slow, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .lineToLinearHeading(
                        new Pose2d(21, 0, 0),
                        SampleMecanumDrive.getVelocityConstraint(slow, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .lineToLinearHeading(
                        new Pose2d(26, -32.2, Math.toRadians(-90)),
                        SampleMecanumDrive.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

//spikeCenterBackUp
        TrajectorySequence spikeCenterBackUp = drive.trajectorySequenceBuilder(spikeCenter.end())
                .back(3,
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

//spikeCenterPixel
        TrajectorySequence spikeCenterPixel = drive.trajectorySequenceBuilder(spikeCenterBackUp.end())
                .lineToLinearHeading(
                        new Pose2d(29.25, 60, Math.toRadians(-90)),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

//spikeCenterPixelPickUp
        TrajectorySequence spikeCenterPixelPickUp = drive.trajectorySequenceBuilder(spikeCenterPixel.end())
                .lineToLinearHeading(
                        new Pose2d(29, 71.5, Math.toRadians(-90)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

//spikeCenterPixelPark
        TrajectorySequence spikeCenterPixelPark = drive.trajectorySequenceBuilder(spikeCenterPixelPickUp.end())
                .lineToLinearHeading(
                        new Pose2d(29, -31, Math.toRadians(-90))
                )
                .turn(Math.toRadians(180))
                .build();

//spikeRight
        TrajectorySequence spikeRight = drive.trajectorySequenceBuilder()
                .splineToConstantHeading(
                        new Vector2d(21.5, -10.5), 0,
                        SampleMecanumDrive.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .lineToConstantHeading(
                        new Vector2d(16.5, -10.5),
                        SampleMecanumDrive.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .lineToLinearHeading(
                        new Pose2d(17.7, -33, Math.toRadians(-90)),
                        SampleMecanumDrive.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

//spikeRightBackUp
        TrajectorySequence spikeRightBackUp = drive.trajectorySequenceBuilder(spikeRight.end())
                .back(3,
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

//spikeRightPark
        TrajectorySequence spikeRightPark = drive.trajectorySequenceBuilder(spikeRightBackUp.end())
                .lineToConstantHeading(
                        new Vector2d(4, -33),
                        SampleMecanumDrive.getVelocityConstraint(slow, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        wristServo.setPosition(1);
        sleep(500);

        //start is clicked
        waitForStart();

        if (isStopRequested()) return;

        //start color detect (algorithm defined earlier)
        //define x_pos
        int x_pos = -1; //TODO change back to -1

        while (opModeIsActive()) {

            mClock.reset();

            //Get label start
            while (x_pos == -1 && mClock.seconds() < 5) { // mClock.seconds
                telemetry.addData("mClock: ", mClock.seconds());
                telemetry.update();
                HuskyLens.Block[] blocks = huskyLens.blocks();

                if (blocks.length > 0) {
                    telemetry.addData("Block count", blocks.length);

                    for (int i = 0; i < blocks.length; i++) {

                        if (blocks[i].id == 1 && blocks[i].width > 15) {
                            //TODO id #
                            x_pos = blocks[i].x;
                            telemetry.addData("x_pos: ", x_pos);
                        } //end assigning x_pos
                    } //end getting block id
                } // end getting blocks
            } //end whileTimer

            break; //break whileOpMode to start autonomous
        } //end whileOpMode ^

        //define spikeZone based on x_pos
        //The HuskyLens device screen is 320 x 240 pixels, with center at position (160, 120).
        if (x_pos > 0 && x_pos < 107) {
            spikeZone = "left";
            telemetry.addData("spikeZone: ", spikeZone);
            telemetry.update();

            //run trajectoryLeft
            drive.followTrajectorySequence(spikeLeft);

            grabberServo.setPosition(0.2);
            sleep(500);

            //arm up
            while(armMotor.getCurrentPosition() >= armHeight) {
                armMotor.setPower(-0.5);
                telemetry.addData("position: ", armMotor.getCurrentPosition());
                telemetry.update();
            }
            armMotor.setPower(0);
            sleep(500);

            grabberServo.setPosition(0.5);
            sleep(500);

            //back up
            drive.followTrajectorySequence(spikeLeftBackUp);

            //arm down
            while(armMotor.getCurrentPosition() <= -750) {
                armMotor.setPower(1);
                telemetry.addData("position: ", armMotor.getCurrentPosition());
                telemetry.update();
            }
            armMotor.setPower(0);

            //park
            drive.followTrajectorySequence(spikeLeftPark);
        }

        if (x_pos >= 107 && x_pos <= 213) {
            spikeZone = "center";
            telemetry.addData("spikeZone: ", spikeZone);
            telemetry.update();

            //run trajectoryCenter
            drive.followTrajectorySequence(spikeCenter);

            grabberServo.setPosition(0.2);
            sleep(250);

            //arm up
            while(armMotor.getCurrentPosition() >= armHeight) {
                armMotor.setPower(-0.5);
                telemetry.addData("position: ", armMotor.getCurrentPosition());
                telemetry.update();
            }
            armMotor.setPower(0);

            grabberServo.setPosition(0.5);
            sleep(500);

            //back up
            drive.followTrajectorySequence(spikeCenterBackUp);

            //arm down
            while(armMotor.getCurrentPosition() <= -750) {
                armMotor.setPower(1);
                telemetry.addData("position: ", armMotor.getCurrentPosition());
                telemetry.update();
            }
            armMotor.setPower(0);

            //pixel
            drive.followTrajectorySequence(spikeCenterPixel);

            wristServo.setPosition(0);
            sleep(250);

            while(armMotor.getCurrentPosition() >= -450) {
                armMotor.setPower(-0.5);
                telemetry.addData("position: ", armMotor.getCurrentPosition());
                telemetry.update();
            }
            armMotor.setPower(0);

            drive.followTrajectorySequence(spikeCenterPixelPickUp);

            while(armMotor.getCurrentPosition() <= -150) {
                armMotor.setPower(1);
                telemetry.addData("position: ", armMotor.getCurrentPosition());
                telemetry.update();
            }
            armMotor.setPower(0);
            sleep(500);

            grabberServo.setPosition(0.2);
            sleep(750);

            while(armMotor.getCurrentPosition() >= -400) {
                armMotor.setPower(-0.5);
                telemetry.addData("position: ", armMotor.getCurrentPosition());
                telemetry.update();
            }
            armMotor.setPower(0);
            sleep(150);

            drive.followTrajectorySequence(spikeCenterPixelPark);

            wristServo.setPosition(0.5);
            sleep(150);

            grabberServo.setPosition(0.5);

        }

        if (x_pos > 213) {
            spikeZone = "right";
            telemetry.addData("spikeZone: ", spikeZone);
            telemetry.update();

            //run trajectoryRight
            drive.followTrajectorySequence(spikeRight);

            grabberServo.setPosition(0.2);
            sleep(500);

            //arm up
            while(armMotor.getCurrentPosition() >= armHeight) {
                armMotor.setPower(-0.5);
                telemetry.addData("position: ", armMotor.getCurrentPosition());
                telemetry.update();
            }
            armMotor.setPower(0);
            sleep(500);

            grabberServo.setPosition(0.5);
            sleep(500);

            //back up
            drive.followTrajectorySequence(spikeRightBackUp);

            //arm down
            while(armMotor.getCurrentPosition() <= -750) {
                armMotor.setPower(1);
                telemetry.addData("position: ", armMotor.getCurrentPosition());
                telemetry.update();
            }
            armMotor.setPower(0);

            //park
            drive.followTrajectorySequence(spikeRightPark);
        }

        //if nothing is detected go to the center spike
        if (x_pos == -1) {
            spikeZone = "left";
            telemetry.addData("spikeZone: ", spikeZone);
            telemetry.update();

            //run trajectoryCenter
            drive.followTrajectorySequence(spikeLeft);

            grabberServo.setPosition(0.2);
            sleep(500);

            //arm up
            while(armMotor.getCurrentPosition() >= armHeight) {
                armMotor.setPower(-0.5);
                telemetry.addData("position: ", armMotor.getCurrentPosition());
                telemetry.update();
            }
            armMotor.setPower(0);
            sleep(500);

            grabberServo.setPosition(0.5);
            sleep(500);

            //back up
            drive.followTrajectorySequence(spikeLeftBackUp);

            //arm down
            while(armMotor.getCurrentPosition() <= -750) {
                armMotor.setPower(1);
                telemetry.addData("position: ", armMotor.getCurrentPosition());
                telemetry.update();
            }
            armMotor.setPower(0);

            //park
            drive.followTrajectorySequence(spikeLeftPark);
        }
        //end color detect

        //one trajectory to start tag detect

        //arm movement before tag driving

        //start tag detect
        int tagID = 0;

        //switch to TAG_RECOGNITION
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        telemetry.update();

        //get ID # for tag to detect
        if (spikeZone == "left") {
            tagID = 4;
        }
        if (spikeZone == "center") {
            tagID = 5;
        }
        if (spikeZone == "right") {
            tagID = 6;
        }

        //trajectories for moving left and right
//        Trajectory slowLeft = drive.trajectoryBuilder(new Pose2d())
//                .strafeLeft(.5,
//                        SampleMecanumDrive.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
//                )
//                .build();
//
//        Trajectory slowRight = drive.trajectoryBuilder(new Pose2d())
//                .strafeRight(.5,
//                        SampleMecanumDrive.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
//                )
//                .build();
//
//        //define final_x_pos
//        int final_x_pos = -1;
//
//        //go to correct final_x_pos
//        while (opModeIsActive()) {
//
//            mClock.reset();
//
//            //Get label start
//            while ((final_x_pos <= 145 || final_x_pos >= 175) && mClock.seconds() < 5) {
//                HuskyLens.Block[] blocks = huskyLens.blocks();
//                telemetry.addData("final_x_pos: ", final_x_pos);
//                telemetry.update();
//
//                if (blocks.length > 0) {
//                    telemetry.addData("Block count", blocks.length);
//
//                    //x_pos = 160 is center
//                    for (int i = 0; i < blocks.length; i++) {
//
//                        if (blocks[i].id == tagID && blocks[i].x <= 150) {
//                            //drive left
//                            while (blocks[i].id == tagID && blocks[i].x <= 150){
//                                drive.followTrajectory(slowLeft);
//                            }
//
//
//                            final_x_pos = blocks[i].x;
//                            telemetry.addData("move left, ", final_x_pos);
//                        }
//
//                        if (blocks[i].id == tagID && blocks[i].x >= 170) {
//                            //drive right
//                            while (blocks[i].id == tagID && blocks[i].x >=170) {
//                                drive.followTrajectory(slowRight);
//                            }
//
//                            final_x_pos = blocks[i].x;
//                            telemetry.addData("move right, ", final_x_pos);
//
//                        }
//                    }
//                }
//            }
//
//            if (final_x_pos == -1 && tagID == 1) {
//                //strafe left
//            }
//
//            if (final_x_pos == -1 && tagID == 2) {
//                //do nothing
//            }
//
//            if (final_x_pos == -1 && tagID == 3) {
//                //strafe right
//            }
//
//
//            break; //break whileOpMode (for x) to continue on to y_pos
//        } //end whileOpMode ^
//
//        //define final_y_pos
//        int final_y_pos = -1;
//
//        //go to correct final_y_pos
//        while (opModeIsActive()) {
//            //y_pos = 165, 240 is bottom, 120 is center, 0 is top
//            while (final_y_pos < 165) {
//                HuskyLens.Block[] blocks = huskyLens.blocks();
//                telemetry.addData("final_y_pos: ", final_y_pos);
//                telemetry.update();
//
//                if (blocks.length > 0) {
//                    telemetry.addData("Block count", blocks.length);
//
//                    //x = 160 is center
//                    for (int i = 0; i < blocks.length; i++) {
//                        telemetry.addData("final_y_pos: ", blocks[i].y);
//                        if (blocks[i].id == tagID && blocks[i].y < 170) {
//                            //drive forward
//
//
//
//
//                            final_y_pos = blocks[i].y;
//                            telemetry.addData("move forward, ", final_y_pos);
//                            telemetry.update();
//                        } // end forward movement to correct final_y_pos
//                    } // for (int...
//                } // block.length
//            } // while (final_y_pos <...
//            break; //break whileOpMode (for y)
//        } //end whileOpMode ^
        //end tag detect

        //drop pixel

        //parking





    } //end runOpMode
} //end LinearOpMode