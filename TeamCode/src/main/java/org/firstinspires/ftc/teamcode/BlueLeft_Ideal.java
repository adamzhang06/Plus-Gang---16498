package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "BlueLeft_Ideal", group = "BlueLeft")
public class BlueLeft_Ideal extends LinearOpMode {

    // huskylens constants
    private HuskyLens huskyLens;
    
    //timer
    public ElapsedTime mClock = new ElapsedTime();
        String spikeZone = "";
    @Override
    public void runOpMode() throws InterruptedException {

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





        //start is clicked
        waitForStart();

        if (isStopRequested()) return;

        //start color detect (algorithm defined earlier)
        //define x_pos
        int x_pos = -1;

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
        if (x_pos < 107) {
            spikeZone = "left";
            telemetry.addData("spikeZone: ", spikeZone);
            telemetry.update();
            //run trajectoryLeft
        }

        if (x_pos >= 107 && x_pos <= 213) {
            spikeZone = "center";
            telemetry.addData("spikeZone: ", spikeZone);
            telemetry.update();
            //run trajectoryCenter

        }

        if (x_pos > 213) {
            spikeZone = "right";
            telemetry.addData("spikeZone: ", spikeZone);
            telemetry.update();
            //run trajectoryRight

        }

        //if nothing is detected go to the center spike
        if (x_pos == -1) {
            spikeZone = "center";
            telemetry.addData("spikeZone: ", spikeZone);
            telemetry.update();
            //run trajectoryCenter

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
            tagID = 1;
        }
        if (spikeZone == "center") {
            tagID = 2;
        }
        if (spikeZone == "right") {
            tagID = 3;
        }

        //define final_x_pos
        int final_x_pos = -1;

        //go to correct final_x_pos
        while (opModeIsActive()) {

            mClock.reset();

            //Get label start
            while ((final_x_pos <= 145 || final_x_pos >= 175) && mClock.seconds() < 5) {
                HuskyLens.Block[] blocks = huskyLens.blocks();
                telemetry.addData("final_x_pos: ", final_x_pos);
                telemetry.update();

                if (blocks.length > 0) {
                    telemetry.addData("Block count", blocks.length);

                    //x_pos = 160 is center
                    for (int i = 0; i < blocks.length; i++) {

                        if (blocks[i].id == tagID && blocks[i].x <= 150) {
                            //drive left



                            final_x_pos = blocks[i].x;
                            telemetry.addData("move left, ", final_x_pos);
                        }
                        if (blocks[i].id == tagID && blocks[i].x >= 170) {
                            //drive right



                            final_x_pos = blocks[i].x;
                            telemetry.addData("move right, ", final_x_pos);

                        }
                    }
                }
            }

            if (final_x_pos == -1 && tagID == 1) {
                //strafe left
            }

            if (final_x_pos == -1 && tagID == 2) {
                //do nothing
            }

            if (final_x_pos == -1 && tagID == 3) {
                //strafe right
            }


            break; //break whileOpMode (for x) to continue on to y_pos
        } //end whileOpMode ^

        //define final_y_pos
        int final_y_pos = -1;

        //go to correct final_y_pos
        while (opModeIsActive()) {
            //y_pos = 165, 240 is bottom, 120 is center, 0 is top
            while (final_y_pos < 165) {
                HuskyLens.Block[] blocks = huskyLens.blocks();
                telemetry.addData("final_y_pos: ", final_y_pos);
                telemetry.update();

                if (blocks.length > 0) {
                    telemetry.addData("Block count", blocks.length);

                    //x = 160 is center
                    for (int i = 0; i < blocks.length; i++) {
                        telemetry.addData("final_y_pos: ", blocks[i].y);
                        if (blocks[i].id == tagID && blocks[i].y < 170) {
                            //drive forward




                            final_y_pos = blocks[i].y;
                            telemetry.addData("move forward, ", final_y_pos);
                            telemetry.update();
                        } // end forward movement to correct final_y_pos
                    } // for (int...
                } // block.length
            } // while (final_y_pos <...
            break; //break whileOpMode (for y)
        } //end whileOpMode ^
        //end tag detect

        //drop pixel

        //parking

    } //end runOpMode
} //end LinearOpMode
