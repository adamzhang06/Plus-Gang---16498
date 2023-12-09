package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "HuskyTagTest")
public class HuskyTagTest extends LinearOpMode {

    // huskylens constants
    private HuskyLens huskyLens;
    
    //no timer needed
    String spikeZone = "left";
    int tagID = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        //define huskylens
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        telemetry.update();


        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        //calling tuned roadrunner
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //define trajectories


        //start is clicked
        waitForStart();

        if (isStopRequested()) return;

        if (spikeZone == "left") {
            tagID = 1;
        }
        if (spikeZone == "center") {
            tagID = 2;
        }
        if (spikeZone == "right") {
            tagID = 3;
        }

        int final_x_pos = -1;
        int final_y_pos = -1;

        while (opModeIsActive()) {

            //Get label start
            while (final_x_pos <= 145 || final_x_pos >= 175) {
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
            break; //break whileOpMode to start autonomous
        } //end whileOpMode ^

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
                        }
                    }
                }
            }
            break;
        }


        telemetry.addData("final_x_pos: ", final_x_pos);
        telemetry.addData("final_y_pos: ", final_y_pos);
        telemetry.update();
        sleep(10000);

    } //end runOpMode
} //end LinearOpMode
