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
@Autonomous(name = "HuskyColorTest")
public class HuskyColorTest extends LinearOpMode {

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

        int x_pos = -1;

        while (opModeIsActive()) {

            mClock.reset();

            //Get label start
            while (x_pos == -1 && mClock.seconds() < 15) { // mClock.seconds
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
        }

        if (x_pos >= 107 && x_pos <= 213) {
            spikeZone = "center";
        }

        if (x_pos > 213) {
            spikeZone = "right";
        }

        if (x_pos == -1) {
            spikeZone = "center";
        }

        telemetry.addData("spikeZone: ", spikeZone);
        telemetry.update();
        sleep(10000);

    } //end runOpMode
} //end LinearOpMode
