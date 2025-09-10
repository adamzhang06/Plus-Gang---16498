package org.firstinspires.ftc.teamcode.Test;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.DriveAcceleration;
import org.firstinspires.ftc.teamcode.Components.HorizontalSlide;
import org.firstinspires.ftc.teamcode.Components.MainDrive;
import org.firstinspires.ftc.teamcode.Components.ViperSlide;
import org.firstinspires.ftc.teamcode.Debug.Debug;

@TeleOp(name = "TeleOpTest", group = "Test")
public class TeleOpTest extends LinearOpMode {
    private double targetPower;
    private double increment;
    private double incrementDividend;
    private boolean isAccelerating;
    private long lastUpdateTime;
    private double currentPower;
    private int updateDelay = 10;

    float frontMultiplier = 1;
    float backMultiplier = 1;

    boolean emergencyStop = false;


    int flipDistanceLimit = 550;
    int slideDistanceLimit = 675;

    Debug debug;

    @Override
    public void runOpMode() {
        // Initialize motors
        MainDrive mainDrive = new MainDrive(this);
        HorizontalSlide hSlide = new HorizontalSlide(this, 3, mainDrive);
        ViperSlide viperSlide = new ViperSlide(this);
        DriveAcceleration driveAccel = new DriveAcceleration(this);


        debug = new Debug(this);
        debug.debugMode = true;


        double direction = 1;

        waitForStart();

        while (opModeIsActive() && !emergencyStop) {
            debug.checkDebugButtons(gamepad1);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Log.d("TeleOpTest", "OpMode started");


    }
}