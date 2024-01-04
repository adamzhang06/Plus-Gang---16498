package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "Main")
public class RoadrunnerTeleOp extends LinearOpMode {
    DcMotor winchMotor;
    Servo launcherServo;
    DcMotor slideMotor;
    DcMotor armMotor;
    Servo grabberServo;
    Servo wristServo;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        winchMotor = hardwareMap.get(DcMotor.class, "winchMotor");
        launcherServo = hardwareMap.get(Servo.class, "launcherServo");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        grabberServo = hardwareMap.get(Servo.class, "grabberServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        int s = 1;
        double wristPos = 0.1;

        int direction = 1;



        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y / s * direction,
                            -gamepad1.left_stick_x / s * direction,
                            -gamepad1.right_stick_x / s / 1.2
                    )
            );


            // --- start of controls ---

            // gamepad 1

            // speed
            if (gamepad1.left_bumper)
                s = 4;
            else if (gamepad1.right_bumper)
                s = 2;
            else
                s = 1;

            // winch
            if(gamepad1.dpad_down) // winch down
                winchMotor.setPower(1);
            else if(gamepad1.dpad_up) // winch up
                winchMotor.setPower(-1);
            else
                winchMotor.setPower(0);

            // switching directions
            if(gamepad1.y)
                direction = 1;
            if(gamepad1.b)
                direction = -1;

            // launcher
            if(gamepad1.back) // launch
                launcherServo.setPosition(1);
            if(gamepad1.start) // reset
                launcherServo.setPosition(0);

            // slide
            if(gamepad1.left_trigger != 0) {
                slideMotor.setPower(-gamepad1.left_trigger);
            }
            else
                slideMotor.setPower(0);
            if(gamepad1.right_trigger != 0) {
                slideMotor.setPower(gamepad1.right_trigger);
            }
            else
                slideMotor.setPower(0);


            // gamepad 2

            // arm
            if(gamepad2.left_trigger != 0) { // arm down
                armMotor.setPower(1);
                telemetry.addData("left trigger: ", -gamepad2.left_trigger);
                telemetry.update();
            }
            else
                armMotor.setPower(0);

            if(gamepad2.right_trigger != 0) { // arm up
                armMotor.setPower(-1);
                telemetry.addData("right trigger: ", gamepad2.right_trigger);
                telemetry.update();
            }
            else
                armMotor.setPower(0);


            // grabber
            if(gamepad2.left_bumper) { // pinch grabber
//                grabberPos -= 0.1;
                grabberServo.setPosition(0.39);
//                telemetry.addData("grabber position: ", grabberPos);
//                telemetry.update();
            }

            if(gamepad2.right_bumper) { // release grabber
//                grabberPos += 0.1;
                grabberServo.setPosition(0.5);
//                telemetry.addData("grabber position: ", grabberPos);
//                telemetry.update();
            }

            // wrist
            if(gamepad2.left_stick_x != 0) {
                wristPos += -gamepad2.left_stick_x;
                if(wristPos < 0)
                    wristPos = 0;
                if(wristPos > 1)
                    wristPos = 1;
                wristServo.setPosition(wristPos);
            }





            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
