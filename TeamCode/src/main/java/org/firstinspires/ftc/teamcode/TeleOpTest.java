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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOpTest", group = "test")

public class TeleOpTest extends LinearOpMode {
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    private DcMotor armMotor;
    private Servo grabberServo;
    private Servo wristServo;
    private Servo launcherServo;
    private DcMotor winchMotor;
    private DcMotor slideMotor;


    @Override
    public void runOpMode() {

        // initialize motors
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

//        wristMotor = hardwareMap.get(DcMotor.class, "wristMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        grabberServo = hardwareMap.get(Servo.class, "grabberServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        winchMotor = hardwareMap.get(DcMotor.class, "winchMotor");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        launcherServo = hardwareMap.get(Servo.class, "launcherServo");

        double grabberPos = 0.1;
        double grabberMinPos = 0.9;
        double grabberMaxPos = 1;

        double wristPos = 0.1;

        double direction = 1;

        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // reverse motors
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // gamepad 1 controls
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1; // Counteract imperfect strafing
//            int direction = 1;
            double rx = gamepad1.right_stick_x;
            if(direction == -1) {
                rx = gamepad1.right_stick_x * -1;
            }
            else {
                rx = gamepad1.right_stick_x;
            }

            if(gamepad1.left_stick_x < 40 && gamepad1.left_stick_x > 60) {
                x = 50;
            }
            if(gamepad1.left_stick_y < 45 && gamepad1.left_stick_y > 55) {
                y = 50;
            }

            int s = 1;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            if (gamepad1.left_bumper)
                s = 2;
            else if (gamepad1.right_bumper)
                s = 4;
            else
                s = 1;

            backLeft.setPower(backLeftPower / s);
            frontLeft.setPower(frontLeftPower / s);

            frontRight.setPower(frontRightPower / s);
            backRight.setPower(backRightPower / s);

            // winch
            if(gamepad1.dpad_down) // winch down
                winchMotor.setPower(-1);
            else if(gamepad1.dpad_up) // winch up
                winchMotor.setPower(1);
            else
                winchMotor.setPower(0);

            // switching directions

            if(gamepad1.y) {
                direction = -1;
                backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
                backRight.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            else if(gamepad1.b) {
                direction = 1;
                backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
                backRight.setDirection(DcMotorSimple.Direction.REVERSE);
            }

            if(gamepad1.back) { // launch
                launcherServo.setPosition(1);
//                sleep(1000);
//                launcherServo.setPosition(0);
            }
            if(gamepad1.start) { // reset
                launcherServo.setPosition(0);
            }

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





            // gamepad 2 controls

            // arm
            if(gamepad2.left_trigger != 0) { // arm up
                armMotor.setPower(-gamepad2.left_trigger);
                telemetry.addData("left trigger: ", -gamepad2.left_trigger);
                telemetry.update();
            }
            else
                armMotor.setPower(0);

            if(gamepad2.right_trigger != 0) { // arm down
                armMotor.setPower(gamepad2.right_trigger);
                telemetry.addData("right trigger: ", gamepad2.right_trigger);
                telemetry.update();
            }
            else
                armMotor.setPower(0);


            // grabber
            if(gamepad2.left_bumper) { // pinch grabber
//                grabberPos -= 0.1;
                grabberServo.setPosition(0);
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
                wristPos += gamepad2.left_stick_x;
                if(wristPos < 0)
                    wristPos = 0;
                if(wristPos > 1)
                    wristPos = 1;
                wristServo.setPosition(wristPos);
            }
        }
    }
}