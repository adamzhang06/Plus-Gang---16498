//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import java.lang.reflect.Array;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import java.util.List;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.CRServo;
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//
//
//@Autonomous
//public class AutoBLUELeft extends LinearOpMode {
//    public ElapsedTime mClock = new ElapsedTime();
//    String position = "";
//
//
//    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
//    //private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/threevideos.tflite";
//
//
//    private static final String[] LABELS = {
//            "1 Bolt",
//            "2 Bulb",
//            "3 Panel"
//    };
//
//    // private static final String[] LABELS = {
//    //         "Blue3",
//    //         "Green2",
//    //         "Red1"
//    // };
//
//    private static final String VUFORIA_KEY =
//            "AedXBGn/////AAABmUXm3AiENU/ssXe+7hwkPHplKaNzh3LR1e8DlK2eePXPzonxD8c7AErUBKe+aXU86yPv0zcgcpMwe8PvkEzwXG1t26bUvgZ1regtWyjj4Oe2xrddT+Twb6rjbmfIMlgGkZALUOMbWwOlaUawO2hZRx1Ayn2V/6RKXbhZLnhm4Malxl8+sHBU8aM5Ti0YjIr+1xMfe9Z1s8S8VkQJJeLSaaq7f0VIPhHYOc4wWG9y/f1vejoHElnKIJzeYFWerxCoMylqqMiiRftTazRvUlWw8I2Z3FvoM2NBthbzM0bqrKu846B/hTB7dkRGtG+3gGPE6duK/HZ5kBs7X1opYYpFH1fOm5VHankFXWeJwfT5QyOY";
//
//    private VuforiaLocalizer vuforia;
//
//    private TFObjectDetector tfod;
//
//    @Override
//    public void runOpMode() {
//
//        initVuforia();
//        initTfod();
//
//        if (tfod != null) {
//            tfod.activate();
//
//            tfod.setZoom(1.0, 16.0/9.0);
//        }
//
//        /** Wait for the game to begin */
//        telemetry.update();
//
//
//        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
//        DcMotor motorBackLeft = hardwareMap.dcMotor.get("backLeft");
//        DcMotor motorFrontRight = hardwareMap.dcMotor.get("frontRight");
//        DcMotor motorBackRight = hardwareMap.dcMotor.get("backRight");
//
//        DcMotor slideMotor = hardwareMap.dcMotor.get("slideMotor");
//        Servo servoCam = hardwareMap.servo.get("cam");
//        int s = 1;
//
//        DcMotor[] motors = {motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight};
//
//        // Reverse motors
//        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
//
//
//        motorFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
//
//
//
//
//        waitForStart();
//
//        double camPos = 0.1;
//        double minPos = 0.1;
//        double maxPos = 1;
//
//
//        slideMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
//        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        String lastRec = "";
//        if (isStopRequested()) return;
//
//        if (opModeIsActive()) {


//            while (opModeIsActive()) {
//
//                servoCam.setPosition(Range.clip(camPos, minPos, maxPos));
//
//                mClock.reset();
//
//
//                //Get label start
//                while (lastRec == "" && mClock.seconds() < 9) { // mClock.seconds
//                    telemetry.addData("mClock: ", mClock.seconds());
//                    telemetry.update();
//                    if (tfod != null) {
//                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//                        if (updatedRecognitions != null) {
//                            telemetry.addData("# Objects Detected", updatedRecognitions.size());
//                            for (Recognition recognition : updatedRecognitions) {
//                                lastRec = recognition.getLabel();
//                                telemetry.addData("Image", lastRec);
//                                telemetry.update();
//                            }
//
//                        }
//
//                    }
//                } //Get label end
//
//                if (lastRec == "") {
//                    lastRec = "2 Bulb";
//                }
//
//                //DRIVE START
//                telemetry.addData("cam", "position=" + camPos);
//                telemetry.update();
//
//                break;
//
//            }

//            telemetry.addData("slide position: ", slideMotor.getCurrentPosition());
//            telemetry.update();
//
//
//            sleep(750);
//            servoCam.setPosition(Range.clip(0.8, minPos, maxPos));
//            sleep(600);
//
//            while(Math.abs(slideMotor.getCurrentPosition()) < 2500) {
//                slideMotor.setPower(75);
//                telemetry.addData("slide position: ", slideMotor.getCurrentPosition());
//                telemetry.update();
//            }
//            slideMotor.setPower(0);
//            sleep(200);
//            drive(0, .20, 1500, motors);
//            sleep(500);
//            drive(0, -.20, 210, motors);
//            sleep(500);
//            drive(.2, 0, 550, motors);
//            sleep(200);
//            drive(0, .20, 180, motors);
//            sleep(500);
//            servoCam.setPosition(Range.clip(0.1, minPos, maxPos));
//            sleep(300);
//            drive(0, -.20, 100, motors);
//            sleep(200);
//
//            while(Math.abs(slideMotor.getCurrentPosition()) > 1500) {
//                slideMotor.setPower(-75);
//                telemetry.addData("slide position: ", slideMotor.getCurrentPosition());
//                telemetry.update();
//            }
//            slideMotor.setPower(0);
//            sleep(100);
//
//            if(lastRec == "1 Bolt") { // left
//                telemetry.addData("object", "bolt");
//                drive(-.2, 0, 2000, motors);
//                sleep(200);
//                drive(0, .2, 115, motors);
//            }
//            else if(lastRec == "2 Bulb") { // center
//                telemetry.addData("object", "bulb");
//                drive(-.2, 0, 800, motors); // 600
//                sleep(200);
//                drive(0, 1, 100, motors);
//            }
//            else if(lastRec == "3 Panel") { // right
//                telemetry.addData("object", "panel");
//                drive(.2, 0, 800, motors); //800
//                sleep(200);
//                drive(0, 1, 115, motors);
//            }
//            else {
//                telemetry.addData("object", "nothing");
//                drive(-.2, 0, 800, motors);
//                sleep(200);
//                drive(0, 1, 100, motors);
//            }
//        }
//    }
//
//    private void initVuforia() {
//
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
//
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//    }
//
//    private void initTfod() {
//        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        tfodParameters.minResultConfidence = 0.75f;
//        tfodParameters.isModelTensorFlow2 = true;
//        tfodParameters.inputSize = 300;
//        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//
//        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
//        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
//        //tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
//    }
//    private void drive(double x, double y, int distance, DcMotor[] motors) {
//        DcMotor motorFrontLeft = motors[0];
//        DcMotor motorBackLeft = motors[1];
//        DcMotor motorFrontRight = motors[2];
//        DcMotor motorBackRight = motors[3];
//
//
//        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        double s = 1.1;
//
//        while(Math.abs(motorFrontLeft.getCurrentPosition()) < distance) {
//            motorFrontLeft.setPower(y + x / s);
//            motorBackLeft.setPower(y - x / s);
//            motorFrontRight.setPower(y - x / s);
//            motorBackRight.setPower(y + x / s);
//
//            telemetry.addData("frontLeft: ", y + x);
//            telemetry.addData("backLeft: ", y - x);
//            telemetry.addData("frontRight: ", y - x);
//            telemetry.addData("backRight: ", y + x);
//            telemetry.update();
//
//        }
//
//        motorFrontLeft.setPower(0);
//        motorBackLeft.setPower(0);
//        motorFrontRight.setPower(0);
//        motorBackRight.setPower(0);
//        resetEncoders(motors);
//
//
//    }
//
//    public void resetEncoders(DcMotor[] motors) {
//        DcMotor motorFrontLeft = motors[0];
//        DcMotor motorBackLeft = motors[1];
//        DcMotor motorFrontRight = motors[2];
//        DcMotor motorBackRight = motors[3];
//
//        motorFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
//        motorBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
//        motorFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
//        motorBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
//
//    }
//}
