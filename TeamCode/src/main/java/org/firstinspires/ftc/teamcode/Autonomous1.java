/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="Autonomous Blue Right ", group="Robot")
public class Autonomous1 extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    private static final double BACK_CONE = 4000;
    private static final double NORMAL_FLIP = 0.52, FLIPPED = 0.403;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    public static final double CONE_OPEN = 0.9;
    public static final double CONE_FOLDED = 0.1;
    ElapsedTime driveTime = new ElapsedTime(), farConeTimer = new ElapsedTime();
    int farConeFlag = 0;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;
    int wheelsPosition = 0;
    Orientation angles;
    ElapsedTime intakeConeTime = new ElapsedTime();
    AprilTagDetection tagOfInterest = null;


    //////////////////Robot Hardware///////////////////////

    // Declare OpMode members for each of the 4 motors.

    YawPitchRollAngles orientation;
    BNO055IMU imu;
    boolean liftBusy = false, scoreConeFirst = true;

    boolean holdingLift = false, firstPass = true;
    PIDController pid = null, rotatePID = null, rotateRobotPID = null, anglePID = null;
    //////////////////////////////////////////

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();





        pid = new PIDController(0.0002, 0, 0.0001);
        rotatePID = new PIDController(0.035, 0.0, 0.002);//  0.04, 0.0, 0.0001
        rotateRobotPID = new PIDController(0.016, 0.0, 0.03);
        anglePID = new PIDController(0.04, 0, 0.00002);
        robot.init();

        robot.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        robot.backLeft.setDirection(DcMotor.Direction.REVERSE);
        robot.frontRight.setDirection(DcMotor.Direction.FORWARD);
        robot.backRight.setDirection(DcMotor.Direction.FORWARD);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        telemetry.setMsTransmissionInterval(50);


        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            anglePID.tolerance = 1;

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == 17 || tag.id == 18 || tag.id == 19) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }
////////////////////////////////////////////////\\


            sleep(20);
            robot.reach.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.reach.setPositionPIDFCoefficients(1);
            robot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        while (opModeIsActive()) {
            //grab cone

            robot.wrist.setPosition(0.1);
            driveStraight(-2150, 2);
            turnToAngle(103,1.2);
            driveSlide(-360,1.5);
            robot.wrist.setPosition(0.5);
            driveTime.reset();
            farConeTimer.reset();
            while (driveTime.seconds()<4)
                putFarCone();
           // driveStraight(-200, 1.5 );
            //driveSlide(-130,1);
            //turnToAngle(110,1);
            robot.intake1.setPosition(0.9);   //intake up
            robot.intake2.setPosition(0.1);
            repeatedScore(0.38, 0.62);
            driveTime.reset();
            while (driveTime.seconds()<4)
                putFarCone();
            repeatedScore(0.35, 0.65);
            driveTime.reset();
            while (driveTime.seconds()<4)
                putFarCone();
            repeatedScore(0.3, 0.7);
            driveTime.reset();
            while (driveTime.seconds()<4)
                putFarCone();
            repeatedScore(0.25, 0.75);
            driveTime.reset();
            while (driveTime.seconds()<4)
                putFarCone();
            repeatedScore(0.2, 0.8);
            driveTime.reset();
            while (driveTime.seconds()<4)
                putFarCone();

            sleep(30000);
            intakeConeTime.reset();
            while (intakeConeTime.seconds()<2)
            {
            scoreCone();
            }
            intakeConeTime.reset();
            while (intakeConeTime.seconds()<3.3)
            {
                 takeCone();
            }

            sleep(30000);
        }
    }

    public void takeCone(){
        if(robot.elevator.getCurrentPosition()<50 && robot.elevator.getTargetPosition() == 0)
        {
            robot.arm.setTargetPosition(0);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.arm.setPower(1);
        }
        if(intakeConeTime.seconds()<0.85) {
            robot.intake1.setPosition(0.2);
            robot.intake2.setPosition(0.8);   //intake down
            if(intakeConeTime.seconds()>0.7)
            {
                robot.claw.setPosition(0.5);//open
                robot.claw2.setPosition(0.5);//start action with open claws
            }
        }
        if(intakeConeTime.seconds()>0.75 && intakeConeTime.seconds() < 2)   //0.65->0.75
        {
            robot.wrist.setPosition(0.85);
            robot.reach1.setPosition(0.69); //open reach
            robot.reach2.setPosition(0.31);
            robot.reach3.setPosition(0.69);
            robot.reach4.setPosition(0.31);
            if(intakeConeTime.seconds()>1.69)
            {
                robot.claw.setPosition(0.0);//close
                robot.claw2.setPosition(1.0);
            }

        }

        if(intakeConeTime.seconds()>1.75)
        {
            robot.reach1.setPosition(0.45); //close reach
            robot.reach2.setPosition(0.55);
            robot.reach3.setPosition(0.45);
            robot.reach4.setPosition(0.55);

        }
//////////////////////////////////////////////////////////////////////////
        //sleep(250);//wait for cone capture.
        if(intakeConeTime.seconds()>2 && intakeConeTime.seconds()<3)
        {
            if(intakeConeTime.seconds()>2.7)
                robot.cone.setPosition(0.32);
            robot.intake1.setPosition(0.9);   //intake up
            robot.intake2.setPosition(0.1);
            if(intakeConeTime.seconds()>2.8)
            {
                robot.wrist.setPosition(0.98);  //drop position
            }
        }
        if(intakeConeTime.seconds()>2.88){  //2.99->2.88
            robot.claw.setPosition(0.5);   //open claw
            robot.claw2.setPosition(0.5);
            //robot.cone.getController().pwmDisable();
            if(intakeConeTime.seconds()>3.05)
                robot.cone.setPosition(0.05);
        }
        //sleep(1000);//1000
        if(intakeConeTime.seconds()>2.95 && intakeConeTime.seconds()<3.205) {
            //open to drop cone

            robot.wrist.setPosition(0.83);


        }
        if(intakeConeTime.seconds()>3.16)
        {
            robot.reach1.setPosition(0.5); //open reach a bit
            robot.reach2.setPosition(0.5);
            robot.reach3.setPosition(0.5);
            robot.reach4.setPosition(0.5);

        }

    }

    public void scoreCone(){
        int TARGET_ELEVATOR = 1460, TARGET_ELEVATOR_ARM = 900;
            if(scoreConeFirst) {
                robot.elevator.setTargetPosition(TARGET_ELEVATOR);//-1850
                robot.elevator2.setTargetPosition(TARGET_ELEVATOR);
                robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.elevator.setPower(1);
                robot.elevator2.setPower(1);
                scoreConeFirst = false;
            }

            if(Math.abs(robot.intake1.getPosition() - 0.9)<0.1)
            {
                robot.claw.setPosition(0.0);//close
                robot.claw2.setPosition(1.0);
            }


        if(robot.elevator.getCurrentPosition()>TARGET_ELEVATOR/2 && robot.elevator.getTargetPosition() == TARGET_ELEVATOR){//1200 elivator height

            robot.cone.setPosition(CONE_OPEN);
            ///////////////////back
            if(robot.elevator.getCurrentPosition()>TARGET_ELEVATOR*0.9 && robot.elevator.getTargetPosition() == TARGET_ELEVATOR)
            { robot.cone.setPosition(CONE_FOLDED);}
            if(robot.elevator.getCurrentPosition()>TARGET_ELEVATOR-5 && robot.elevator.getTargetPosition() == TARGET_ELEVATOR)
            {
                robot.elevator.setTargetPosition(0);
                robot.elevator2.setTargetPosition( 0);
                robot.elevator.setPower(0.5);
                robot.elevator2.setPower(0.5);
            }
        }

        if(robot.elevator.getCurrentPosition()<TARGET_ELEVATOR*0.75 && robot.elevator.getTargetPosition() == 0 ){         //&& bPressedTimeout.seconds()<3){
            robot.cone.setPosition(0.25);//.getController().pwmDisable();
            robot.elevator.setPower(0.0);
            robot.elevator2.setPower(0.0);
            if(Math.abs(robot.intake1.getPosition() - 0.9)<0.1)
            {
                robot.claw.setPosition(0.0);//close
                robot.claw2.setPosition(1.0);
            }
        }
        ///////////////////////////////////////////
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f meter.", detection.pose.x));
        telemetry.addLine(String.format("Translation Y: %.2f meter.", detection.pose.y));
        telemetry.addLine(String.format("Translation Z: %.2f meter.", detection.pose.z));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    public void driveStraight(int distance, double timeout) {
        double power = 0.8;




        /////////////////    Reset Encoders     ////////////////////////
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        /////////////////////////////////////////////////////////////////
        robot.frontRight.setTargetPosition(distance);
        robot.backRight.setTargetPosition(distance);
        robot.frontLeft.setTargetPosition(distance);
        robot.backLeft.setTargetPosition(distance);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double p = 0.05;
        driveTime.reset();
        while (opModeIsActive() && driveTime.seconds() < timeout) {
            robot.frontRight.setPower(power);
            robot.backRight.setPower(power);
            robot.frontLeft.setPower(power);
            robot.backLeft.setPower(power);
            telemetry.addData("Driving straigh:", robot.frontRight.getCurrentPosition());
            telemetry.addData("Time elapsed:", driveTime.seconds());
            telemetry.update();
        }
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);
    }


    public void driveSlide(int distance, double timeout) {
        double power = 0.8;




        /////////////////    Reset Encoders     ////////////////////////
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        /////////////////////////////////////////////////////////////////
        robot.frontRight.setTargetPosition(distance);
        robot.backRight.setTargetPosition(-distance);
        robot.frontLeft.setTargetPosition(-distance);
        robot.backLeft.setTargetPosition(distance);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double p = 0.05;
        driveTime.reset();
        while (opModeIsActive() && driveTime.seconds() < timeout) {
            robot.frontRight.setPower(power);
            robot.backRight.setPower(power);
            robot.frontLeft.setPower(power);
            robot.backLeft.setPower(power);
            telemetry.addData("Driving straigh:", robot.frontRight.getCurrentPosition());
            telemetry.addData("Time elapsed:", driveTime.seconds());
            telemetry.update();
        }
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);
    }


 public void putFarCone()
 {

//     if(farConeFlag == 0){
//         robot.cone.setPosition(0.2);
//         robot.claw.setPosition(0);//close grip
//         robot.claw2.setPosition(1);
//         robot.elevator.setTargetPosition(1200);//elevator position for arm stretch.
//         robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//         robot.elevator.setPower(1);
//         farConeTimer.reset();
//         farConeFlag = 1;
//     }
//     if(robot.elevator.getCurrentPosition()>1200-300 && robot.elevator.getTargetPosition() == 1200 && farConeTimer.seconds()<2){
//         robot.arm.setTargetPosition(-780);
//         robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//         robot.arm.setPower(1);
//     }
//     if(robot.arm.getCurrentPosition()<-150 && robot.arm.getCurrentPosition()>-730 && robot.arm.getTargetPosition() == -780)
//         robot.cone.setPosition(0.85);
//     //  else if(robot.arm.getCurrentPosition()<-730 && robot.arm.getTargetPosition() == -761)
//     if(robot.arm.getCurrentPosition()<-766 && robot.arm.getTargetPosition() == -780)
//     {
//         robot.cone.setPosition(0.2);
//         robot.arm.setTargetPosition(0);
//     }
//     if(robot.elevator.getTargetPosition() == 1200 && robot.arm.getTargetPosition() == 0 && robot.arm.getCurrentPosition()>-60 && farConeTimer.seconds()>1.5)
//     {
//         robot.elevator.setPower(0);
//
//         //robot.cone.getController().pwmDisable();
//         robot.cone.setPosition(0.04);
//         if(robot.elevator.getCurrentPosition()>-10)
//         {
//             robot.elevator.setTargetPosition(0);
//             robot.elevator.setPower(0);
//
//             robot.arm.setPower(0);
//             farConeTimer.reset();
//         }
//     }

     while(driveTime.seconds()<=0.8)
     {
         robot.elevator.setTargetPosition(1380);
         robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         robot.elevator.setPower(1);
         robot.elevator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         robot.elevator2.setPower(robot.elevator.getPower());
         sleep(20);
     }

     while(driveTime.seconds()>0.8 && driveTime.seconds()<=2.2)
     {
         robot.cone.setPosition(0.8);
         robot.arm.setTargetPosition(-750);
         robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         robot.arm.setPower(1);
         sleep(20);
         if(driveTime.seconds()>0.9 )//&& driveTime.seconds()<1.6)
             robot.cone.setPosition(1);
         if(driveTime.seconds()>1.4)
             robot.cone.setPosition(0.04);
     }
     robot.arm.setTargetPosition(0);
     while (driveTime.seconds()>2.2 && driveTime.seconds()<3)
     {
        telemetry.addData("ARM", robot.arm.getCurrentPosition());
     }
     robot.elevator2.setPower(0);
     robot.elevator.setPower(0);
 }


    public void turnToAngle(int angle, double timeout) {

        rotateRobotPID.rest = angle;
        robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveTime.reset();
        while (opModeIsActive() && driveTime.seconds() < timeout) {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            rotateRobotPID.input = angles.firstAngle;
            rotateRobotPID.calculate();
            robot.frontRight.setPower(rotateRobotPID.output);
            robot.backRight.setPower(rotateRobotPID.output);
            robot.frontLeft.setPower(-rotateRobotPID.output);
            robot.backLeft.setPower(-rotateRobotPID.output);
            telemetry.addData("PID output", rotateRobotPID.output);
            telemetry.update();
        }
        robot.frontRight.setPower(0.0);
        robot.backRight.setPower(0.0);
        robot.frontLeft.setPower(0.0);
        robot.backLeft.setPower(0.0);
    }

    public void repeatedScore(double intake1, double intake2)
    {
        ////////////////////////////////////////////////////////////////
        robot.intake1.setPosition(intake1);
        robot.intake2.setPosition(intake2);   //intake down
        sleep(300);
        robot.reach1.setPosition(0.5); //open reach
        robot.reach2.setPosition(0.5);
        robot.reach3.setPosition(0.5);
        robot.reach4.setPosition(0.5);
        sleep(500);
        robot.claw.setPosition(0.5);
        robot.claw2.setPosition(0.5);
        sleep(150);

        robot.reach1.setPosition(1); //open reach
        robot.reach2.setPosition(0.0);
        robot.reach3.setPosition(1);
        robot.reach4.setPosition(0.0);

        sleep(500);
        robot.claw2.setPosition(1.0);
        robot.claw.setPosition(0.0);//close

sleep(250);
        robot.intake1.setPosition(0.85);   //intake up
        robot.intake2.setPosition(0.15);
        //robot.wrist.setPosition(0.98);  //drop position
        robot.wrist.setPosition(0.83);
        robot.reach1.setPosition(0.3); //close reach
        robot.reach2.setPosition(0.7);
        robot.reach3.setPosition(0.3);
        robot.reach4.setPosition(0.7);

        sleep(700);

        robot.claw.setPosition(0.5);   //open claw
        robot.claw2.setPosition(0.5);

        sleep(500);
        robot.reach1.setPosition(0.5); //close reach
        robot.reach2.setPosition(0.5);
        robot.reach3.setPosition(0.5);
        robot.reach4.setPosition(0.5);
//        robot.cone.setPosition(1 );
//        sleep(600);
//        robot.cone.setPosition(0.04);
//////////////////////////////////////////////////////////////////
    }




}