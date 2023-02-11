package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
@Autonomous(name = "Auto FTC", group = "Robot")
public class Auto2 extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    ElapsedTime driveTime = new ElapsedTime();
    ElapsedTime intakeConeTime = new ElapsedTime();
    public static final double CONE_OPEN = 0.9;
    public static final double CONE_FOLDED = 0.1;
    PIDController rotateRobotPID = new PIDController(0.016, 0.0, 0.03);
    Orientation angles;
    BNO055IMU imu;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        waitForStart();
        while (opModeIsActive()) {
            //grab cone

            robot.wrist.setPosition(0.1);
            driveStraight(-2150, 2);
            turnToAngle(105,1.2);
            driveSlide(-340,1.5);
            robot.wrist.setPosition(0.5);
            driveTime.reset();

            while (driveTime.seconds()<4)
                putFarCone();
            driveStraight(-200, 1.5 );
            driveSlide(-170,1);
            turnToAngle(110,1);
            robot.intake1.setPosition(0.9);   //intake up
            robot.intake2.setPosition(0.1);
            repeatedScore(0.38, 0.62);
//            repeatedScore(0.35, 0.65);
//            repeatedScore(0.3, 0.7);
//            repeatedScore(0.25, 0.75);
//            repeatedScore(0.2, 0.8);

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

        while(driveTime.seconds()<=1)
        {
            robot.elevator.setTargetPosition(1400);
            robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.elevator.setPower(1);
            robot.elevator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.elevator2.setPower(robot.elevator.getPower());
            sleep(20);
        }

        while(driveTime.seconds()>1 && driveTime.seconds()<=3)
        {
            robot.cone.setPosition(0.8);
            robot.arm.setTargetPosition(-850);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.arm.setPower(1);
            sleep(20);
            if(driveTime.seconds()>1.1 )//&& driveTime.seconds()<1.6)
                robot.cone.setPosition(0.8);
            if(driveTime.seconds()>1.6)
                robot.cone.setPosition(0.04);
        }
        robot.arm.setTargetPosition(0);
        while (driveTime.seconds()>3 && driveTime.seconds()<4)
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

        robot.reach1.setPosition(0.95); //open reach
        robot.reach2.setPosition(0.05);
        robot.reach3.setPosition(0.95);
        robot.reach4.setPosition(0.05);

        sleep(500);
        robot.claw2.setPosition(1.0);
        robot.claw.setPosition(0.0);//close

        sleep(250);
        robot.intake1.setPosition(0.9);   //intake up
        robot.intake2.setPosition(0.1);
        robot.wrist.setPosition(0.98);  //drop position
        robot.wrist.setPosition(0.83);
        robot.reach1.setPosition(0.2); //close reach
        robot.reach2.setPosition(0.8);
        robot.reach3.setPosition(0.2);
        robot.reach4.setPosition(0.8);

        sleep(700);

        robot.claw.setPosition(0.5);   //open claw
        robot.claw2.setPosition(0.5);

        sleep(500);
        robot.reach1.setPosition(0.5); //close reach
        robot.reach2.setPosition(0.5);
        robot.reach3.setPosition(0.5);
        robot.reach4.setPosition(0.5);
        robot.cone.setPosition(1 );
        sleep(600);
        robot.cone.setPosition(0.04);
//////////////////////////////////////////////////////////////////
    }




}
