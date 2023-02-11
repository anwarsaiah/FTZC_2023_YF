package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name= "Crazy", group = "Robot")
public class AutoCrazy extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    boolean endReached, startReached, firstPass, intakeUP, intakeCone = false, armDistanceStretch = false, armLongDistanceStretch = false, gamePad_a = false, openStopper = false;
    public static final double CONE_OPEN = 0.88;
    public static final double CONE_FOLDED = 0.04;
    ElapsedTime rightBumper2PressedTimeout = new ElapsedTime(), yPressedTimeout2 = new ElapsedTime();
    ElapsedTime bPressedTimeout = new ElapsedTime(),yPressedTimeout = new ElapsedTime();
    ElapsedTime aPressedTimeout = new ElapsedTime(), grabConeTimer = new ElapsedTime(), gamePad_aTimer = new ElapsedTime(), xPressedTimeout = new ElapsedTime();
    ElapsedTime playTime = new ElapsedTime(), driveTime = new ElapsedTime();
    PIDController reachPID = new PIDController(0.06, 0.0, 0.01);
    boolean reachRunning = false;
    double reachRest = 0;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    PIDController pid = null, rotatePID = null, rotateRobotPID = null, anglePID = null;
    Orientation angles;
    BNO055IMU imu;
    double coneHeight;
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);



        pid = new PIDController(0.0002, 0, 0.0001);
        rotatePID = new PIDController(0.035, 0.0, 0.002);//  0.04, 0.0, 0.0001
        rotateRobotPID = new PIDController(0.016, 0.0, 0.03);
        anglePID = new PIDController(0.04, 0, 0.00002);

        boolean simulateGamepd1_a = false;
        boolean simulateGamepd1_y = false;

        waitForStart();
        playTime.reset();

        while(opModeIsActive()) {

//            if(playTime.seconds()<4.7){
//                robot.wrist.setPosition(0.1);
//                driveStraight(-2150, 2);
//                turnToAngle(103,1.2);
//                driveSlide(-360,1.5);
//            }

            coneHeight = 0.335;

            if(playTime.seconds()>4 && playTime.seconds()<4.1)
                simulateGamepd1_y = true;
            if(playTime.seconds()>4.3 && playTime.seconds()<4.4)
                simulateGamepd1_y = false;

            if(playTime.seconds()>5.1 && playTime.seconds()<6.1)  //#1
                simulateGamepd1_a = true;
            if(playTime.seconds()>5.3&& playTime.seconds()<6.4)
                simulateGamepd1_a = false;

            if(playTime.seconds()>7.5 && playTime.seconds()<7.6)
                simulateGamepd1_y = true;
            if(playTime.seconds()>7.2&& playTime.seconds()<7.3)
                simulateGamepd1_y = false;
            coneHeight = 0.335;
            if(playTime.seconds()>9&& playTime.seconds()<9.1)  //#2
                simulateGamepd1_a = true;
            if(playTime.seconds()>9.3&& playTime.seconds()<9.4)
                simulateGamepd1_a = false;

            if(playTime.seconds()>12.6 && playTime.seconds()<12.7)
                simulateGamepd1_y = true;
            if(playTime.seconds()>12.9&& playTime.seconds()<13)
            {simulateGamepd1_y = false;
            coneHeight = 0.325;}
            if(playTime.seconds()>16.9&& playTime.seconds()<17)  ///#3
                simulateGamepd1_a = true;
            if(playTime.seconds()>17.3&& playTime.seconds()<17.4)
            {simulateGamepd1_a = false;coneHeight = 0.29;}

            if(playTime.seconds()>20.3 && playTime.seconds()<20.4)
                simulateGamepd1_y = true;
            if(playTime.seconds()>20.6&& playTime.seconds()<20.7)
            {  simulateGamepd1_y = false;
            coneHeight = 0.21;}
            if(playTime.seconds()>22.3&& playTime.seconds()<22.4)//#4
                simulateGamepd1_a = true;
            if(playTime.seconds()>22.6&& playTime.seconds()<22.7)
            {simulateGamepd1_a = false;coneHeight = 0.28;}

            if(playTime.seconds()>25.6 && playTime.seconds()<25.7)
                simulateGamepd1_y = true;
            if(playTime.seconds()>25.9&& playTime.seconds()<26)
            {  simulateGamepd1_y = false;
                coneHeight = 0.2;}
            if(playTime.seconds()>27.9&& playTime.seconds()<28) //#5
                simulateGamepd1_a = true;
            if(playTime.seconds()>29&& playTime.seconds()<29.1)
            {simulateGamepd1_a = false;coneHeight = 0.25;}

//            if(playTime.seconds()>20.3 && playTime.seconds()<20.4)
//                simulateGamepd1_y = true;
//            if(playTime.seconds()>20.6&& playTime.seconds()<20.7)
//            {  simulateGamepd1_y = false;
//                coneHeight = 0.31;}
//            if(playTime.seconds()>22.3&& playTime.seconds()<22.4)
//                simulateGamepd1_a = true;
//            if(playTime.seconds()>22.6&& playTime.seconds()<22.7)
//                simulateGamepd1_a = false;

            robot.elevator2.setPower(robot.elevator.getPower());

            if(gamepad1.x && xPressedTimeout.seconds()>0.75){
                //robot.cone.setPosition(robot.cone.getPosition() == 0.85 ? 0.2 : 0.85);
                if(Math.abs(robot.cone.getPosition()- 0.85)<0.01)
                    robot.cone.setPosition(0.04);
                else
                    robot.cone.setPosition(0.85);

                xPressedTimeout.reset();
            }
            if(simulateGamepd1_a && !gamePad_a)
            {
                gamePad_a = true;
                aPressedTimeout.reset();
            }
            if(gamePad_a)
            {
                takeCone();
            }
            robot.driveMecanum();
            distances();
            int TARGET_ELEVATOR = 1460, TARGET_ELEVATOR_ARM = 900;

            if(gamepad1.b && bPressedTimeout.seconds()>1){
                robot.elevator.setTargetPosition(TARGET_ELEVATOR);//-1850
                robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.elevator.setPower(1);
                bPressedTimeout.reset();
                if(Math.abs(robot.intake1.getPosition() - 0.9)<0.1)
                {
                    robot.claw.setPosition(0.4);//close loose
                    robot.claw2.setPosition(0.6);
                }

            }

            if(robot.elevator.getCurrentPosition()>TARGET_ELEVATOR-800 && robot.elevator.getTargetPosition() == TARGET_ELEVATOR){//1200 elivator height
                robot.cone.setPosition(CONE_OPEN);
                ///////////////////back
                if(robot.elevator.getCurrentPosition()>(TARGET_ELEVATOR-30) && robot.elevator.getTargetPosition() == TARGET_ELEVATOR)
                { robot.cone.setPosition(CONE_FOLDED);}
                if(robot.elevator.getCurrentPosition()>TARGET_ELEVATOR-5 && robot.elevator.getTargetPosition() == TARGET_ELEVATOR)
                {
                    robot.elevator.setTargetPosition(0);
                    robot.elevator.setPower(0.0);

                }
            }

            if(robot.elevator.getCurrentPosition()<TARGET_ELEVATOR*0.75 && robot.elevator.getTargetPosition() == 0 && bPressedTimeout.seconds()<2){
                // robot.cone.setPosition(0.25);//.getController().pwmDisable();
                robot.elevator.setPower(0.0);

                if(Math.abs(robot.intake1.getPosition() - 0.9)<0.1)
                {
                    robot.claw.setPosition(0.0);//close
                    robot.claw2.setPosition(1.0);
                }
            }
            ///////////////////////////////////////////

            if(simulateGamepd1_y && yPressedTimeout.seconds()>0.4){
                robot.cone.setPosition(0.2);
                robot.claw.setPosition(0);//close grip
                robot.claw2.setPosition(1);
                robot.elevator.setTargetPosition(TARGET_ELEVATOR_ARM);//elevator position for arm stretch.
                robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.elevator.setPower(1);

                yPressedTimeout.reset();
            }/////  heeeeeeeeeeerrrrrrrrrrreeeeeeeeeeeeeeeeeee
            if(robot.elevator.getCurrentPosition()>TARGET_ELEVATOR_ARM-300 && robot.elevator.getTargetPosition() == TARGET_ELEVATOR_ARM && yPressedTimeout.seconds()<1.5){
                robot.arm.setTargetPosition(-880);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setPower(1);
            }
            if(robot.arm.getCurrentPosition()<-150 && robot.arm.getCurrentPosition()>-730 && robot.arm.getTargetPosition() == -880)
                robot.cone.setPosition(1);
            //  else if(robot.arm.getCurrentPosition()<-730 && robot.arm.getTargetPosition() == -761)
            if(robot.arm.getCurrentPosition()<-766 && robot.arm.getTargetPosition() == -880)
            {
                robot.cone.setPosition(0.2);
                robot.arm.setTargetPosition(0);
            }
            if(robot.elevator.getTargetPosition() == TARGET_ELEVATOR_ARM && robot.arm.getTargetPosition() == 0 && robot.arm.getCurrentPosition()>-60 && yPressedTimeout.seconds()>1.5)
            {
                robot.elevator.setPower(0);

                //robot.cone.getController().pwmDisable();
                robot.cone.setPosition(0.04);
                if(robot.elevator.getCurrentPosition()>-10)
                {
                    robot.elevator.setTargetPosition(0);
                    robot.elevator.setPower(0);

                    robot.arm.setPower(0);
                    yPressedTimeout.reset();
                }
            }

            telemetry.addData("elevator : elevator2", robot.elevator.getCurrentPosition()+" "+robot.elevator2.getCurrentPosition());
            telemetry.addData("ARM", robot.arm.getCurrentPosition());
            telemetry.update();

        }//opModeIsActive()

    }//runOpMode()
    public void distances(){

        //////    Square    ////////
        if(gamepad1.right_bumper && rightBumper2PressedTimeout.seconds()>0.75){
            armDistanceStretch = !armDistanceStretch;
            rightBumper2PressedTimeout.reset();
        }
        if(armDistanceStretch){
            if(Math.abs(robot.cone.getPosition()- 0.85)>0.01)
                robot.cone.setPosition(0.25);
            robot.arm.setTargetPosition(-125);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.arm.setPower(0.7);
        }
        if(!armDistanceStretch && rightBumper2PressedTimeout.seconds()<1)
        {
            //robot.cone.getController().pwmDisable();

            robot.arm.setTargetPosition(0);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.arm.setPower(0.7);
            if(rightBumper2PressedTimeout.seconds()>0.8)
                robot.cone.setPosition(0.04);
        }

        //////    Y    ////////

        if(gamepad2.y && yPressedTimeout2.seconds()>0.75){ //score far cone
            if(armLongDistanceStretch)
            {
                robot.arm.setTargetPosition(0);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setPower(0.7);
            }
            armLongDistanceStretch = !armLongDistanceStretch;
            yPressedTimeout2.reset();
        }
        if(armLongDistanceStretch){
            robot.cone.setPosition(0.1);
            // sleep(50);
            robot.arm.setTargetPosition(-460);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.arm.setPower(0.7);
        }
        if(!armLongDistanceStretch && yPressedTimeout2.seconds()<2)
        {
            robot.cone.setPosition(0.04);
        }
    }

    public void takeCone(){
        if(robot.elevator.getCurrentPosition()<50 && robot.elevator.getTargetPosition() == 0)
        {
            robot.arm.setTargetPosition(0);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.arm.setPower(1);
        }
        if(aPressedTimeout.seconds()<0.85) {
            robot.intake1.setPosition(coneHeight);
            robot.intake2.setPosition(1-coneHeight);   //intake down

            if(aPressedTimeout.seconds()>0.7)
            {
                robot.claw.setPosition(0.5);//open
                robot.claw2.setPosition(0.5);//start action with open claws
            }
        }
        if(aPressedTimeout.seconds()>0.75 && aPressedTimeout.seconds() < 2)   //0.65->0.75
        {
            robot.wrist.setPosition(0.85);
            robot.reach1.setPosition(0.69); //open reach
            robot.reach2.setPosition(0.31);
            robot.reach3.setPosition(0.69);
            robot.reach4.setPosition(0.31);
            if(aPressedTimeout.seconds()>1.69)
            {
                robot.claw.setPosition(0.0);//close
                robot.claw2.setPosition(1.0);
            }

        }

        if(aPressedTimeout.seconds()>1.75)
        {
            robot.intake1.setPosition(0.9);   //intake up
            robot.intake2.setPosition(0.1);

        }
//////////////////////////////////////////////////////////////////////////
        //sleep(250);//wait for cone capture.
        if(aPressedTimeout.seconds()>2 && aPressedTimeout.seconds()<3)
        {
            robot.reach1.setPosition(0.45); //close reach
            robot.reach2.setPosition(0.55);
            robot.reach3.setPosition(0.45);
            robot.reach4.setPosition(0.55);

            if(aPressedTimeout.seconds()>2.8)
            {
                robot.wrist.setPosition(1);  //drop position
            }
        }
        if(aPressedTimeout.seconds()>2.88 && aPressedTimeout.seconds()<3.1)
        {  //2.99->2.88
            robot.claw.setPosition(0.5);   //open claw
            robot.claw2.setPosition(0.5);
        }
        //sleep(1000);//1000
        if(aPressedTimeout.seconds()>3.1 && aPressedTimeout.seconds()<3.205) {
            //open to drop cone
            robot.wrist.setPosition(0.83);
        }
        if(aPressedTimeout.seconds()>3.16)
        {
            robot.reach1.setPosition(0.5); //open reach a bit
            robot.reach2.setPosition(0.5);
            robot.reach3.setPosition(0.5);
            robot.reach4.setPosition(0.5);
            gamePad_a = false;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
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
}
