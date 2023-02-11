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

        boolean reachedFarCone =false, bingo = false;
        waitForStart();
        playTime.reset();

        while(opModeIsActive()) {

            if(playTime.seconds()<4.2){
                robot.wrist.setPosition(0.8);
                driveStraight(-2150, 2);
                turnToAngle(103,1.2);
                driveSlide(-360,1);
            }



            if(playTime.seconds()>4.5 && playTime.seconds()< 4.6 )
                simulateGamepd1_y = true;
            if(playTime.seconds()>4.7 && playTime.seconds()< 4.8 )
            { simulateGamepd1_y = false; coneHeight =0.338;}

            if(playTime.seconds()>5.7 && playTime.seconds()< 5.8 )  //#1
                simulateGamepd1_a = true;
            if(playTime.seconds()>5.8 && playTime.seconds()< 5.9 )
                simulateGamepd1_a = false;

            if(playTime.seconds()>8.8 && playTime.seconds()< 8.9 )
                simulateGamepd1_y = true;
            if(playTime.seconds()>8.9 && playTime.seconds()< 9 )
            { simulateGamepd1_y = false;
            coneHeight = 0.32;}
            if(playTime.seconds()> 10 && playTime.seconds()< 10.1 )  //#2
                simulateGamepd1_a = true;
            if(playTime.seconds()> 10.1 && playTime.seconds()< 10.2 )
                simulateGamepd1_a = false;

            if(playTime.seconds()>13.1 && playTime.seconds()<13.2 )
                simulateGamepd1_y = true;
            if(playTime.seconds()>13.2 && playTime.seconds()<13.3 )
            {simulateGamepd1_y = false;
            coneHeight = 0.3;}
            if(playTime.seconds()>14.1 && playTime.seconds()<14.2 )  ///#3
                simulateGamepd1_a = true;
            if(playTime.seconds()>14.2 && playTime.seconds()<14.3 )
            {simulateGamepd1_a = false;}

            if(playTime.seconds()>17.2 && playTime.seconds()<17.3 )
                simulateGamepd1_y = true;
            if(playTime.seconds()>17.3 && playTime.seconds()<17.4 )
            {  simulateGamepd1_y = false;
            coneHeight = 0.24;}
            if(playTime.seconds()>18.2 && playTime.seconds()< 18.9 )//#4
                simulateGamepd1_a = true;
            if(playTime.seconds()>18.9 && playTime.seconds()< 19 )
            {simulateGamepd1_a = false;}

            if(playTime.seconds()>21.3 && playTime.seconds()<21.4 )
                simulateGamepd1_y = true;
            if(playTime.seconds()>21.4 && playTime.seconds()< 21.5 )
            {  simulateGamepd1_y = false;coneHeight = 0.2;}
            if(playTime.seconds()>22.3 && playTime.seconds()<22.4 ) //#5
                simulateGamepd1_a = true;
            if(playTime.seconds()>22.4 && playTime.seconds()< 22.5)
            {simulateGamepd1_a = false;}

            if(playTime.seconds()>25.4 && playTime.seconds()< 25.5 )
                simulateGamepd1_y = true;
            if(playTime.seconds()>25.5 && playTime.seconds()< 25.6 )
            {  simulateGamepd1_y = false;}
//            if(playTime.seconds()>22.3&& playTime.seconds()<22.4)
//                simulateGamepd1_a = true;
//            if(playTime.seconds()>22.6&& playTime.seconds()<22.7)
//                simulateGamepd1_a = false;



            /*if(gamepad1.x && xPressedTimeout.seconds()>0.75){
                //robot.cone.setPosition(robot.cone.getPosition() == 0.85 ? 0.2 : 0.85);
                if(Math.abs(robot.cone.getPosition()- 0.85)<0.01)
                    robot.cone.setPosition(0.04);
                else
                    robot.cone.setPosition(0.85);

                xPressedTimeout.reset();
            }*/
            if(simulateGamepd1_a && !gamePad_a)
            {
                gamePad_a = true;
                aPressedTimeout.reset();
            }
            if(gamePad_a)
            {
                takeCone();
            }
            /////////////////////////////////////////////////
            int TARGET_ELEVATOR_ARM = 1000;
            robot.elevator2.setPower(robot.elevator.getPower());
  /*
            robot.driveMecanum();
            distances();


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
*/
            if(simulateGamepd1_y && yPressedTimeout.seconds()>0.4)
            {
                robot.cone.setPosition(0.2);
                robot.claw.setPosition(0);//close grip
                robot.claw2.setPosition(1);
                robot.elevator.setTargetPosition(TARGET_ELEVATOR_ARM);//elevator position for arm stretch.
                robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.elevator.setPower(1);
                bingo = false;

                yPressedTimeout.reset();
            }/////  heeeeeeeeeeerrrrrrrrrrreeeeeeeeeeeeeeeeeee
            if(robot.elevator.getCurrentPosition()>TARGET_ELEVATOR_ARM-300 && robot.elevator.getTargetPosition() == TARGET_ELEVATOR_ARM && !reachedFarCone && !bingo){//&& yPressedTimeout.seconds()<1.5
                robot.arm.setTargetPosition(-880);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setPower(0.8);
            }
            if(robot.arm.getCurrentPosition()<-150 && robot.arm.getCurrentPosition()>-730 && robot.arm.getTargetPosition() == -880)
                robot.cone.setPosition(1);
            //  else if(robot.arm.getCurrentPosition()<-730 && robot.arm.getTargetPosition() == -761)
            if(robot.arm.getCurrentPosition()<-766 && robot.arm.getTargetPosition() == -880 && robot.elevator.getCurrentPosition()>TARGET_ELEVATOR_ARM-100)
            {
                robot.cone.setPosition(0.2);
                robot.arm.setTargetPosition(0);
                reachedFarCone = true;
                bingo =true;
            }
            if(robot.elevator.getTargetPosition() == TARGET_ELEVATOR_ARM && robot.arm.getTargetPosition() == 0 && robot.arm.getCurrentPosition()>-60 && bingo)//&& yPressedTimeout.seconds()>1.5)
            {
                robot.elevator.setPower(0);

                //robot.cone.getController().pwmDisable();
                robot.cone.setPosition(0.04);
                if(robot.elevator.getCurrentPosition()<200)
                {
                    robot.elevator.setTargetPosition(0);
                    robot.elevator.setPower(0);

                    robot.arm.setPower(0);
                    yPressedTimeout.reset();
                    reachedFarCone = false;
                }
            }

            telemetry.addData("elevator : elevator2", robot.elevator.getCurrentPosition()+" "+robot.elevator2.getCurrentPosition());
            telemetry.addData("ARM", robot.arm.getCurrentPosition());
            telemetry.update();

        }//opModeIsActive()

    }//runOpMode()


    public void takeCone(){
        if(robot.elevator.getCurrentPosition()<50 && robot.elevator.getTargetPosition() == 0 && robot.arm.getTargetPosition() != 0)
        {
            robot.arm.setTargetPosition(0);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.arm.setPower(1);
        }
        if(aPressedTimeout.seconds()<0.85)
        {
            robot.intake1.setPosition(coneHeight);
            robot.intake2.setPosition(1-coneHeight);   //intake down

            if(aPressedTimeout.seconds()>0.7)
            {
                robot.claw.setPosition(0.5);//open
                robot.claw2.setPosition(0.5);//start action with open claws
                //robot.wrist.setPosition(0 );
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
        if(aPressedTimeout.seconds()>1.85) // 1.75 -> 1.85
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
        if(aPressedTimeout.seconds()>2.78 && aPressedTimeout.seconds()<3.1) //2.88->2.78
        {  //2.99->2.88
            robot.claw.setPosition(0.5);   //open claw
            robot.claw2.setPosition(0.5);
        }
        //sleep(1000);//1000
        if(aPressedTimeout.seconds()>2.95 && aPressedTimeout.seconds()<3.205) {
            //open to drop cone
            robot.wrist.setPosition(0.78);
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
