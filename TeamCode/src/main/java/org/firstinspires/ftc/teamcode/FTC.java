package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name= "FTC", group = "Robot")
public class FTC extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    boolean endReached, startReached, firstPass, intakeUP, intakeCone = false, armDistanceStretch = false, armLongDistanceStretch = false, gamePad_a = false, openStopper = false;
    public static final double CONE_OPEN = 0.9;
    public static final double CONE_FOLDED = 0.1;
    ElapsedTime squarePressedTimeout2 = new ElapsedTime(), yPressedTimeout2 = new ElapsedTime();
    ElapsedTime bPressedTimeout = new ElapsedTime(),yPressedTimeout = new ElapsedTime();
    ElapsedTime aPressedTimeout = new ElapsedTime(), grabConeTimer = new ElapsedTime(), gamePad_aTimer = new ElapsedTime();
    PIDController reachPID = new PIDController(0.06, 0.0, 0.01);
    boolean reachRunning = false;
    double reachRest = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init();

        waitForStart();

        while(opModeIsActive()) {


            if(gamepad1.a && !gamePad_a)
            {
                gamePad_a = true;
                grabConeTimer.reset();
                gamePad_aTimer.reset();
                aPressedTimeout.reset();
                openStopper = true;

            }
            if(gamePad_a)
            {
                takeCone();
            }
            robot.driveMecanum();
            distances();
            int TARGET_ELEVATOR = 1200, TARGET_ELEVATOR_ARM = 900;
            if(gamepad1.b && bPressedTimeout.seconds()>1){
                robot.claw.setPosition(1);//close grip
                robot.claw2.setPosition(0);
                robot.elevator.setTargetPosition(TARGET_ELEVATOR);//-1850
                robot.elevator2.setTargetPosition(TARGET_ELEVATOR);
                robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.elevator.setPower(1);
                robot.elevator2.setPower(1);
                bPressedTimeout.reset();
            }
            if(robot.elevator.getCurrentPosition()>TARGET_ELEVATOR/2 && robot.elevator.getTargetPosition() == TARGET_ELEVATOR){//1200 elivator height
                robot.cone.setPosition(CONE_OPEN);
                ///////////////////back
                if(robot.elevator.getCurrentPosition()>TARGET_ELEVATOR-100 && robot.elevator.getTargetPosition() == TARGET_ELEVATOR)
                    robot.cone.setPosition(CONE_FOLDED);
                if(robot.elevator.getCurrentPosition()>TARGET_ELEVATOR-10 && robot.elevator.getTargetPosition() == TARGET_ELEVATOR){
                    robot.cone.getController().pwmDisable();

                    robot.elevator.setTargetPosition(0);
                    robot.elevator2.setTargetPosition( 0);

                    robot.elevator.setPower(0);
                    robot.elevator2.setPower(0);
                }
            }
            ///////////////////////////////////////////

            if(gamepad1.y && yPressedTimeout.seconds()>1.2){
                robot.cone.setPosition(0.49);
                robot.claw.setPosition(1);//close grip
                robot.claw2.setPosition(0);
                robot.elevator.setTargetPosition(TARGET_ELEVATOR_ARM);//elevator position for arm stretch.
                robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.elevator.setPower(1);

                robot.elevator2.setTargetPosition(TARGET_ELEVATOR_ARM);
                robot.elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.elevator2.setPower(1);

                yPressedTimeout.reset();
            }
            if(robot.elevator.getCurrentPosition()>TARGET_ELEVATOR-300 && robot.elevator.getTargetPosition() == TARGET_ELEVATOR_ARM && yPressedTimeout.seconds()<2){
                robot.arm.setTargetPosition(-751);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setPower(0.8);
            }
            if(robot.arm.getCurrentPosition()<-150 && robot.arm.getCurrentPosition()>-730 && robot.arm.getTargetPosition() == -751)
                robot.cone.setPosition(0.85);
            else if(robot.arm.getCurrentPosition()<-730 && robot.arm.getTargetPosition() == -751)
            {
                robot.cone.setPosition(0.3);
                robot.arm.setTargetPosition(0);
            }
            if(robot.elevator.getTargetPosition() == TARGET_ELEVATOR_ARM && robot.arm.getTargetPosition() == 0 && robot.arm.getCurrentPosition()>-60 && yPressedTimeout.seconds()>1.5)
            {
                robot.claw.setPosition(0.57);
                robot.claw2.setPosition(0.5);
                robot.elevator.setPower(0);
                robot.elevator2.setPower(0);
                robot.cone.getController().pwmDisable();
                if(robot.elevator.getCurrentPosition()>-10)
                {
                    robot.elevator.setPower(0);
                    robot.elevator2.setPower(0);
                    robot.arm.setPower(0);
                    yPressedTimeout.reset();
                }
            }

            telemetry.addData("elevator : elevator2", robot.elevator.getCurrentPosition()+" "+robot.elevator2.getCurrentPosition());
            telemetry.update();

        }//opModeIsActive()

    }//runOpMode()
    public void distances(){

        //////    Square    ////////
        if(gamepad2.square && squarePressedTimeout2.seconds()>0.75){
            if(armDistanceStretch)
            {
                robot.arm.setTargetPosition(0);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setPower(0.7);
            }
            armDistanceStretch = !armDistanceStretch;
            squarePressedTimeout2.reset();
        }
        if(armDistanceStretch){
            robot.cone.setPosition(0.4);
            robot.arm.setTargetPosition(-95);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.arm.setPower(0.7);
        }
        if(!armDistanceStretch && squarePressedTimeout2.seconds()<2)
        {
            robot.cone.getController().pwmDisable();
        }

        //////    Y    ////////

        if(gamepad2.y && yPressedTimeout2.seconds()>0.75){
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
            robot.cone.setPosition(0.4);
            sleep(300);
            robot.arm.setTargetPosition(-180);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.arm.setPower(0.7);
        }
        if(!armLongDistanceStretch && yPressedTimeout2.seconds()<2)
        {
            robot.cone.getController().pwmDisable();
        }
    }

    public void takeCone(){


    if(aPressedTimeout.seconds()<1) {
        robot.intake1.setPosition(0.3);
        robot.intake2.setPosition(0.7);   //intake down

    }

    if(aPressedTimeout.seconds()>1.1 && aPressedTimeout.seconds() < 2)
        {
            robot.wrist.setPosition(0.85);
            robot.reach1.setPosition(0.7); //open reach
            robot.reach2.setPosition(0.3);
            robot.reach3.setPosition(0.7);
            robot.reach4.setPosition(0.3);
            if(aPressedTimeout.seconds()>1.69)
            {
                robot.claw.setPosition(1);//grab cone
                robot.claw2.setPosition(0);
            }

        }

    if(aPressedTimeout.seconds()>2)
    {
        robot.reach1.setPosition(0.45); //close reach
        robot.reach2.setPosition(0.55);
        robot.reach3.setPosition(0.45);
        robot.reach4.setPosition(0.55);

    }
//////////////////////////////////////////////////////////////////////////
    //sleep(250);//wait for cone capture.
    if(aPressedTimeout.seconds()>2.5 && aPressedTimeout.seconds()<3.5)
    {
    if(aPressedTimeout.seconds()>3.2)
         robot.cone.setPosition(0.36);

    robot.intake1.setPosition(0.9);
    robot.intake2.setPosition(0.1);
        if(aPressedTimeout.seconds()>3.3)
        {
            robot.wrist.setPosition(0.92);  //drop position
        }
    }

    //sleep(1000);//1000
    if(aPressedTimeout.seconds()>3.495 && aPressedTimeout.seconds()<4) {
        //open to drop cone

        robot.wrist.setPosition(0.73);

        if(aPressedTimeout.seconds()>3.55){
        robot.claw.setPosition(0);
        robot.claw2.setPosition(1);
            robot.cone.getController().pwmDisable();
        }

        if(aPressedTimeout.seconds()>3.65) {
            robot.claw.setPosition(0.5); //stop opening
            robot.claw2.setPosition(0.5);

        }
        if(aPressedTimeout.seconds()>3.7) {
            robot.claw.setPosition(1); //close a bit
            robot.claw2.setPosition(0);
        }
        if(aPressedTimeout.seconds()>3.9) {
            robot.claw.setPosition(0.5); //stop opening
            robot.claw2.setPosition(0.5);
        }
    }
        if(aPressedTimeout.seconds()>3.9){
            robot.reach1.setPosition(0.55); //open reach a bit
            robot.reach2.setPosition(0.45);
            robot.reach3.setPosition(0.55);
            robot.reach4.setPosition(0.45);
            gamePad_a = false;
        }

}

    ////////////////////////////////////////////////////////////////////////////////

}
