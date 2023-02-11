package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name= "FTC", group = "Robot")
public class FTC extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    boolean endReached, startReached, firstPass, intakeUP, intakeCone = false, armDistanceStretch = false, armLongDistanceStretch = false, gamePad_a = false, openStopper = false;
    public static final double CONE_OPEN = 0.88;
    public static final double CONE_FOLDED = 0.04;
    ElapsedTime rightBumper2PressedTimeout = new ElapsedTime(), yPressedTimeout2 = new ElapsedTime();
    ElapsedTime bPressedTimeout = new ElapsedTime(),yPressedTimeout = new ElapsedTime();
    ElapsedTime aPressedTimeout = new ElapsedTime(), grabConeTimer = new ElapsedTime(), gamePad_aTimer = new ElapsedTime(), xPressedTimeout = new ElapsedTime();
    PIDController reachPID = new PIDController(0.06, 0.0, 0.01);
    boolean reachRunning = false;
    double reachRest = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init();

        waitForStart();


        while(opModeIsActive()) {

             robot.elevator2.setPower(robot.elevator.getPower());

            if(gamepad1.x && xPressedTimeout.seconds()>0.75){
                //robot.cone.setPosition(robot.cone.getPosition() == 0.85 ? 0.2 : 0.85);
                if(Math.abs(robot.cone.getPosition()- 0.85)<0.01)
                    robot.cone.setPosition(0.04);
                else
                    robot.cone.setPosition(0.85);

                xPressedTimeout.reset();
            }
            if(gamepad1.a && !gamePad_a)
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

            if(gamepad1.y && yPressedTimeout.seconds()>1.2){
                robot.cone.setPosition(0.2);
                robot.claw.setPosition(0);//close grip
                robot.claw2.setPosition(1);
                robot.elevator.setTargetPosition(TARGET_ELEVATOR_ARM);//elevator position for arm stretch.
                robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.elevator.setPower(1);

                yPressedTimeout.reset();
            }
            if(robot.elevator.getCurrentPosition()>TARGET_ELEVATOR_ARM-300 && robot.elevator.getTargetPosition() == TARGET_ELEVATOR_ARM && yPressedTimeout.seconds()<2){
                robot.arm.setTargetPosition(-780);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setPower(1);
            }
            if(robot.arm.getCurrentPosition()<-150 && robot.arm.getCurrentPosition()>-730 && robot.arm.getTargetPosition() == -780)
                robot.cone.setPosition(0.85);
          //  else if(robot.arm.getCurrentPosition()<-730 && robot.arm.getTargetPosition() == -761)
            if(robot.arm.getCurrentPosition()<-766 && robot.arm.getTargetPosition() == -780)
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
        robot.intake1.setPosition(0.2);
        robot.intake2.setPosition(0.8);   //intake down

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
        robot.reach1.setPosition(0.45); //close reach
        robot.reach2.setPosition(0.55);
        robot.reach3.setPosition(0.45);
        robot.reach4.setPosition(0.55);

    }
//////////////////////////////////////////////////////////////////////////
    //sleep(250);//wait for cone capture.
    if(aPressedTimeout.seconds()>2 && aPressedTimeout.seconds()<3)
    {
        robot.intake1.setPosition(0.9);   //intake up
        robot.intake2.setPosition(0.1);
        if(aPressedTimeout.seconds()>2.8)
        {
            robot.wrist.setPosition(0.98);  //drop position
        }
    }
   if(aPressedTimeout.seconds()>2.88 && aPressedTimeout.seconds()<3.1)
   {  //2.99->2.88
            robot.claw.setPosition(0.5);   //open claw
            robot.claw2.setPosition(0.5);
   }
    //sleep(1000);//1000
    if(aPressedTimeout.seconds()>2.95 && aPressedTimeout.seconds()<3.205) {
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

}
