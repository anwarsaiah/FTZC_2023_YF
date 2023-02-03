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

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This particular OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Robot: Teleop POV", group="Robot")
//@Disabled
public class RobotTeleopPOV_Linear extends LinearOpMode {
    /* Declare OpMode members. */
//    public DcMotor  frontLeft = null;
//    public DcMotor  frontRight = null;
//    public DcMotor  backLeft = null;
//    public DcMotor  backRight = null;
//
//    public DcMotor  elevator     = null;
//    public DcMotorEx  arm    = null;
//    public DcMotorEx reach = null;
//    public DcMotorEx intake = null;
//
//    public Servo    wrist   = null;
//    public Servo    claw   = null;
//    public Servo    claw2   = null;
//    public Servo    cone   = null;
//    public Servo   intake1 = null;
//    public Servo   intake2 = null;
//    public Servo   stopper = null;
//
//    public RevTouchSensor touch = null;


    RobotHardware robot = new RobotHardware(this);


    boolean gamePad_a = false;
    public boolean liftingElevator;
    ElapsedTime elapsedTime = new ElapsedTime();
    ElapsedTime scoreTime = new ElapsedTime();
    ElapsedTime grabConeTimer = new ElapsedTime();
    ElapsedTime gamePad_aTimer = new ElapsedTime();
    ElapsedTime yPressedTimeout = new ElapsedTime();
    ElapsedTime bPressedTimeout = new ElapsedTime();
    ElapsedTime squarePressedTimeout2 = new ElapsedTime(), yPressedTimeout2 = new ElapsedTime();
    boolean endReached, startReached, firstPass, intakeUP, intakeCone = false, armDistanceStretch = false, armLongDistanceStretch = false;
    public static final double CONE_OPEN = 0.9;
    public static final double CONE_FOLDED = 0.1;


    @Override
    public void runOpMode() {
//        double left;
//        double right;
//        double drive;
//        double turn;
//        double max;
//
//        // Define and Initialize Motors
//
//        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
//        frontRight = hardwareMap.get(DcMotor.class, "frontright");
//        backLeft = hardwareMap.get(DcMotor.class, "backleft");
//        backRight = hardwareMap.get(DcMotor.class, "backright");
//        elevator = hardwareMap.get(DcMotor.class, "elevator");
//        arm = hardwareMap.get(DcMotorEx.class, "arm");
//        reach = hardwareMap.get(DcMotorEx.class, "reach");
//        stopper = hardwareMap.get(Servo.class, "stopper");
//
//
//
//        wrist = hardwareMap.get(Servo.class, "wrist");
//        claw = hardwareMap.get(Servo.class, "claw");
//        cone = hardwareMap.get(Servo.class, "cone");
//        claw2 = hardwareMap.get(Servo.class, "claw2");
//        intake1 = hardwareMap.get(Servo.class,"intake1");
//        intake2 = hardwareMap.get(Servo.class,"intake2");
//
//
//        touch = hardwareMap.get(RevTouchSensor.class,"touch");
//
//        frontLeft.setDirection(DcMotor.Direction.REVERSE);
//        backLeft.setDirection(DcMotor.Direction.REVERSE);
//        frontRight.setDirection(DcMotor.Direction.FORWARD);
//        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Send telemetry message to signify robot waiting;
        robot.init();
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.addData("cone position", robot.cone.getPosition());
        telemetry.update();
        //////
        endReached = false;
        startReached = false;
        firstPass = true;
        liftingElevator = false;
        intakeUP=false;
        telemetry.addData("reach:", robot.reach.getCurrentPosition());
        telemetry.update();
        //cone.setPosition(0.1);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        elapsedTime.reset();
        yPressedTimeout.reset();
        scoreTime.reset();
        //intake1.setPosition(0.5);
        //intake2.setPosition(0.5);
        robot.reach.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.reach.setPositionPIDFCoefficients(1);
        robot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // run until the end of the match (driver presses STOP)
while (opModeIsActive()) {
    robot.driveMecanum();
    distances();
    closerReach(); //Motor in run without encoder mode and idle, after call.
    if(gamepad1.a && !gamePad_a)
    {
        gamePad_a = true;
        grabConeTimer.reset();
        gamePad_aTimer.reset();
    }
    if(gamePad_a)
    {
             takeCone();
    }
    //Distance from Bar!

    if(gamepad1.b && bPressedTimeout.seconds()>1){
        robot.claw.setPosition(1);//close grip
        robot.claw2.setPosition(0);
    //    elevator.setPower(-1.0);
    //    sleep(800);
        robot.elevator.setTargetPosition(-1850);
        robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elevator.setPower(1);
        bPressedTimeout.reset();
    }
    ////////////////////////////////////////////
        if(robot.elevator.getCurrentPosition()<-1000 && robot.elevator.getTargetPosition() == -1850){
        robot.cone.setPosition(CONE_OPEN);
        ///////////////////back
        if(robot.elevator.getCurrentPosition()<-1450 && robot.elevator.getTargetPosition() == -1850)
             robot.cone.setPosition(CONE_FOLDED);
        if(robot.elevator.getCurrentPosition()<-1800 && robot.elevator.getTargetPosition() == -1850){
             robot.cone.getController().pwmDisable();
             robot.elevator.setTargetPosition(0);
        }
        }
        ///////////////////////////////////////////

        if(gamepad1.y && yPressedTimeout.seconds()>1.2){
            robot.cone.setPosition(0.49);
            robot.claw.setPosition(1);//close grip
            robot.claw2.setPosition(0);
            robot.elevator.setTargetPosition(-1539);//elevator position for arm stretch.
            robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.elevator.setPower(1);
            yPressedTimeout.reset();
        }
        if(robot.elevator.getCurrentPosition()<-1400 && robot.elevator.getTargetPosition() == -1539 && yPressedTimeout.seconds()<2){
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
       if(robot.elevator.getTargetPosition() == -1539 && robot.arm.getTargetPosition() == 0 && robot.arm.getCurrentPosition()>-60 && yPressedTimeout.seconds()>1.5)
       {
           robot.claw.setPosition(0.57);
           robot.claw2.setPosition(0.5);
           robot.elevator.setTargetPosition(0);
           robot.cone.getController().pwmDisable();
           if(robot.elevator.getCurrentPosition()>-10)
           {
               robot.elevator.setPower(0);
               robot.arm.setPower(0);
               yPressedTimeout.reset();
           }
       }


              // Send telemetry message to signify robot running;
              telemetry.addData("left stick gamepad1 <---  --->", gamepad1.left_stick_x);
              telemetry.addData("elevator",  "Offset = "+ robot.elevator.getCurrentPosition());
              telemetry.addData("Arm(Telescope)", robot.arm.getCurrentPosition());

              telemetry.addData("reach", "Offset = "+robot.reach.getCurrentPosition());

              telemetry.addData("arm", "Offset = "+robot.arm.getCurrentPosition());
              telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }


public void closerReach()
{
    if(!robot.touch.isPressed())
    {
        robot.reach.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.reach.setPower(1.0);
    }
    else{
        robot.stopper.setPosition(0);//close hook!
        sleep(250);
        robot.reach.setPower(0);
    }
}
public void openReach(){
    robot.reach.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    robot.reach.setPower(1.0);
    robot.stopper.setPosition(1);
    sleep(250);
    robot.reach.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
}

public void takeCone(){
    robot.claw.setPosition(0.57);
    robot.claw2.setPosition(0.5);
    robot.intake1.setPosition(1);
    robot.intake2.setPosition(0);   //intake down
    robot.wrist.setPosition(0.75);
    sleep(120);
    openReach(); //motor will be stopped after this call.
    robot.reach.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    grabConeTimer.reset();
    while (robot.reach.getCurrentPosition()>-738){
        robot.reach.setPower(-1.0);
        telemetry.addData("reach", robot.reach.getCurrentPosition());
        telemetry.update();
        if(robot.reach.getCurrentPosition()<-650)//-550
        {
            robot.claw.setPosition(1);//grab cone
            robot.claw2.setPosition(0);
        }
        if(grabConeTimer.seconds()>2)
            break;
    }
    robot.reach.setTargetPosition(-740);//-580
    robot.reach.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.reach.setPower(1);
//////////////////////////////////////////////////////////////////////////
    sleep(250);//wait for cone capture.
    robot.wrist.setPosition(0.92);
    robot.intake1.setPosition(0.5);
    robot.intake2.setPosition(0.5);
    robot.reach.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    closerReach();

    sleep(1000);//1000
    robot.claw.setPosition(0);//open to drop cone
    robot.claw2.setPosition(1);
    sleep(300);//300
    robot.wrist.setPosition(0.73);
    robot.claw.setPosition(0.5); //stop opening
    robot.claw2.setPosition(0.5);
    sleep(50);
    gamePad_a = false;

}
    /*public void driveMecanum(){
        ///////drive wheels..Mecanum
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad2.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad2.left_stick_x;
        double yaw     =  gamepad2.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // Send calculated power to wheels

        frontLeft.setPower(rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);
        frontLeft.setPower(leftFrontPower);
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
    }*/
    /////////////distances()/////////////////////////////////
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
    ////////////////////////////////////////////////////////////////////////////////
}
