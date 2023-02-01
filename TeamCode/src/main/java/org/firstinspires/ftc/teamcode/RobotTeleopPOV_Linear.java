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

import com.qualcomm.hardware.motors.RevRobotics20HdHexMotor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.R;

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
    public DcMotor  frontLeft = null;
    public DcMotor  frontRight = null;
    public DcMotor  backLeft = null;
    public DcMotor  backRight = null;

    public DcMotor  elevator     = null;
    public DcMotorEx  arm    = null;
    public DcMotorEx reach = null;
    public DcMotorEx intake = null;

    public Servo    wrist   = null;
    public Servo    claw   = null;
    public Servo    claw2   = null;
    public Servo    cone   = null;
    public Servo   intake1 = null;
    public Servo   intake2 = null;
    public Servo   stopper = null;

    public RevTouchSensor touch = null;


    public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    boolean gamePad_a = false;
    public boolean liftingElevator;
    ElapsedTime elapsedTime = new ElapsedTime();
    ElapsedTime scoreTime = new ElapsedTime();
    ElapsedTime grabConeTimer = new ElapsedTime();
    ElapsedTime gamePad_aTimer = new ElapsedTime();
    boolean endReached, startReached, firstPass, intakeUP, intakeCone = false;
    public static final double CONE_OPEN = 0.9;
    public static final double CONE_FOLDED = 0.1;


    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;

        // Define and Initialize Motors

        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        backLeft = hardwareMap.get(DcMotor.class, "backleft");
        backRight = hardwareMap.get(DcMotor.class, "backright");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        reach = hardwareMap.get(DcMotorEx.class, "reach");
        stopper = hardwareMap.get(Servo.class, "stopper");



        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        cone = hardwareMap.get(Servo.class, "cone");
        claw2 = hardwareMap.get(Servo.class, "claw2");
        intake1 = hardwareMap.get(Servo.class,"intake1");
        intake2 = hardwareMap.get(Servo.class,"intake2");


        touch = hardwareMap.get(RevTouchSensor.class,"touch");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);



        // Send telemetry message to signify robot waiting;

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.addData("cone position", cone.getPosition());
        telemetry.update();
        //////
        endReached = false;
        startReached = false;
        firstPass = true;
        liftingElevator = false;
        intakeUP=false;
        telemetry.addData("reach:", reach.getCurrentPosition());
        telemetry.update();
        cone.setPosition(0.1);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        elapsedTime.reset();
        scoreTime.reset();

        //intake1.setPosition(0.5);
        //intake2.setPosition(0.5);
        reach.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        reach.setPositionPIDFCoefficients(1);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // run until the end of the match (driver presses STOP)
while (opModeIsActive()) {
closerReach(); //Motor in run without encoder mode and idle, after call.
    if(gamepad1.x){
        reach.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        reach.setPower(-0.6);
    }

if(gamepad1.a && !gamePad_a)
{
    gamePad_a = true;
    grabConeTimer.reset();
    gamePad_aTimer.reset();
}
if(gamePad_a)
{
    claw.setPosition(0.57);
    claw2.setPosition(0.5);
    intake1.setPosition(1);
    intake2.setPosition(0);   //intake down
    wrist.setPosition(0.75);
    sleep(120);
    openReach(); //motor will be stopped after this call.
    reach.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    while (reach.getCurrentPosition()>-580){
        reach.setPower(-1.0);
        telemetry.addData("reach", reach.getCurrentPosition());
        telemetry.update();
        if(reach.getCurrentPosition()<-550)//-480
        {
           claw.setPosition(1);//grab cone
           claw2.setPosition(0);
        }

    }
    reach.setTargetPosition(-580);
    reach.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    reach.setPower(1);
//////////////////////////////////////////////////////////////////////////
    sleep(250);//wait for cone capture.
    wrist.setPosition(0.92);
    intake1.setPosition(0.5);
    intake2.setPosition(0.5);
    reach.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    closerReach();

    sleep(1000);//1000
    claw.setPosition(0);//open to drop cone
    claw2.setPosition(1);
    sleep(300);//300
    wrist.setPosition(0.73);
    claw.setPosition(0.5); //stop opening
    claw2.setPosition(0.5);
    sleep(50);
    gamePad_a = false;

}

if(gamepad1.b){
    claw.setPosition(1);//close grip
    claw2.setPosition(0);
//    elevator.setPower(-1.0);
//    sleep(800);
    elevator.setTargetPosition(-1850);
    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    elevator.setPower(1);
}
////////////////////////////////////////////
    if(elevator.getCurrentPosition()<-1200){
    cone.setPosition(CONE_OPEN);
    ///////////////////back
    if(elevator.getCurrentPosition()<-1450)
    cone.setPosition(CONE_FOLDED);
    if(elevator.getCurrentPosition()<-1750){
    cone.getController().pwmDisable();
    elevator.setTargetPosition(0);
    }
    }
    ///////////////////////////////////////////
            drive =  gamepad1.left_stick_y;
            turn  =  -gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            left  = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            // Output the safe vales to the motor drives.
            frontLeft.setPower(left);
            backLeft.setPower(left);
            frontRight.setPower(right);
            backRight.setPower(right);

              // Send telemetry message to signify robot running;
              //telemetry.addData("elevator1",  "Offset = "+ elevator1.getCurrentPosition());
              telemetry.addData("elevator",  "Offset = "+ elevator.getCurrentPosition());


              telemetry.addData("reach", "Offset = "+reach.getCurrentPosition());

              telemetry.addData("arm", "Offset = "+arm.getCurrentPosition());
//            telemetry.addData("left",  "%.2f", left);
//            telemetry.addData("right", "%.2f", right);
              telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }

   public  void  grabCone(){
        if(grabConeTimer.seconds()<0.35)
            reach.setPower(-0.7);
        else reach.setPower(0);
   }

public void closerReach(){


    if(!touch.isPressed())
    {
        reach.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        reach.setPower(1.0);
    }
    else{
        stopper.setPosition(0);//close hook!
        sleep(250);
        reach.setPower(0);
    }
}
public void openReach(){
    reach.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    reach.setPower(1.0);
    stopper.setPosition(1);
    sleep(250);
    reach.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
}
}
