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

@TeleOp(name="Robot: Test Reach", group="Robot")
//@Disabled
public class TestReach extends LinearOpMode {
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
    ElapsedTime yPressedTimeout = new ElapsedTime();
    ElapsedTime bPressedTimeout = new ElapsedTime();
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
        //cone.setPosition(0.1);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        elapsedTime.reset();
        yPressedTimeout.reset();
        scoreTime.reset();

        //intake1.setPosition(0.5);
        //intake2.setPosition(0.5);
        reach.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        reach.setPositionPIDFCoefficients(1);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            stopper.setPosition(1);
            if(gamepad1.x){
                reach.setTargetPosition(-740);
                reach.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                reach.setPower(1.0);
                telemetry.addData("X pressed",0);
                telemetry.update();
            }
            if(gamepad1.y){
                reach.setTargetPosition(0);
                reach.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                reach.setPower(1.0);
            }
            if(gamepad1.a){
                reach.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                reach.setPower(0.5);
            }
            if(gamepad1.b){
                reach.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                reach.setPower(-0.5);
            }
            // Send telemetry message to signify robot running;
            telemetry.addData("reach", "Offset = "+reach.getCurrentPosition());
            telemetry.update();
        }
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
        reach.setPower(-1.0);
        stopper.setPosition(1);
        sleep(250);
        reach.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void takeCone(){
        claw.setPosition(0.57);
        claw2.setPosition(0.5);
        intake1.setPosition(1);
        intake2.setPosition(0);   //intake down
        wrist.setPosition(0.75);
        sleep(120);
        openReach(); //motor will be stopped after this call.
        reach.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        grabConeTimer.reset();
        while (reach.getCurrentPosition()<580){
            reach.setPower(-1.0);
            telemetry.addData("reach", reach.getCurrentPosition());
            telemetry.update();
            if(reach.getCurrentPosition()>550)//-550
            {
                claw.setPosition(1);//grab cone
                claw2.setPosition(0);
            }
            if(grabConeTimer.seconds()>2)
                break;
        }
        reach.setTargetPosition(580);//-580
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
    public void driveMecanum(){
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
    }
}
