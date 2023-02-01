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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
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

@TeleOp(name="Robot: Teleop Test", group="Robot")
//@Disabled
public class TestTeleop extends LinearOpMode {
    /* Declare OpMode members. */
    public DcMotor  frontLeft = null;
    public DcMotor  frontRight = null;
    public DcMotor  backLeft = null;
    public DcMotor  backRight = null;
    public DcMotor  elevator1     = null;
    public DcMotorEx  elevator2    = null;
    public Servo    intakeServo   = null;
    public Servo    elevatorServo   = null;
    public Servo    cone   = null;
    public Servo    elevatorServo2   = null;
    public DcMotorEx dish = null;
    public DcMotorEx intake = null;
    public TouchSensor start = null;
    public TouchSensor end = null;
    double clawOffset = 0;

    public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    boolean gamePad_a = false;

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
        elevator1 = hardwareMap.get(DcMotor.class, "elevator1");
        elevator2 = hardwareMap.get(DcMotorEx.class, "elevator2");
        intakeServo = hardwareMap.get(Servo.class, "intakeservo");
        elevatorServo = hardwareMap.get(Servo.class, "elevatorservo");
        cone = hardwareMap.get(Servo.class, "cone");
        elevatorServo2 = hardwareMap.get(Servo.class, "elevatorservo2");
        dish = hardwareMap.get(DcMotorEx.class, "dish");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        start = hardwareMap.get(TouchSensor.class, "start");
        end = hardwareMap.get(TouchSensor.class, "end");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        boolean endReached, firstPass;
        long snapShot =0;

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.addData("cone position", cone.getPosition());
        telemetry.addData("elevator2",  "Offset = "+ elevator2.getCurrentPosition());
        telemetry.update();
        //////

        elevator2.setDirection(DcMotor.Direction.REVERSE);  //this has no effect what so ever!!
        elevator1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dish.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator2.setTargetPosition(-1650);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setTargetPositionTolerance(5);
        elevator2.setPositionPIDFCoefficients(3);

        endReached = false;
        firstPass = true;
        // Wait for the game to start (driver presses PLAY)
        elevatorServo.setPosition(0);
        dish.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       // cone.setPosition(0.9);
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.a)
            {elevator2.setPower(0.5);
            elevator1.setPower(0.5);}
            else{elevator1.setPower(0);
            elevator2.setPower(0);}
            if(gamepad1.b)
            {
                dish.setPower(-0.5);

            }
            else
                dish.setPower(0);



            telemetry.addData("cone position",  "Offset = "+ cone.getPosition());
            telemetry.addData("elevatorServo position",  "Offset = "+ elevatorServo.getPosition());

//            telemetry.addData("start Pressed", start.isPressed());
//            telemetry.addData("end Pressed", end.isPressed());
//            telemetry.addData("dish", "Offset = "+dish.getCurrentPosition());
//            telemetry.addData("intake", "Offset = "+intake.getCurrentPosition());

            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }

    private void moveCone(){
    }
}
