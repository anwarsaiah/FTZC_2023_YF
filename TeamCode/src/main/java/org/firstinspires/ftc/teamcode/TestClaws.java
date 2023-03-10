package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Test Elevator", group = "robot")
public class TestClaws extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(this);
        robot.init();
        robot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        while (opModeIsActive()){

            telemetry.addData("Elevator", robot.elevator.getCurrentPosition());
            telemetry.addData("Elevator2", robot.elevator2.getCurrentPosition());
            telemetry.update();

           if(gamepad1.a)
           {
               robot.claw.setPosition(0.0);
               robot.claw2.setPosition(1.0);
               robot.elevator.setTargetPosition(500);//-1850
               robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               robot.elevator.setPower(1);

               robot.elevator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
               robot.elevator2.setPower(robot.elevator.getPower());
           }
            if(gamepad1.b){
                robot.claw.setPosition(0.0);//close
                robot.claw2.setPosition(1.0);
                robot.elevator.setPower(0);
                robot.elevator2.setPower(0);
            }
            if(gamepad1.x){
                robot.cone.setPosition(0.9);
            }
            if(gamepad1.y){
                robot.cone.setPosition(0.04);
            }
        }
    }
}
