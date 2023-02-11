package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Test CONE", group = "robot")
public class TestConeServo extends LinearOpMode {
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
            telemetry.addData("ARM", robot.arm.getCurrentPosition());
            telemetry.update();

            if(gamepad1.a)
            {
                robot.cone.setPosition(0.1);
                robot.elevator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.elevator2.setPower(robot.elevator.getPower());
            }
            if(gamepad1.b){
                robot.cone.setPosition(0.05);
            }
            if(gamepad1.x){
                robot.cone.setPosition(0.9);
            }
        }
    }
}
