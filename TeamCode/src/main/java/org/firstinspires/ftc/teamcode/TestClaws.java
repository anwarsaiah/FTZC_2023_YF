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
        robot.elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.elevator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()){
            robot.elevator.setPower(gamepad1.left_stick_y);
            robot.elevator2.setPower(gamepad1.left_stick_y);
            telemetry.addData("elevator : elevator2", robot.elevator.getCurrentPosition()+" "+robot.elevator2.getCurrentPosition());
            telemetry.update();
        }
    }
}
