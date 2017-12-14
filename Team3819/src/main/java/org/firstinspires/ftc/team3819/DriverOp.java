package org.firstinspires.ftc.team3819;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team3819.Structural.RobotHardware;

/**
 * Created by 200409273 on 12/13/2017.
 */

@TeleOp(name="TeleOp")
public class DriverOp extends OpMode {

    RobotHardware robot = null;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);

        robot.init();
    }
    private void driverOne(){
        double rightStick = gamepad1.left_stick_y;
        double leftStick = gamepad1.right_stick_y;
        leftStick = (Math.abs(leftStick) > 0.05 ? (leftStick) : 0);
        rightStick = (Math.abs(rightStick) > 0.05 ? (rightStick) : 0);
        robot.driveLeft(rightStick);
        robot.driveRight(leftStick);
    }

    private void driverTwo() {
        double leftStick = gamepad2.left_stick_y;
        leftStick = (Math.abs(leftStick) > 0.1 ? (leftStick): 0);
        robot.lift(leftStick);
        if(gamepad2.left_bumper) robot.openClaw();
        else if(gamepad2.right_bumper) robot.closeClaw();
        if(gamepad2.a) robot.jewelUp();
        else if(gamepad2.b) robot.jewelDown();
    }

    @Override
    public void loop() {
        driverOne();
    }
}
