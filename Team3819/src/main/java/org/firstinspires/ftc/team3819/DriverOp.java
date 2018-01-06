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
    int ziyan = 1;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        robot.init();
    }

    private void driverOne() {
        double rightStick = gamepad1.right_stick_y;
        double leftStick = gamepad1.left_stick_y;
        leftStick = (Math.abs(leftStick) > 0.05 ? (leftStick) : 0);
        rightStick = (Math.abs(rightStick) > 0.05 ? (rightStick) : 0);
        robot.driveLeft(leftStick);
        robot.driveRight(rightStick);
    }

    private void driverTwo() {
        double leftStick = gamepad2.left_stick_y;
        double rightStick = gamepad2.right_stick_y;
        leftStick = (Math.abs(leftStick) > 0.1 ? (leftStick) : 0);
        rightStick = (Math.abs(rightStick) > 0.1 ? (rightStick) : 0);
        robot.rotor(rightStick / 2 * ziyan);
        robot.lift(leftStick);
        if (gamepad2.left_bumper) {
            robot.closeClaw();
        } else if (gamepad2.right_bumper) {
            robot.openClaw();
        }

        if (gamepad2.dpad_up) robot.jewelUp();
        else if (gamepad2.dpad_down) robot.jewelDown();
        else if (gamepad2.dpad_left) robot.jewelMid();
    }
    @Override
    public void loop() {
        init();
        driverOne();
        driverTwo();

    }
}