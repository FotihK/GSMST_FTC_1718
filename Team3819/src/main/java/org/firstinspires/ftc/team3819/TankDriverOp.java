package org.firstinspires.ftc.team3819;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team3819.Structural.RobotHardware;

/**
 * Created by 200409273 on 12/13/2017.
 */

@TeleOp(name="tankTele")
public class TankDriverOp extends OpMode {

    RobotHardware robot;
    private int wheelRun = 0;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        robot.initTele();
    }

    private void driverOne() {
        double leftStick1 = gamepad1.left_stick_y;
        leftStick1 = -(Math.abs(leftStick1) > 0.1 ? (leftStick1) : 0);
        double rightStick1 = gamepad1.right_stick_y;
        rightStick1 = -(Math.abs(rightStick1) > 0.1 ? (rightStick1) : 0);

        robot.drive(leftStick1, rightStick1);

        if (gamepad1.dpad_up) robot.jewelUp();
        else if (gamepad1.dpad_down) robot.jewelDown();
        else if (gamepad1.dpad_left) robot.jewelMid();
    }

    private void driverTwo() {
        double leftStick = gamepad2.left_stick_y;
        leftStick = (Math.abs(leftStick) > 0.1 ? (leftStick) : 0);
        double rightStick = gamepad2.right_stick_y;
        rightStick = (Math.abs(rightStick) > 0.1 ? (rightStick) : 0);

        robot.testLift(leftStick / 4);
        robot.flip(rightStick / 2);

        /*if (gamepad2.dpad_down)
            robot.levelZero();
        else if (gamepad2.dpad_right)
            robot.levelOne();
        else if (gamepad2.dpad_left)
            robot.levelTwo();
        else if (gamepad2.dpad_up)
            robot.levelThree(); */

        if (gamepad2.a) {
            robot.intake();
        }
        if (gamepad2.b) {
            robot.stopIntake();
        }
        if(gamepad2.x)
            robot.outtake();

    }

    @Override
    public void loop() {
        init();
        driverOne();
        driverTwo();

    }
}