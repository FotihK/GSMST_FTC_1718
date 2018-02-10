package org.firstinspires.ftc.team3819;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team3819.Structural.RobotHardware;

/**
 * Created by 200409273 on 12/13/2017.
 */

@TeleOp(name="TeleOp")
public class DriverOp extends OpMode {

    RobotHardware robot;
    private int wheelRun = 0;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        robot.init();
    }

    private void driverOne() {
        robot.drive(gamepad1);

        if (gamepad1.dpad_up) robot.jewelUp();
        else if (gamepad1.dpad_down) robot.jewelDown();
        else if (gamepad1.dpad_left) robot.jewelMid();
    }

    private void driverTwo() {
        double leftStick = gamepad2.left_stick_y;
        double rightStick = gamepad2.right_stick_y;
        leftStick = (Math.abs(leftStick) > 0.1 ? (leftStick) : 0);
        rightStick = (Math.abs(rightStick) > 0.1 ? (rightStick) : 0);
        robot.flip(leftStick);
        if(gamepad2.dpad_down)
            robot.levelZero();
        else if(gamepad2.dpad_right)
            robot.levelOne();
        else if(gamepad2.dpad_left)
            robot.levelTwo();
        else if(gamepad2.dpad_up)
            robot.levelThree();

        if (gamepad2.left_bumper||gamepad2.right_bumper) {
            if(wheelRun == 0) {
                robot.intake();
                wheelRun = 1;
            }
            else{
                robot.stopIntake();
                wheelRun = 0;
            }
        }

    }
    @Override
    public void loop() {
        init();
        driverOne();
        driverTwo();

    }
}