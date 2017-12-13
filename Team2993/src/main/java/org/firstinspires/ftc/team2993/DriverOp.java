package org.firstinspires.ftc.team2993;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team2993.structural.RobotHardware;
import org.firstinspires.ftc.team2993.structural.Sensors;


@TeleOp(name = "TeleOp - 2993", group="Regular")
public class DriverOp extends OpMode
{
    private RobotHardware robot;
    private Sensors color;

    private final double threshhold = 0.1;
    private double speed = .5d;

    @Override
    public void init()
    {
        robot = new RobotHardware(hardwareMap);
        color = new Sensors(hardwareMap);

        //robot.sideArm.setPosition(1);
    }

    @Override
    public void loop()
    {
        driverOne();
    }

    @Override
    public void stop()
    {
        robot.SetDrive(0,0);
    }

    public void driverOne()
    {
        double leftStick = gamepad1.left_stick_y;
        double rightStick = gamepad1.right_stick_y;
        leftStick = (Math.abs(leftStick) > threshhold ? leftStick : 0);
        rightStick = (Math.abs(rightStick) > threshhold ? rightStick : 0);

        robot.SetLeft(leftStick * speed);
        robot.SetRight(rightStick * speed);

        if      (gamepad1.b)
            robot.SetArm(-.6);
        else if (gamepad1.x)
            robot.SetArm(.3);
        else
            robot.SetArm(0);

        if      (gamepad1.left_bumper)
            robot.SetClaw(-.3f);
        else if (gamepad1.right_bumper)
            robot.SetClaw(.3f);
        else
            robot.SetClaw(0f);

        if (gamepad1.a)
            speed = speed == 1d ? .5d : 1d;



        if (gamepad1.y)
        {
            telemetry.addLine(String.valueOf(color.GetColor()));
            telemetry.update();
        }
    }
}