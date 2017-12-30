package org.firstinspires.ftc.team2993;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2993.structural.RobotHardware;
import org.firstinspires.ftc.team2993.structural.Sensors;


@TeleOp(name = "TeleOp - 2993", group="Regular")
public class DriverOp extends OpMode
{
    private RobotHardware robot;
    private Sensors color;

    private final double threshhold = 0.1;
    private double speed = .5d;
    private double ClawSpeed = 0d;
    private boolean clawOff = true;

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void init()
    {
        robot = new RobotHardware(hardwareMap);
        color = new Sensors(hardwareMap);

        robot.sideArm.setPosition(0);
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
            robot.SetArm(-.6d);
        else if (gamepad1.x)
            robot.SetArm(.3d);
        else
            robot.SetArm(0);

        if      (gamepad1.left_bumper)
            ClawSpeed = -.3d;
        else if (gamepad1.right_bumper)
            ClawSpeed = .3d;
        else if (!clawOff)
            ClawSpeed = .1d;
        else if (clawOff)
            ClawSpeed = 0d;

        robot.SetClaw(ClawSpeed);

        if (gamepad1.dpad_left && timer.time() > 250)
        {
            timer.reset();
            robot.sideArm.setPosition(0);
        }

        if (gamepad1.a && timer.time() > 500)
        {
            timer.reset();
            speed = speed == .5d ? .25d : .5d;
        }



        if (gamepad1.y)   // Debug
        {
            telemetry.addLine(String.valueOf(color.GetColor()));
            telemetry.addLine(String.valueOf(gamepad1.left_trigger));
            telemetry.addLine(String.valueOf(getRuntime()));
            telemetry.update();
        }
    }
}