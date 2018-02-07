package org.firstinspires.ftc.team2993;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2993.Hardware.RobotHardware;
import org.firstinspires.ftc.team2993.Hardware.Sensors;



@TeleOp(name="Auto - Blue", group="blue")
public class AutonomousBlue extends LinearOpMode
{
    private RobotHardware robot;
    private Sensors color;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    int team = 1;

    int addedTime = 0;

    @Override
    public void runOpMode()
    {
        robot = new RobotHardware(hardwareMap);
        color = new Sensors(hardwareMap);

        robot.sideArm.setPosition(0);

        waitForStart();

        wait(2000);
        robot.sideArm.setPosition(1);
        wait(2000);

        int colorValue = color.GetTeamColor();

        if (colorValue < 0)
            robot.SetMotors(.25 * team);
        else
        {
            robot.SetMotors(-.25 * team);
            addedTime = 500;
        }
        wait(500);

        robot.Clear();
        robot.sideArm.setPosition(0);
        wait(1000);

        robot.SetMotors(1);
        wait(1500 + addedTime);
        robot.Clear();
    }

    public void wait (int ms)
    {
        timer.reset();
        while (timer.time() < ms)
            idle();
    }
}
