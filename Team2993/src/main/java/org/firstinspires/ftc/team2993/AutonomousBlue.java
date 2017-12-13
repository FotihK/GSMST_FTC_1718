package org.firstinspires.ftc.team2993;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2993.structural.RobotHardware;
import org.firstinspires.ftc.team2993.structural.Sensors;



@TeleOp(name="Auto - Blue", group="blue")
public class AutonomousBlue extends LinearOpMode
{
    private RobotHardware robot;
    private Sensors color;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    int team = 1;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new RobotHardware(hardwareMap);
        color = new Sensors(hardwareMap);

        robot.sideArm.setPosition(0);

        waitForStart();

        wait(2000);
        robot.sideArm.setPosition(1);
        wait(2000);

        if (color.GetColor() > 0)
            robot.Turn(.25 * team);
        else
            robot.Turn(-.25 * team);
        wait(200);

        robot.clear();
        robot.sideArm.setPosition(0);
        wait(2000);

        robot.Drive(.5);
        wait(1000);

        robot.clear();
    }

    public void wait (int ms)
    {
        timer.reset();
        while(timer.time() < ms) { idle(); }
    }


}

