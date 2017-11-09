package org.firstinspires.ftc.team2993;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.team2993.structural.RobotHardware;
import org.firstinspires.ftc.team2993.structural.Sensors;



@TeleOp(name="Auto - Blue", group="test")
public class AutonomousBlue extends LinearOpMode
{
    private RobotHardware robot;
    private Sensors color;
    ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new RobotHardware(hardwareMap);
        color = new Sensors(hardwareMap);
        robot.init();
        color.init();

        robot.sideArm.setPosition(.5);

        waitForStart();

        Wait(2000);
        robot.sideArm.setPosition(1);
        Wait(2000);

        if (GetColor() > 0)
            robot.turn(.25);
        else
            robot.turn(-.25);
        Wait(500);
        robot.sideArm.setPosition(.5);
        wait(1000);

        robot.drive(.5);
        Wait(2000);
    }

    public int GetColor()
    {
        return color.color.red() - color.color.blue();
    }

    public void Wait (int ms)
    {
        eTime.reset();
        while(eTime.time() < ms) {}
    }
}

