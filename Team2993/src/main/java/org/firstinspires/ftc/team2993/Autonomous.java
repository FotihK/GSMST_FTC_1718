package org.firstinspires.ftc.team2993;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2993.structural.RobotHardware;


@TeleOp(name="Auto New", group="test")
public class Autonomous extends LinearOpMode
{
    private RobotHardware robot;
    ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() throws InterruptedException
    {
        waitForStart();

        robot = new RobotHardware(hardwareMap);
        robot.init();

        robot.sideArm.setPosition(0);
        wait(2000);
        robot.sideArm.setPosition(1);

        /*
        robot.claw.setPosition(0);
        while(eTime.time() < 1000) {}
        eTime.reset();

        robot.drive(-.5);
        while(eTime.time() < 2000) {}*/
    }

    public void wait (int ms)
    {
        eTime.reset();
        while(eTime.time() < ms) {}
        eTime.reset();
    }
}