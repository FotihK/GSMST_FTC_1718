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
    public void runOpMode()
    {
        robot = new RobotHardware(hardwareMap);
        color = new Sensors(hardwareMap);

        robot.sideArm.setPosition(0);

        waitForStart();

        wait(2000);
        robot.sideArm.setPosition(1);
        wait(2000);

        int colorValue = color.GetColor();

        telemetry.addLine(String.valueOf(colorValue));
        telemetry.addLine("Blue:  " + String.valueOf(color.color.blue()));
        telemetry.addLine("Red:   " + String.valueOf(color.color.red()));
        telemetry.update();

        if (colorValue > 0)
            robot.Drive(.25 * team);
        else
            robot.Drive(-.25 * team);
        wait(500);

        robot.clear();
        robot.sideArm.setPosition(0);
        wait(1000);

        // HOLA,MEINE NAME IS JACE KIM.OHAYOO Watashi wa Jace desu.

        robot.clear();
    }

    public void wait (int ms)
    {
        timer.reset();
        while(timer.time() < ms) { idle(); }
    }
}
