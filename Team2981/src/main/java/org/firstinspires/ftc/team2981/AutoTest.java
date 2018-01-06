package org.firstinspires.ftc.team2981;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team2981.structural.RobotHardwareMec;

/**
 * Created by 200462069 on 1/4/2018.
 */

@Autonomous(name = "autoTest")
public class AutoTest extends LinearOpMode {

    private RobotHardwareMec robot;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardwareMec(hardwareMap);
        robot.init();
        waitForStart();
        sleep(2000);

        robot.drive(0.5, 0, 0);
        sleep(500);

        robot.stop();
        sleep(750);

        robot.drive(-0.5, 0, 0);
        sleep(500);

        robot.stop();
        sleep(750);

        robot.drive(0, 0.5, 0);
        sleep(500);

        robot.stop();
        sleep(750);

        robot.drive(0, -0.5, 0);
        sleep(500);

        robot.stop();
        sleep(750);

        robot.drive(0, 0, 0.5);
        sleep(500);

        robot.stop();
        sleep(750);

        robot.drive(0, 0, -0.5);
        sleep(500);

        robot.stop();
        sleep(750);

    }
}
