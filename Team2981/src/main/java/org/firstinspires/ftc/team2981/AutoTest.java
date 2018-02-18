package org.firstinspires.ftc.team2981;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team2981.structural.RobotHardwareMec;

/**
 * Created by 200462069 on 1/4/2018.
 */

@Autonomous(name = "autoTest")
public class AutoTest extends LinearOpMode {

    private RobotHardwareMec robot = null;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardwareMec(hardwareMap);
        robot.init();
        robot.jewelUp();
        robot.resetEnc();
        waitForStart();
        robot.forwardEnc(0.9, 82);
        boom();
        robot.turnEnc(0.9, 90);
        boom();
        robot.forwardEnc(0.9, 96);
        boom();
        robot.turnEnc(0.9, 90);
        boom();
        robot.forwardEnc(0.9, 85);
        boom();

        robot.turnEnc(0.9, 180);
        boom();
        robot.forwardEnc(0.9, 85);
        boom();
        robot.strafeEnc(-0.9, 96);
        boom();
        robot.forwardEnc(-0.9, 85);
        boom();


    }

    public void boom(){
        while(robot.isBusy() && opModeIsActive()) idle();
        robot.stop();
        sleep(1500);
    }
}
