package org.firstinspires.ftc.team2981;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team2981.structural.RobotHardwareMec;

/**
 * Created by 200462069 on 1/4/2018.
 */

@Autonomous(name = "autoTest")
public class AutoTest extends LinearOpMode {

    private Servo jewel;


    @Override
    public void runOpMode() throws InterruptedException {
        jewel = hardwareMap.get(Servo.class, "jewel");
        waitForStart();

        for(double ang = 0.25; ang <= 0.75; ang += 0.25){
            jewel.setPosition(ang);
            telemetry.addData("Ang: ", ang);
            telemetry.update();
            sleep(1000);
        }

        for(double ang = 0.75; ang >= 0.25; ang -= 0.25){
            jewel.setPosition(ang);
            telemetry.addData("Ang: ", ang);
            telemetry.update();
            sleep(1000);
        }


    }
}
