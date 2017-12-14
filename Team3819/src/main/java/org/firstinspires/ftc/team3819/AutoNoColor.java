package org.firstinspires.ftc.team3819;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.team3819.Structural.RobotHardware;

import java.util.Timer;

/**
 * Created by 200409273 on 12/13/2017.
 */
@Autonomous(name="AutoNoColor")

public class AutoNoColor extends OpMode {
    RobotHardware robot = null;
    Timer timer = new Timer();

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);

        robot.init();
    }


}
