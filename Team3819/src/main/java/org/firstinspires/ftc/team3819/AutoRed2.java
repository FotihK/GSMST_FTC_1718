package org.firstinspires.ftc.team3819;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 200409273 on 1/6/2018.
 */
@Autonomous(name="AutoRed2")
public class AutoRed2 extends AutoBlue2 {
    @Override
    public void runOpMode() throws InterruptedException {
        team = 1;
        super.runOpMode();
    }
}
