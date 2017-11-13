package org.firstinspires.ftc.team2993;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Auto - Red", group="blue")
public class AutonomousRed extends AutonomousBlue
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        team = -1;
        super.runOpMode();
    }
}
