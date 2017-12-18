package org.firstinspires.ftc.team2993;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Auto - Red", group="red")
public class AutonomousRed extends AutonomousBlue
{
    @Override
    public void runOpMode()
    {
        team = -1;
        super.runOpMode();
    }
}
