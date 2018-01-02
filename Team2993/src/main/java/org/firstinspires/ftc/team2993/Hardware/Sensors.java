package org.firstinspires.ftc.team2993.Hardware;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Sensors
{
    public ModernRoboticsI2cColorSensor color;
    private HardwareMap Map;

    public Sensors(HardwareMap map)
    {
        Map = map;
        Initialize();
    }

    public void Initialize()
    {
        color = Map.get(ModernRoboticsI2cColorSensor.class, "color");
    }

    public int GetTeamColor()
    {
        return color.blue() - color.red();
    }
}