package org.firstinspires.ftc.team2981;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team2981.structural.Sensors;

/**
 * Created by 200462069 on 11/11/2017.
 */
@TeleOp(name = "ColorTest")
public class ColorTest extends OpMode {

    Sensors sensors = null;

    @Override
    public void init() {
        sensors = new Sensors(hardwareMap);
        sensors.init();
    }

    @Override
    public void loop() {
        telemetry.addData("RGB: ", "(%d, %d, %d)", sensors.getRGB()[0], sensors.getRGB()[1], sensors.getRGB()[2]);
    }
}
