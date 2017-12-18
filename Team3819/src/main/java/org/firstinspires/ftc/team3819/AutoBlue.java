package org.firstinspires.ftc.team3819;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team3819.Structural.RobotHardware;

/**
 * Created by 200409273 on 12/14/2017.
 */

@Autonomous(name="AutoBlue3819",group="Auto")
public class AutoBlue extends LinearOpMode {

    private RobotHardware robot = null;
    private ModernRoboticsI2cColorSensor color = null;
    private ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    int team = 1;

    public void initialize() {
        robot = new RobotHardware(hardwareMap);
        robot.init();
        color = (ModernRoboticsI2cColorSensor) hardwareMap.get(ColorSensor.class, "color");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        robot.closeClaw();
        robot.lift(.5);
        sleep(1000);
        robot.lift(0);
        int blue=0, red=0, count=0;

        robot.jewelDown();
        time.reset();
        while(time.milliseconds() < 2000){
            blue += color.blue();
            red += color.red();
            count++;
            sleep(100);
        }


        telemetry.addLine("Blue: " + Integer.valueOf(blue));
        telemetry.addLine("Red: " + Integer.valueOf(red));
        telemetry.addLine("Count: " + Integer.valueOf(count));
        telemetry.update();

        blue /= count;
        red /= count;
        if(blue>red) {
            robot.drive(.5*team);
            sleep(750);
            robot.stop();
            sleep(500);
            robot.jewelUp();
            sleep(1000);
            robot.drive(.75*team);
            sleep(1500);
            robot.stop();
        }
        else {
            robot.drive(-.5*team);
            sleep(750);
            robot.stop();
            sleep(500);
            robot.jewelUp();
            sleep(1000);
            robot.drive(.75*team);
            sleep(2000);
            robot.stop();
        }
    }
}
