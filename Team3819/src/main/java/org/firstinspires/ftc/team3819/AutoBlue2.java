package org.firstinspires.ftc.team3819;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// Jace is dumb

import org.firstinspires.ftc.team3819.Structural.RobotHardware;

/**
 * Created by 200409273 on 12/14/2017.
 */
@Autonomous(name="AutoBlue2",group="Auto")
public class AutoBlue2 extends LinearOpMode {

    private RobotHardware robot = null;
    private ModernRoboticsI2cColorSensor color = null;
    private ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    int team = 1;
    String key = null;

    public void initialize() {
        robot = new RobotHardware(hardwareMap);
        robot.init();
        color = (ModernRoboticsI2cColorSensor) hardwareMap.get(ColorSensor.class, "color");
    }

    public void idler() {
        time.reset();
        while (robot.isBusy() && opModeIsActive() && time.milliseconds()<3000)
        {idle();}
        robot.stop();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        robot.jewelUp();
        waitForStart();

        robot.intake();
        sleep(750);
        robot.stopIntake();
        key = robot.vuphoria().toLowerCase();

        robot.noEncode();
        robot.drive(0,0,.25);
        sleep(700);
        robot.stop();
        sleep(200);
        time.reset();
        while(time.milliseconds()<1500 && key.equals("unknown")&&opModeIsActive())
            key = robot.vuphoria().toLowerCase(); //read Image
        sleep(200);
        robot.drive(0,0,-.25);
        sleep(700);
        robot.stop();
        sleep(200);
        robot.encode();


        int blue = 0, red = 0, count = 0;

        robot.jewelDown();
        time.reset();
        while (time.milliseconds() < 2000) {
            blue += color.blue();
            red += color.red();
            count++;
            sleep(100);
        }

        telemetry.addLine("Blue: " + Integer.valueOf(blue));
        telemetry.addLine("Red: " + Integer.valueOf(red));
        telemetry.addLine("Count: " + Integer.valueOf(count));
        telemetry.addLine(key);
        telemetry.update();

        blue /= count;
        red /= count;
        if (blue > red) {
            robot.driveDistIn(-3 * team, .3);
            idler();
            sleep(200);
            robot.jewelUp();
            sleep(500);
            robot.driveDistIn(3 * team, .3);
            idler();
            sleep(500);
        } else { //27 34 42
            robot.driveDistIn(3 * team, .3);
            idler();
            sleep(200);
            robot.jewelUp();
            sleep(500);
            robot.driveDistIn(-3 * team, .3);
            idler();
            sleep(500);
        }

        robot.driveDistIn(28*team, .5);
        idler();
        robot.noEncode();
        robot.drive(0,0,-1);
        sleep(1200);
        robot.stop();
        robot.encode();

        if ((key.equals("left") && team == -1) || (key.equals("right") && team == 1)) {
            robot.driveDistIn(9, .75);
            idler();
        } else if (key.equals("center")) {
            robot.driveDistIn(16, .75);
            idler();
        } else {
            robot.driveDistIn(22, .75);
            idler();
        }

        robot.noEncode();
        robot.drive(0,0,-1*team);
        sleep(1200);
        robot.stop();
        robot.encode();

        robot.flip(-.7);
        sleep(750);             //flips cube off
        robot.flip(.7);
        sleep(1000);
        robot.flip(0);

        robot.driveDistIn(-9,.7);
        idler();
        robot.driveDistIn(4,.5);   //pushes in multiple times
        idler();
        robot.driveDistIn(-5,.7);
        idler();
        robot.driveDistIn(4,.5);
        idler();

    }
}