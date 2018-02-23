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
    int team = -1;
    int place = 0;

    public void initialize() {
        robot = new RobotHardware(hardwareMap);
        robot.init();
        color = (ModernRoboticsI2cColorSensor) hardwareMap.get(ColorSensor.class, "color");
    }

    public void idler() {
        while (robot.isBusy() && opModeIsActive())
            idle();
        stop();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        robot.jewelUp();
        waitForStart();

        sleep(500);

        int blue = 0, red = 0, count = 0;

        robot.jewelDown();
        time.reset();
        while (time.milliseconds() < 2000) {
            blue += color.blue();
            red += color.red();
            count++;
            sleep(100);
        }

        String place = robot.vuphoria(); //read Image

        telemetry.addLine("Blue: " + Integer.valueOf(blue));
        telemetry.addLine("Red: " + Integer.valueOf(red));
        telemetry.addLine("Count: " + Integer.valueOf(count));
        telemetry.update();

        blue /= count;
        red /= count;
        if (blue > red) {
            robot.driveDistIn(-4 * team, .2);
            idler();
            sleep(200);
            robot.jewelUp();
            sleep(500);
            robot.driveDistIn(4 * team, .2);
            idler();
            sleep(500);
        } else { //27 34 42
            robot.driveDistIn(4 * team, .2);
            idler();
            sleep(200);
            robot.jewelUp();
            sleep(500);
            robot.driveDistIn(-4 * team, .2);
            idler();
            sleep(500);
        }

        robot.driveDistIn(28, .5);
        idler();
        robot.right();                               //moves off platform & rotates
        idler();

        if ((place.equals("left") && team == -1) || (place.equals("right") && team == 1)) {
            robot.driveDistIn(6 * team, .75);
            idler();
        } else if (place.equals("center")) {
            robot.driveDistIn(13 * team, .75);
            idler();
        } else {
            robot.driveDistIn(20 * team, .75);
            idler();
        }

        robot.right();  //rotates to have back face grid
        idler();

        robot.flip(.5);
        sleep(750);             //flips cube off
        robot.flip(-.8);
        sleep(700);
        robot.flip(0);

        robot.driveDistIn(6,.5);
        idler();
        robot.driveDistIn(-4,.5);   //pushes in multiple times
        idler();
        robot.driveDistIn(5,.5);
        idler();
        robot.driveDistIn(-4,.5);
        idler();

    }
}