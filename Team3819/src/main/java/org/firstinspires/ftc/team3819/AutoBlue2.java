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

    public void initialize() {
        robot = new RobotHardware(hardwareMap);
        robot.init();
        color = (ModernRoboticsI2cColorSensor) hardwareMap.get(ColorSensor.class, "color");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        robot.jewelUp();
        waitForStart();


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
            robot.driveDistIn(-4, .2);
            sleep(500);
            robot.jewelUp();
            sleep(200);
            robot.driveDistIn(4, .2);
        }
        else {
            robot.driveDistIn(4, .2);
            sleep(500);
            robot.jewelUp();
            sleep(200);
            robot.driveDistIn(-4, .2);
        }

        robot.driveDistIn(30, .75);
        sleep(500);
        robot.left();
        if(place==0)
            robot.driveDistIn(27*team, .75);
        else if(place==1)
            robot.driveDistIn(34*team, .75);
        else
            robot.driveDistIn(42*team,.75);

        robot.left();

        robot.flip(.5);
        sleep(500);             //flips cube off
        robot.flip(0);

        sleep(500);             //
        robot.driveDistIn(5,.1);   // Pushed cube in, backs up, pushes again, backs up
        robot.driveDistIn(-5,.1);   //
        robot.driveDistIn(5,.1);   //
        robot.driveDistIn(-4,.1);  //
      }
}