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
    int team = -1;

    public void initialize() {
        robot = new RobotHardware(hardwareMap);
        robot.init();
        color = (ModernRoboticsI2cColorSensor) hardwareMap.get(ColorSensor.class, "color");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        robot.closeClaw();
        robot.jewelUp();
        waitForStart();

        robot.closeClaw();
        robot.lift(-.75);
        sleep(500);
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
            robot.drive(.25*team);
            sleep(500); //forward 3/8 or .375
            robot.stop();
            sleep(500);
            robot.jewelUp();
            sleep(500);
            robot.drive(-.25*team);
            sleep(500);
            robot.stop();
            sleep(500);
            robot.drive(.5*team);
            sleep(2750); // forward 1.125
            robot.stop(); // total movement 1.5
        }
        else {
            robot.drive(-.25*team);
            sleep(500); //backwards 3/8 or .375
            robot.stop();
            sleep(500);
            robot.jewelUp();
            sleep(500);
            robot.drive(.25*team);
            sleep(500);
            robot.stop();
            sleep(500);
            robot.drive(.5*team);
            sleep(2750); //forwards 1.875
            robot.stop(); //total movement 1.5
        }

            robot.driveLeft(.25);
            robot.driveRight(-.25);

        sleep(1600);
        robot.stop();
        sleep(500);
        robot.drive(.25*team);
        sleep(750);
        robot.stop();
        robot.openClaw();
        sleep(500);
        robot.drive(-.25*team);
        sleep(500);
        robot.stop();
    }
}
