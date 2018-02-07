package org.firstinspires.ftc.team2981;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2981.structural.RobotHardwareMec;
import org.firstinspires.ftc.team2981.structural.Sensors;
import org.firstinspires.ftc.team2981.structural.Vision;

import static org.firstinspires.ftc.team2981.structural.RobotHardwareMec.SystemState.*;


/**
 * Created by 200462069 on 10/18/2017.
 */
@Autonomous(name = "Auto", group = "Auto")
public class Auto extends LinearOpMode {

    private RobotHardwareMec robot;
    private Sensors sensors;
    private Vision vision;

    private boolean blue;
    private ElapsedTime meaner = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    public void initialize() {
        robot = new RobotHardwareMec(hardwareMap);
        sensors = new Sensors(hardwareMap);
        vision = new Vision(hardwareMap);

        robot.init();
        sensors.init();
        visionThread.start();
        robot.jewelUp();

        waitForStart();
    }

    private Thread visionThread = new Thread(new Runnable() {
        @Override
        public void run() {
            vision.start(1);

            while (!isStarted() && !isStopRequested()) {
                if (gamepad1.dpad_up) {
                    vision.switchLocation(1);
                    blue = true;
                } else if (gamepad1.dpad_down) {
                    vision.switchLocation(2);
                    blue = true;
                } else if (gamepad1.dpad_left) {
                    vision.switchLocation(3);
                    blue = false;
                } else if (gamepad1.dpad_right) {
                    vision.switchLocation(4);
                    blue = false;
                }
                telemetry.addData("Team ", blue ? "Blue" : "Red");
                telemetry.addData("Location: ", vision.getLoc() + " : " + vision.getLocName());
                telemetry.update();
            }
/*
            while (opModeIsActive()) {
                OpenGLMatrix loc = null;
                float[] trans = null;
                float[] rot = null;
                try {
                    telemetry.addData("VuMark ", "%s visible", vision.trackMark());
                } catch (Exception ignored) {
                    telemetry.addData("Error: ", " VuMark Access Failed");
                }
                try {
                    loc = vision.trackLocation();
                    telemetry.addData("Position: ", loc != null ? loc.formatAsTransform() : "Unknown");
                } catch (Exception ignored) {
                    telemetry.addData("Error: ", " Location Access Failed");
                }
                try {
                    trans = vision.getTrans();
                    if (trans != null) {
                        telemetry.addData("Translation: ", "(%f, %f, %f)", trans[0], trans[1], trans[2]);
                    } else telemetry.addData("Translation: ", "Unknown");
                } catch (Exception ignored) {
                    telemetry.addData("Error: ", " Translation Access Failed");
                }
                try {
                    rot = vision.getRot();
                    if (rot != null) {
                        telemetry.addData("Rotation: ", "(%f, %f, %f)", rot[0], rot[1], rot[2]);
                    } else telemetry.addData("Rotation: ", "Unknown");
                } catch (Exception ignored) {
                    telemetry.addData("Error: ", " Rotation Access Failed");
                }
                telemetry.addData("RGB: ", "(%d, %d, %d)", sensors.getRGB()[0], sensors.getRGB()[1], sensors.getRGB()[2]);
                telemetry.update();
            }
            */
            vision.stop();
        }
    });


    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        sleep(1500);
        robot.jewelDown();
        meaner.reset();
        int blue = 0, red = 0, count = 0;
        while (meaner.milliseconds() < 2000 && opModeIsActive()) {
            blue += sensors.getRGB()[2];
            red += sensors.getRGB()[0];
            count++;
            sleep(50);
        }
        blue /= count;
        red /= count;

        telemetry.addData("B/R/C: ", "%d/%d/%d", blue, red, count);
        telemetry.update();
        sleep(1000);

        if(red > blue){
            if(this.blue) robot.drive(-0.45, 0, 0);
            else robot.drive(0.45, 0, 0);
        } else {
            if(this.blue) robot.drive(0.45, 0, 0);
            else robot.drive(-0.45, 0, 0);
        }

        sleep(350);
        robot.stop();
        robot.jewelUp();
        sleep(1000);

        if(red > blue){
            if(this.blue) robot.drive(0.45, 0, 0);
            else robot.drive(-0.45, 0, 0);
        } else {
            if(this.blue) robot.drive(-0.45, 0, 0);
            else robot.drive(0.45, 0, 0);
        }

        sleep(550);
        robot.stop();

        sleep(5000);

        if(this.blue){
            robot.drive(-0.67, 0, 0);
        } else {
            robot.drive(0.7, 0, 0);
        }
        sleep(1400);
        robot.stop();
        sleep(1000);

        switch(vision.getLoc()){
            case 1:
            case 3:
                robot.drive(0, -0.65, 0);
                break;
            case 2:
            case 4:
                robot.drive(0, 0.4, 0);
        }
        sleep(1300);
        robot.stop();

        if(vision.getLoc() == 3) {

            robot.setConveyorLeft(FORWARD);
            robot.setConveyorRight(FORWARD);

            sleep(250);

            robot.setIntakeRight(FORWARD);
            robot.setIntakeLeft(FORWARD);

            sleep(1750);

            robot.setConveyorLeft(OFF);
            robot.setConveyorRight(OFF);

            for (int i = 0; i < 3; i++) {
                if (this.blue) {
                    robot.drive(0.5, 0, 0);
                } else {
                    robot.drive(-0.5, 0, 0);
                }

                sleep(850);
                if (this.blue) {
                    robot.drive(-0.5, 0, 0);
                } else {
                    robot.drive(0.5, 0, 0);
                }
                sleep(850);
            }

            robot.stop();

            if (this.blue) {
                robot.drive(0.5, 0, 0);
            } else {
                robot.drive(-0.5, 0, 0);
            }

            sleep(450);
            robot.stop();
        } else if(vision.getLoc() == 1) {
            robot.setConveyorLeft(BACKWARD);
            robot.setConveyorRight(BACKWARD);

            sleep(1750);

            for (int i = 0; i < 3; i++) {
                if (this.blue) {
                    robot.drive(0.5, 0, 0);
                } else {
                    robot.drive(-0.5, 0, 0);
                }

                sleep(850);
                if (this.blue) {
                    robot.drive(-0.525, 0, 0);
                } else {
                    robot.drive(0.525, 0, 0);
                }
                sleep(850);
            }

            robot.stop();

            if (this.blue) {
                robot.drive(0.5, 0, 0);
            } else {
                robot.drive(-0.5, 0, 0);
            }

            sleep(450);
            robot.stop();
        }


    }
}
