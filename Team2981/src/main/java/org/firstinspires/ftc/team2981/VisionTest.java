package org.firstinspires.ftc.team2981;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.team2981.structural.Vision;

/**
 * Created by 200462069 on 11/10/2017.
 */

@Autonomous(name = "VisionTest")
public class VisionTest extends LinearOpMode {

    private Vision vision;
    private boolean blueTeam;

    private Thread visionThread = new Thread(new Runnable() {
        @Override
        public void run() {
            vision.start(1);

            while (!isStarted()) {
                if (gamepad1.dpad_up) {
                    vision.switchLocation(1);
                    blueTeam = true;
                } else if (gamepad1.dpad_down) {
                    vision.switchLocation(2);
                    blueTeam = true;
                } else if (gamepad1.dpad_left) {
                    vision.switchLocation(3);
                    blueTeam = false;
                } else if (gamepad1.dpad_right) {
                    vision.switchLocation(4);
                    blueTeam = false;
                }
                telemetry.addData("Team ", blueTeam ? "Blue" : "Red");
                telemetry.addData("Location: ", vision.getLoc());
                telemetry.update();
            }

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
                } catch (Exception ignored) {
                    telemetry.addData("Error: ", " Location Access Failed");
                }
                try {
                    trans = vision.getTrans();
                } catch (Exception ignored) {
                    telemetry.addData("Error: ", " Translation Access Failed");
                }
                try {
                    rot = vision.getRot();
                } catch (Exception ignored) {
                    telemetry.addData("Error: ", " Rotation Access Failed");
                }
                try {
                    telemetry.addData("Position: ", loc != null ? loc.formatAsTransform() : "Unknown");
                    if (trans != null) {
                        telemetry.addData("Translation: ", "(%f, %f, %f)", trans[0], trans[1], trans[2]);
                    } else telemetry.addData("Translation: ", "Unknown");
                    if (rot != null) {
                        telemetry.addData("Rotation: ", "(%f, %f, %f)", rot[0], rot[1], rot[2]);
                    } else telemetry.addData("Rotation: ", "Unknown");
                } catch (Exception e) {
                    telemetry.addData("Error: ", " Data Access Failed");
                }
                telemetry.update();
            }
        }
    });

    @Override
    public void runOpMode() throws InterruptedException {
        vision = new Vision(hardwareMap);
        visionThread.start();
        waitForStart();
        while (!isStopRequested()) {
            sleep(10);
        }
    }
}
