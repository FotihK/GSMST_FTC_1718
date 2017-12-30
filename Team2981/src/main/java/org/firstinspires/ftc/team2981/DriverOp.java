package org.firstinspires.ftc.team2981;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.team2981.structural.*;

import static org.firstinspires.ftc.team2981.structural.RobotHardwareMec.SystemState.*;

/**
 * Created by 200462069 on 10/14/2017.
 */

@TeleOp(name = "TeleOp", group="Regular")
public class DriverOp extends OpMode {
    private RobotHardwareMec robot;
    private Sensors sensors;
    //private Vision vision;
/*
    private Thread vuf = new Thread(new Runnable() {
        @Override
        public void run() {
            vision = new Vision(hardwareMap);
            vision.start(1);
        }
    });
*/
    @Override
    public void init() {
        robot = new RobotHardwareMec(hardwareMap);
        sensors = new Sensors(hardwareMap);

        robot.init();
        sensors.init();
        sensors.calibrate();
        robot.jewelUp();
        //vuf.start();
    }

    @Override
    public void loop(){
        double[] v = robot.drive(gamepad1);
        telemetry.addData("Y/X: ", "y: %f, x: %f", -gamepad1.left_stick_y, gamepad1.left_stick_x);
        telemetry.addData("F/S: ", "f: %f, s: %f", v[0], v[1]);
        telemetry.addData("V/T: ", "v: %f, t: %f", v[2], v[3]);
        telemetry.addData("Individual: ", "v1: %f, v2: %f, v3: %f, v4: %f", v[4], v[5], v[6], v[7]);

        if(gamepad1.a){
            robot.setIntakeLeft(FORWARD);
            robot.setIntakeRight(FORWARD);
        } else if(gamepad1.y){
            robot.setIntakeLeft(BACKWARD);
            robot.setIntakeRight(BACKWARD);
        } else {
            if (gamepad1.left_bumper) {
                robot.setIntakeLeft(BACKWARD);
            } else if (gamepad1.left_trigger > 0.05) {
                robot.setIntakeLeft(FORWARD);
            } else robot.setIntakeLeft(OFF);

            if (gamepad1.right_bumper) {
                robot.setIntakeRight(BACKWARD);
            } else if (gamepad1.right_trigger > 0.05) {
                robot.setIntakeRight(FORWARD);
            } else robot.setIntakeRight(OFF);
        }

        if(gamepad2.a){
            robot.setConveyorLeft(FORWARD);
            robot.setConveyorRight(FORWARD);
        } else if(gamepad2.y){
            robot.setConveyorLeft(BACKWARD);
            robot.setConveyorRight(BACKWARD);
        } else {
            if (gamepad2.left_bumper) {
                robot.setConveyorLeft(BACKWARD);
            } else if (gamepad2.left_trigger > 0.05) {
                robot.setConveyorLeft(FORWARD);
            } else robot.setConveyorLeft(OFF);

            if (gamepad2.right_bumper) {
                robot.setConveyorRight(BACKWARD);
            } else if (gamepad2.right_trigger > 0.05) {
                robot.setConveyorRight(FORWARD);
            } else robot.setConveyorRight(OFF);
        }
/*
        if(gamepad2.dpad_up){
            if (robot.getLiftState() == FORWARD) robot.setLift(OFF);
            else robot.setLift(FORWARD);
        } else if(gamepad2.dpad_down){
            if (robot.getLiftState() == BACKWARD) robot.setLift(OFF);
            else robot.setLift(BACKWARD);
        }
*/
        //telemetry();

    }
/*
    private void telemetry(){

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
    }
    */
}
