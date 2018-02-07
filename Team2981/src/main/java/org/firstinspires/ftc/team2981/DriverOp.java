package org.firstinspires.ftc.team2981;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.team2981.structural.*;

import static org.firstinspires.ftc.team2981.structural.RobotHardwareMec.SystemState.*;

/**
 * Created by 200462069 on 10/14/2017.
 */

@TeleOp(name = "TeleOp", group = "Regular")
public class DriverOp extends OpMode {
    private RobotHardwareMec robot;
    private Sensors sensors;

    @Override
    public void init() {
        robot = new RobotHardwareMec(hardwareMap);
        sensors = new Sensors(hardwareMap);

        robot.init();
        sensors.init();
        sensors.calibrate();
        robot.jewelUp();
    }

    @Override
    public void loop() {
        double[] v = robot.drive(gamepad1);

        if (gamepad1.a) {
            robot.setIntakeLeft(FORWARD);
            robot.setIntakeRight(FORWARD);
        } else if (gamepad1.y) {
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

        if (gamepad2.a) {
            robot.setConveyorLeft(FORWARD);
            robot.setConveyorRight(FORWARD);
        } else if (gamepad2.y) {
            robot.setConveyorLeft(BACKWARD);
            robot.setConveyorRight(BACKWARD);
        } else {
            if (gamepad2.left_bumper) {
                robot.setConveyorLeft(BACKWARD);
            } else if (gamepad2.left_trigger > 0.05) {
                robot.setConveyorLeft(FORWARD);
            } else {
                robot.setConveyorLeft(OFF);
            }

            if (gamepad2.right_bumper) {
                robot.setConveyorRight(BACKWARD);
            } else if (gamepad2.right_trigger > 0.05) {
                robot.setConveyorRight(FORWARD);
            } else {
                robot.setConveyorRight(OFF);
            }
        }

        if (gamepad2.dpad_up) {
            robot.setLiftLeft(FORWARD);
            robot.setLiftRight(FORWARD);
        } else if (gamepad2.dpad_down) {
            robot.setLiftLeft(BACKWARD);
            robot.setLiftRight(BACKWARD);
        } else {
            if(gamepad2.dpad_left){
                robot.setLiftLeft(FORWARD);
            } else if(gamepad2.x){
                robot.setLiftLeft(BACKWARD);
            } else robot.setLiftLeft(OFF);

            if(gamepad2.dpad_right){
                robot.setLiftRight(FORWARD);
            } else if(gamepad2.b){
                robot.setLiftRight(BACKWARD);
            } else robot.setLiftRight(OFF);
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
        telemetry(v);

    }

    private void telemetry(double[] v) {
        telemetry.addData("Y/X: ", "y: %f, x: %f", gamepad1.left_stick_y, gamepad1.left_stick_x);
        telemetry.addData("F/S: ", "f: %f, s: %f", v[0], v[1]);
        telemetry.addData("V/T: ", "v: %f, t: %f", v[2], v[3]);
        telemetry.addData("Individual: ", "v1: %f, v2: %f, v3: %f, v4: %f", v[4], v[5], v[6], v[7]);
    }

}
