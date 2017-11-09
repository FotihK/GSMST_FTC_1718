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
    private Vision vision;

    private Thread vuf = new Thread(new Runnable() {
        @Override
        public void run() {
            vision = new Vision(hardwareMap);
            vision.start(1);
        }
    });

    @Override
    public void init() {
        robot = new RobotHardwareMec(hardwareMap);
        sensors = new Sensors(hardwareMap);

        robot.init();
        sensors.calibrate();
        vuf.start();
    }

    @Override
    public void loop(){
        robot.drive(gamepad1);

        if(gamepad1.left_bumper){
            if(robot.getIntakeState() != FORWARD) robot.setIntake(FORWARD);
            else robot.setIntake(OFF);
        } else if(gamepad1.right_bumper){
            if(robot.getIntakeState() != BACKWARD) robot.setIntake(BACKWARD);
            else robot.setIntake(OFF);
        }

        if(gamepad2.left_bumper){
            if(robot.getConveyorState() != FORWARD) robot.setConveyor(FORWARD);
            else robot.setConveyor(OFF);
        } else if(gamepad2.right_bumper){
            if(robot.getConveyorState() != BACKWARD) robot.setConveyor(BACKWARD);
            else robot.setConveyor(OFF);
        }

        if(gamepad2.dpad_up){
            if(robot.getLiftState() != FORWARD) robot.setLift(FORWARD);
            else robot.setLift(OFF);
        } else if(gamepad2.dpad_down){
            if(robot.getLiftState() != BACKWARD) robot.setLift(BACKWARD);
            else robot.setLift(OFF);
        }
    }
/*
    private void driverOne(){
        double leftStick = gamepad1.left_stick_y;
        double rightStick = gamepad1.right_stick_y;
        leftStick = (Math.abs(leftStick) > 0.05 ? (leftStick)/.85 : 0);
        rightStick = (Math.abs(rightStick) > 0.05 ? (rightStick + (0.05*Math.signum(rightStick))/.85) : 0);

        robot.driveLeft(leftStick);
        robot.driveRight(rightStick);
    }

    private void driverTwo(){
        double power = (gamepad2.dpad_up ? 0.65 : (gamepad2.dpad_down ? -0.30 : 0));
        robot.fourBar(power);

        if(gamepad2.x) robot.closeClaw();
        else if(gamepad2.y) robot.openClaw();
        else if(gamepad2.a) robot.closeClawMore();

        if(gamepad2.left_bumper) robot.jewelDown();
        else if(gamepad2.right_bumper) robot.jewelUp();

    }

    private void telemetry(){

        telemetry.addData("VuMark ", "%s visible", vision.trackMark());
        telemetry.addData("Claw ", robot.isClawCloser() ? "is closed." : "is open.");
        OpenGLMatrix loc = vision.trackLocation();
        float[] trans = vision.getTrans();
        float[] rot = vision.getRot();
        telemetry.addData("Position: ", loc != null ? loc.formatAsTransform() : "Unknown");
        if(trans != null) {
            telemetry.addData("Translation: ", "(%f, %f, %f)", trans[0], trans[1], trans[2]);
        } else telemetry.addData("Translation: ", "Unknown");
        if (rot != null) {
            telemetry.addData("Rotation: ", "(%f, %f, %f)", rot[0], rot[1], rot[2]);
        } else telemetry.addData("Rotation: ", "Unknown");

        telemetry.addData("RGB: ", "(%d, %d, %d)", sensors.getRGB()[0], sensors.getRGB()[1], sensors.getRGB()[2]);
    }

    @Override
    public void loop() {
        driverOne();
        driverTwo();
        //telemetry();
    }

    @Override
    public void stop(){
        //vision.stop();
    }
*/
}
