package org.firstinspires.ftc.team2981;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team2981.structural.RobotHardware;
import org.firstinspires.ftc.team2981.structural.Sensors;
import org.firstinspires.ftc.team2981.structural.Vision;

/**
 * Created by 200462069 on 10/18/2017.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutoNoNo",group="Auto")
public class AutoNoColorNoWait extends LinearOpMode {

    private RobotHardware robot;
    private Sensors sensors;
    private Vision vision;

    static final double DRIVE_SPEED = 0.7;
    static final double TURN_SPEED = 0.5;
    static final double HEADING_THRESH = 1;
    static final double P_TURN_COEFF = 0.1;
    static final double P_DRIVE_COEFF = 0.15;
    private boolean blueTeam;
    private ElapsedTime meaner = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime runtime = new ElapsedTime();

    public void initialize(){
        robot = new RobotHardware(hardwareMap);
        sensors = new Sensors(hardwareMap);
        vision = new Vision(hardwareMap);

        robot.init();
        sensors.init();
        vision.start(1);

        /*
        sensors.calibrate();
        while(!isStopRequested() && sensors.isGyroCalibrating()){
            sleep(50);
            idle();
        }

        */

        while(!isStarted()){
            if(gamepad1.dpad_up) {
                vision.switchLocation(1);
                blueTeam = true;
            }
            else if(gamepad1.dpad_down) {
                vision.switchLocation(2);
                blueTeam = true;
            }
            else if(gamepad1.dpad_left) {
                vision.switchLocation(3);
                blueTeam = false;
            }
            else if(gamepad1.dpad_down) {
                vision.switchLocation(4);
                blueTeam = false;
            }
            telemetry.addData("Team ", blueTeam ? "Blue" : "Red");
            telemetry.addData("Location: ", vision.getLoc());
            telemetry.update();
        }

        robot.resetEnc();
        telemetry.clearAll();
        telemetry.update();
        sensors.resetGyro();
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        //sleep(10000);
        robot.drive(-0.3);
        sleep(350);
        robot.stop();
        robot.jewelDown();
        meaner.reset();
        int blue = 1, red = 0, count = 0;
        /*
        while(meaner.milliseconds() < 1000){
            blue += sensors.getRGB()[2];
            red += sensors.getRGB()[0];
            count++;
            sleep(50);
        }

        blue /= count;
        red /= count;
        */
        sleep(1500);
        boolean left = false;
        if(blueTeam){
            if(blue > red) {
                encoderDrive(0.5, -5, 5, 4);
                left = true;
            }
            else {
                encoderDrive(0.5, 5, -5, 4);
                left = false;
            }
        } else {
            if(red > blue) {
                encoderDrive(0.5, -5, 5, 4);
                left = true;
            }
            else {
                encoderDrive(0.5, 5, -5, 4);
                left = false;
            }
        }

        if(vision.getLoc() == 1){
            if(left) {
                encoderDrive(0.5, -5, 5, 4);
                encoderDrive(0.7, 5, 5 ,5);
            }
            else {
                encoderDrive(0.5, 5, -5, 4);
                encoderDrive(0.7, -5, -5 ,5);
            }
        } else if(vision.getLoc() == 2){
            if(left) {
                encoderDrive(0.5, -5, 5, 4);
                encoderDrive(0.7, 5, 5 ,5);
            }
            else {
                encoderDrive(0.5, 5, -5, 4);
                encoderDrive(0.7, -5, -5 ,5);
            }
        } else if(vision.getLoc() == 3){
            if(left) {
                encoderDrive(0.5, -5, 5, 4);
                encoderDrive(0.7, -5, -5 ,5);
            }
            else {
                encoderDrive(0.5, 5, -5, 4);
                encoderDrive(0.7, 5, 5 ,5);
            }
        } else
            if(left) {
                encoderDrive(0.5, -5, 5, 4);
                encoderDrive(0.7, -5, -5 ,5);
            }
            else {
                encoderDrive(0.5, 5, -5, 4);
                encoderDrive(0.7, 5, 5 ,5);
            }
        }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.fL.getCurrentPosition() + (int)(leftInches * RobotHardware.CPI);
            newRightTarget = robot.fR.getCurrentPosition() + (int)(rightInches * RobotHardware.CPI);
            robot.fL.setTargetPosition(newLeftTarget);
            robot.fR.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.fL.setPower(Math.abs(speed));
            robot.fR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.fL.isBusy() || robot.fR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.fL.getCurrentPosition(),
                        robot.fR.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.fL.setPower(0);
            robot.fR.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

              sleep(500);   // optional pause after each move
        }
    }
}
