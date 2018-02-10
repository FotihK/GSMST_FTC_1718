package org.firstinspires.ftc.team3819.Structural;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import java.util.Timer;

/**
 * Created by 200462069 on 9/12/2017.
 */

public class RobotHardware {

    private HardwareMap map = null;

    public DcMotor  fL = null, fR = null, bL = null, bR = null, flipR = null, flipL = null;                //Drive Motors
    private DcMotor  rLift = null, lLift = null;
    private Servo    jewel = null, rWheel = null, lWheel = null;

    private static final int       CPR = 1120;                                 //encoder counts per revolution
    private static final double    DIAMETER = 4;                               //encoded drive wheel diameter (in)
    private static final double    GEARING = 1;
    public static final double     CPI = (CPR * GEARING) / (DIAMETER * 3.14);
    public static final double     CPF = CPI * 12;
    public static int               position = 0;


    private final double    JEWEL_UP = 0;
    private final double    JEWEL_DOWN = .55;
    private final double    JEWEL_MID = .25;
    private final double DRIVE_SCALE = 1;
    private final int wheelRun = 0;

    public RobotHardware(HardwareMap map){
        this.map = map;
    }

    public void init(){
        fL.setDirection(DcMotorSimple.Direction.FORWARD);
        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.FORWARD);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);

        rLift.setDirection(DcMotorSimple.Direction.REVERSE);
        lLift.setDirection(DcMotorSimple.Direction.FORWARD);
        flipR.setDirection(DcMotorSimple.Direction.REVERSE);
        flipL.setDirection(DcMotorSimple.Direction.FORWARD);

        fL = map.get(DcMotor.class, "fL");
        fR = map.get(DcMotor.class, "fR");
        bL = map.get(DcMotor.class, "bL");
        bR = map.get(DcMotor.class, "bR");
        rLift = map.get(DcMotor.class, "rLift");
        lLift = map.get(DcMotor.class, "lLift");
        flipR = map.get(DcMotor.class, "flipR");
        flipL = map.get(DcMotor.class, "flipL");

        jewel = map.get(Servo.class, "jewel");
        rWheel = map.get(Servo.class, "rWheel");
        lWheel = map.get(Servo.class, "lWheel");

        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        resetEnc();
    }

    public void levelZero() {
        lLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lLift.setTargetPosition(0);
        rLift.setTargetPosition(0);

        lLift.setPower(-.2);
        rLift.setPower(-.2);

        position = 0;
    }

    public void levelOne() {
        lLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lLift.setTargetPosition(360);
        rLift.setTargetPosition(360);
        if(position>1) {
            lLift.setPower(-.2);
            rLift.setPower(-.2);
        }
        else {
            lLift.setPower(.2);
            rLift.setPower(.2);
        }


        position = 1;
    }

    public void levelTwo() {
        lLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lLift.setTargetPosition(720);
        rLift.setTargetPosition(720);
        if(position>2) {
            lLift.setPower(-.2);
            rLift.setPower(-.2);
        }
        else {
            lLift.setPower(.2);
            rLift.setPower(.2);
        }


        position = 2;
    }

    public void levelThree() {
        lLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lLift.setTargetPosition(1080);
        rLift.setTargetPosition(1080);
        if(position>3) {
            lLift.setPower(-.2);
            rLift.setPower(-.2);
        }
        else {
            lLift.setPower(.2);
            rLift.setPower(.2);
        }


        position = 3;
    }

    public void intake(){
            rWheel.setPosition(.8);
            lWheel.setPosition(-.8);
    }

    public void stopIntake(){
        rWheel.setPosition(0);
        lWheel.setPosition(0);
    }

    public void flip(double power) {
        power = Range.clip(power, -1, 1);
        flipR.setPower(power/2);
        flipL.setPower(power/2);
    }

    public void driveDistIn(double in, double p) {  // send dist and speed
        double counts = CPI * in;
        fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fR.setTargetPosition(counts);
        fL.setTargetPosition(counts);
        while(fR.getCurrentPosition()>= counts || fL.getCurrentPosition() >= counts) {
            fR.setPower(p);
            fL.setPower(p);
            bL.setPower(p);
            bR.setPower(p);
        }
        stop();
    }

    public void driveDistFt(double ft, double p) {  // send dist and speed
        double counts = CPF * ft;
        fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fR.setTargetPosition(counts);
        fL.setTargetPosition(counts);
        while(fR.getCurrentPosition()>= counts || fL.getCurrentPosition() >= counts) {
            fR.setPower(p);
            fL.setPower(p);
            bL.setPower(p);
            bR.setPower(p);
        }
        stop();
    }

    public void driveLeft(double power) {
        power = Range.clip(power, -1, 1);
            fL.setPower(power);
    }

    public void drive (double power) {
        power = Range.clip(power, -1, 1);
        fL.setPower(power);
        fR.setPower(power);
        bL.setPower(power);
        bR.setPower(power);
    }

    public void driveRight(double power){
        power = Range.clip(power, -1, 1);
        fR.setPower(power);
    }

public double[] drive(Gamepad gp) {
        double leftY = -gp.left_stick_y;
        double leftX = gp.left_stick_x;
        double rightX = gp.right_stick_x;
        leftY = Math.abs(leftY) > 0.03 ? leftY : 0;
        leftX = Math.abs(leftX) > 0.03 ? leftX : 0;
        rightX = Math.abs(rightX) > 0.03 ? rightX : 0;
        double a = 5.0 / 8;

        return drive(scale(leftY, a), scale(leftX, a), scale(rightX, a));
    }

    public double[] drive(double forward, double strafe, double turn) {
        double f = -Range.clip(forward, -1, 1);
        double s = Range.clip(strafe, -1, 1);
        double t = Range.clip(turn, -1, 1);

        double hyp = Math.hypot(f, s);
        double theta = Math.atan2(f, s) - Math.PI / 4;
        double v[] = new double[4];

        v[0] = (hyp * Math.cos(theta)) + t;
        v[1] = (hyp * Math.cos(theta)) - t;
        v[2] = (hyp * Math.sin(theta)) + t;
        v[3] = (hyp * Math.sin(theta)) - t;

        v = normalize(v);

        fR.setPower(v[0] * DRIVE_SCALE);
        fL.setPower(v[1] * DRIVE_SCALE);
        bR.setPower(v[2] * DRIVE_SCALE);
        bL.setPower(v[3] * DRIVE_SCALE);

        return new double[]{f, s, hyp, theta * 180 / Math.PI, v[0], v[1], v[2], v[3]};
    }

    private double[] normalize(double[] values) {
        double max = Math.abs(values[0]);
        for (double val : values) if (Math.abs(val) > max) max = Math.abs(val);
        if(max >= 1) {
            for (int i = 0; i < values.length; i++) {
                values[i] = values[i] / max;
            }
        }
        return values;
    }

    private double scale(double x, double a) {
        return x * ((a * x * x) - a + 1);
    }

    public void stop() {
        drive(0);
    }

    public void turn(double power){     //positive for turn left, negative for turn right
        driveLeft(-power);
        driveRight(power);
    }

    public boolean driveIsBusy(){
        return fL.isBusy() || fR.isBusy();
    }

    public void resetEnc(){
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void jewelDown(){
        jewel.setPosition(JEWEL_DOWN);
    }

    public void jewelUp(){
        jewel.setPosition(JEWEL_UP);
    }

    public void jewelMid() { jewel.setPosition(JEWEL_MID); }

}
