package org.firstinspires.ftc.team3819.Structural;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Timer;

/**
 * Created by 200462069 on 9/12/2017.
 */

public class RobotHardware {

    private HardwareMap map = null;

    public DcMotor  fL = null, fR = null, bL = null, bR = null, flipR = null, flipL = null;                //Drive Motors
    private DcMotor  rLift = null, lLift = null;
    private Servo    jewel = null;
    private CRServo  rWheel = null, lWheel = null;
    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    VuforiaLocalizer.Parameters parameters;

    private static final int       CPR = 1120;                                 //encoder counts per revolution
    private static final double    DIAMETER = 4;                               //encoded drive wheel diameter (in)
    private static final double    GEARING = 1;
    public static final double     CPI = (CPR * GEARING) / (DIAMETER * 3.14);
    public static final double     CPF = CPI * 12;
    public static double           position = -.5;



    private final double    JEWEL_UP = 1;
    private final double    JEWEL_DOWN = .35;
    private final double    JEWEL_MID = .8;
    private final double DRIVE_SCALE = 1;
    private final int wheelRun = 0;

    public RobotHardware(HardwareMap map){
        this.map = map;
    }

    public void init(){
        fL = map.get(DcMotor.class, "fL");
        fR = map.get(DcMotor.class, "fR");
        bL = map.get(DcMotor.class, "bL");
        bR = map.get(DcMotor.class, "bR");
        rLift = map.get(DcMotor.class, "rLift");
        lLift = map.get(DcMotor.class, "lLift");
        flipR = map.get(DcMotor.class, "flipR");
        flipL = map.get(DcMotor.class, "flipL");

        jewel = map.get(Servo.class, "jewel");

        rWheel = map.get(CRServo.class, "rWheel");
        lWheel = map.get(CRServo.class, "lWheel");

        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        fR.setDirection(DcMotorSimple.Direction.FORWARD);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.FORWARD);

        rLift.setDirection(DcMotorSimple.Direction.REVERSE);
        lLift.setDirection(DcMotorSimple.Direction.FORWARD);
        flipR.setDirection(DcMotorSimple.Direction.REVERSE);
        flipL.setDirection(DcMotorSimple.Direction.FORWARD);

        int cameraMonitorViewId = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", map.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AaDxUz3/////AAAAGSJYyeGWDU3Im6rx85GbYy5gq8Raja7mkDAmDCUs6BvNAusEF4omrCaZUFeEG7gyW/Sq1exxlBmowJ4IY2ICrleyyxb1XJaFw0IsYhuBzESI/duL9SW2gVXcULoqBd7q4wniSHWZNNlkMYiuSbaW6z7299VJzV0QEi0HugnY+5PhZHUts9CU+lGIukrkAIDWP5bXOEmERBRpl4XKWIviWeCGHiVQVwAjeBEPnX1fsqRf+178gAoXEXDanp9cHriUGyU4a0vqhvJyb2LoQG5NrNLoFGUMU45pTWdjjY8TuVv9sfYSVwcboP2vzFeh8TVBbQRJrNrdWiRbw35nn+JSrZY6ulR5ZSDTq7l1apzxTy/s\n";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        relicTrackables.activate();

    }

    public void initTele(){
        fL = map.get(DcMotor.class, "fL");
        fR = map.get(DcMotor.class, "fR");
        bL = map.get(DcMotor.class, "bL");
        bR = map.get(DcMotor.class, "bR");
        rLift = map.get(DcMotor.class, "rLift");
        lLift = map.get(DcMotor.class, "lLift");
        flipR = map.get(DcMotor.class, "flipR");
        flipL = map.get(DcMotor.class, "flipL");

        jewel = map.get(Servo.class, "jewel");

        rWheel = map.get(CRServo.class, "rWheel");
        lWheel = map.get(CRServo.class, "lWheel");

        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        fR.setDirection(DcMotorSimple.Direction.FORWARD);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.FORWARD);

        rLift.setDirection(DcMotorSimple.Direction.REVERSE);
        lLift.setDirection(DcMotorSimple.Direction.FORWARD);
        flipR.setDirection(DcMotorSimple.Direction.REVERSE);
        flipL.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public String vuphoria() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        return vuMark.toString();
    }

    public int[] getEncoders() {
        int[] a = new int[2];
        a[0] = fR.getCurrentPosition();
        a[1] = fL.getCurrentPosition();
        return a;
    }

    public void testLift(double power) {
        rLift.setPower(power);
        lLift.setPower(power);
    }

    public void levelZero() {
        lLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lLift.setTargetPosition(0);
        rLift.setTargetPosition(0);

        lLift.setPower(-.6);
        rLift.setPower(-.6);

        position = 0;
    }

    public void levelOne() {
        lLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lLift.setTargetPosition(360);
        rLift.setTargetPosition(360);

        if(position>1) {
            lLift.setPower(-.6);
            rLift.setPower(-.6);
        }
        else {
            lLift.setPower(.6);
            rLift.setPower(.6);
        }


        position = 1;
    }

    public void levelTwo() {
        lLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lLift.setTargetPosition(720);
        rLift.setTargetPosition(720);

        if(position>2) {
            lLift.setPower(-.6);
            rLift.setPower(-.6);
        }
        else {
            lLift.setPower(.6);
            rLift.setPower(.6);
        }


        position = 2;
    }

    public void levelThree() {
        lLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lLift.setTargetPosition(1080);
        rLift.setTargetPosition(1080);

        if(position>3) {
            lLift.setPower(-.6);
            rLift.setPower(-.6);
        }
        else {
            lLift.setPower(.6);
            rLift.setPower(.6);
        }


        position = 3;
    }

    public void intake(){
            rWheel.setPower(-1);
            lWheel.setPower(1);
    }

    public void outtake() {
        rWheel.setPower(1);
        lWheel.setPower(-1);
    }

    public void stopIntake(){
        rWheel.setPower(0);
        lWheel.setPower(0);
    }

    public void flip(double power) {
        power = Range.clip(power, -1, 1);
        if(power<0) {
            flipR.setPower(power / 2);
            flipL.setPower(power / 2);
        }
        else {
            flipR.setPower(power / 6);
            flipL.setPower(power / 6);
        }
    }

    public void driveDistIn(double in, double p) {  // send dist and speed
        resetEnc();

        int counts = (int) (CPI * in);
        fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fR.setTargetPosition(counts);
        fL.setTargetPosition(counts);

        if (counts < 0)
            drive(-p);
        else
            drive(p);
    }

    public boolean isBusy() {
        return fL.isBusy() || fR.isBusy();
    }

    public boolean liftIsBusy() {
        return lLift.isBusy() || rLift.isBusy();
    }

    public void right() {
        resetEnc();
        int counts = (int)(CPI * 16);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setTargetPosition(-counts);
        fL.setTargetPosition(counts);
        drive(0,0,.5);
    }

    public void left() {
        resetEnc();
        int counts = (int)(CPI * 16);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setTargetPosition(counts);
        fL.setTargetPosition(-counts);
        drive(0,0,-.5);
    }

    public void noEncode() {
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void encode() {
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void spin(double power) {
        fR.setPower(-power);
        bR.setPower(-power);
        fL.setPower(power);
        bL.setPower(power);
    }

    public double[] drive(Gamepad gp) {
        double leftY = -gp.left_stick_y;
        double leftX = gp.left_stick_x;
        double rightX = -gp.right_stick_x;
        leftY = Math.abs(leftY) > 0.03 ? leftY : 0;
        leftX = Math.abs(leftX) > 0.03 ? leftX : 0;
        rightX = Math.abs(rightX) > 0.03 ? rightX : 0;
        double a = 5.0 / 8;

        return drive(scale(leftY, a), scale(leftX, a), scale(rightX, a));
    }

    public void drive(double power) {
        fR.setPower(power);
        fL.setPower(power);
        bR.setPower(power);
        bL.setPower(power);
    }

    public double[] drive(double forward, double strafe, double turn) {
        double f = Range.clip(forward, -1, 1);
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

        bR.setPower(v[0] * DRIVE_SCALE);
        fL.setPower(v[1] * DRIVE_SCALE);
        fR.setPower(v[2] * DRIVE_SCALE);
        bL.setPower(v[3] * DRIVE_SCALE);

        return new double[]{f, s, hyp, theta * 180 / Math.PI, v[0], v[1], v[2], v[3]};
    }

    public void drive(double a, double b) {
        bL.setPower(a);
        fL.setPower(a);
        bR.setPower(b);
        fR.setPower(b);
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
        fR.setPower(0);
        fL.setPower(0);
        bR.setPower(0);
        bL.setPower(0);
    }

    public void resetEnc(){
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void jewelDown(){
        jewel.setPosition(JEWEL_DOWN);
    }

    public void jewelUp(){
        jewel.setPosition(JEWEL_UP);
    }

    public void jewelMid() { jewel.setPosition(JEWEL_MID); }

}
