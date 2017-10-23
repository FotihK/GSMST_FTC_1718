package org.firstinspires.ftc.team2981.structural;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by 200462069 on 9/12/2017.
 */

public class RobotHardware {

    private HardwareMap map = null;

    public DcMotor  fL = null, fR = null;                          //Drive Motors
    private DcMotor  fourBarLeft = null, fourBarRight = null;                                                      //4-Bar motor
    private Servo    claw = null, jewel = null;

    private static final int       CPR = 1120;                                                          //encoder counts per revolution
    private static final double    DIAMETER = 4;                                                  //encoded drive wheel circumference
    private static final double    GEARING = 2;
    public static final double    CPI = (CPR * GEARING) / (DIAMETER * 3.14);

    private final double    CLAW_OPEN_VAL = 0;
    private final double    CLAW_CLOSED_VAL = 0.65;
    private final double    CLAW_CLOSED_MORE_VAL = 0.8;
    private final double    JEWEL_UP = 0.12;
    private final double    JEWEL_DOWN = 0.65;

    private boolean clawClosed = false;

    public RobotHardware(HardwareMap map){
        this.map = map;
    }

    public void init(){
        fL = map.get(DcMotor.class, "fL");
        fR = map.get(DcMotor.class, "fR");

        fourBarLeft = map.get(DcMotor.class, "fourBarLeft");
        fourBarRight = map.get(DcMotor.class, "fourBarRight");

        claw = map.get(Servo.class, "claw");
        jewel = map.get(Servo.class, "jewel");

        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        fL.setDirection(DcMotorSimple.Direction.FORWARD);

        fourBarLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fourBarRight.setDirection(DcMotorSimple.Direction.FORWARD);

        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fourBarLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fourBarRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        closeClaw();
        jewelUp();

        resetEnc();
    }

    public void driveLeft(double power){
        power = Range.clip(power, -1, 1);
        fL.setPower(power);
    }

    public void driveRight(double power){
        power = Range.clip(power, -1, 1);
        fR.setPower(power);
    }

    public void drive(double power){
        driveLeft(power);
        driveRight(power);
    }

    public void fourBar(double power){
        power = Range.clip(power, -1, 1);
        fourBarLeft.setPower(power);
        fourBarRight.setPower(power);
    }

    public void stop(){
        drive(0);
    }

    public void resetEnc(){
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fourBarLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fourBarRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void driveDist(double dist, double power){
        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int counts = (int) (dist * CPI);
        fL.setTargetPosition(counts);
        fR.setTargetPosition(counts);

        drive(power);
        resetEnc();
    }

    public void turn(double power){     //positive for turn left, negative for turn right
        driveLeft(-power);
        driveRight(power);
    }

    public boolean driveIsBusy(){
        return fL.isBusy() || fR.isBusy();
    }

    public boolean fourBarIsBusy(){
        return fourBarRight.isBusy() || fourBarLeft.isBusy();
    }

    public void closeClaw(){
        claw.setPosition(CLAW_CLOSED_VAL);
        clawClosed = true;
    }

    public void closeClawMore(){
        claw.setPosition(CLAW_CLOSED_MORE_VAL);
        clawClosed = true;
    }

    public void openClaw(){
        claw.setPosition(CLAW_OPEN_VAL);
        clawClosed = false;
    }

    public boolean isClawCloser(){
        return clawClosed;
    }

    public void jewelDown(){
        jewel.setPosition(JEWEL_DOWN);
    }

    public void jewelUp(){
        jewel.setPosition(JEWEL_UP);
    }


}
