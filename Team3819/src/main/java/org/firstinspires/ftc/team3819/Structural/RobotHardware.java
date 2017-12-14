package org.firstinspires.ftc.team3819.Structural;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import java.util.Timer;

/**
 * Created by 200462069 on 9/12/2017.
 */

public class RobotHardware {

    private HardwareMap map = null;

    public DcMotor  fL = null, fR = null;                          //Drive Motors
    private DcMotor  lift = null;
    private Servo    claw = null, jewel = null;

    private static final int       CPR = 1120;                                                          //encoder counts per revolution
    private static final double    DIAMETER = 4;                                                  //encoded drive wheel circumference
    private static final double    GEARING = 2;
    public static final double    CPI = (CPR * GEARING) / (DIAMETER * 3.14);

    private final double    CLAW_OPEN_VAL = 0;
    private final double    CLAW_CLOSED_VAL = 0.65;
    private final double    CLAW_CLOSED_MORE_VAL = 0.8;
    private final double    JEWEL_UP = 0.65;
    private final double    JEWEL_DOWN = 0.12;

    private boolean clawClosed = false;

    public RobotHardware(HardwareMap map){
        this.map = map;
    }

    public void init(){
        fL = map.get(DcMotor.class, "fL");
        fR = map.get(DcMotor.class, "fR");

        lift = map.get(DcMotor.class, "lift");

        claw = map.get(Servo.class, "claw");
        jewel = map.get(Servo.class, "jewel");

        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        fL.setDirection(DcMotorSimple.Direction.FORWARD);

        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        closeClaw();
        jewelUp();

    }

    public void lift(double power) {
        power = Range.clip(power, -1, 1);
        lift.setPower(power);
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
