package org.firstinspires.ftc.team2981.structural;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


/**
 * Created by 200462069 on 11/8/2017.
 */

public class RobotHardwareMec {

    public enum SystemState {
        FORWARD, BACKWARD, OFF
    }

    private HardwareMap map = null;
    private DcMotor fL = null, fR = null, bL = null, bR = null;
    private CRServo intakeLeft = null, intakeRight = null;
    private DcMotor conveyorLeft = null, conveyorRight = null;
    //private DcMotor lift = null;
    private Servo jewel = null;

    private SystemState intakeLeftState = SystemState.OFF;
    private SystemState intakeRightState = SystemState.OFF;
    private SystemState conveyorLeftState = SystemState.OFF;
    private SystemState conveyorRightState = SystemState.OFF;
    //private SystemState liftState = SystemState.OFF;

    private final double CONVEYOR_POWER = 1;
    private final double INTAKE_POWER = 0.5;
    private final double LIFT_POWER = 0.4;
    private final double JEWEL_UP = 1;
    private final double JEWEL_DOWN = 0.49;

    public RobotHardwareMec(HardwareMap hwMap){
        map = hwMap;
    }

    public void init(){
        fL = map.get(DcMotor.class, "frontLeft");
        fR = map.get(DcMotor.class, "frontRight");
        bL = map.get(DcMotor.class, "backLeft");
        bR = map.get(DcMotor.class, "backRight");

        intakeLeft = map.get(CRServo.class, "intakeLeft");
        intakeRight = map.get(CRServo.class, "intakeRight");

        conveyorLeft = map.get(DcMotor.class, "conveyorLeft");
        conveyorRight = map.get(DcMotor.class, "conveyorRight");

        //lift = map.get(DcMotor.class, "lift");

        jewel = map.get(Servo.class, "jewel");

        fR.setDirection(DcMotorSimple.Direction.FORWARD);
        bR.setDirection(DcMotorSimple.Direction.FORWARD);
        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */

        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeRight.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        conveyorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        conveyorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //lift.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public double[] drive(Gamepad gp){
        double leftY =  gp.left_stick_y;
        double leftX =  gp.left_stick_x;
        double rightX = gp.right_stick_x;
        leftY = Math.abs(leftY) > 0.03 ? leftY : 0;
        leftX = Math.abs(leftX) > 0.03 ? leftX : 0;
        rightX = Math.abs(rightX) > 0.03 ? rightX : 0;
        double a = 0.7;

        return drive(scale(leftY,a), scale(leftX,a), scale(rightX,a));
    }

    public double scale(double x, double a){
        return x*((a*x*x) - a + 1);
    }

    public double[] drive(double forward, double strafe, double turn){
        forward = Range.clip(forward, -1, 1);
        strafe = Range.clip(strafe, -1, 1);
        turn = Range.clip(turn, -1, 1);

        double v = Math.hypot(strafe, forward);
        double theta = Math.atan2(forward, strafe) - Math.PI/4;
        final double v1 = (v * Math.cos(theta)) + turn;
        final double v2 = (v * Math.cos(theta)) - turn;
        final double v3 = (v * Math.sin(theta)) + turn;
        final double v4 = (v * Math.sin(theta)) - turn;

        double vmax = Math.max(Math.max(Math.abs(v1), Math.abs(v2)), Math.max(Math.abs(v3), Math.abs(v4)));
        if(vmax == 0){
            fL.setPower(0);
            fR.setPower(0);
            bL.setPower(0);
            bR.setPower(0);
            vmax = 1;
        } else {
            fR.setPower(v1 * Math.abs(v1 / vmax) * 0.8);
            fL.setPower(v2 * Math.abs(v2 / vmax) * 0.8);
            bR.setPower(v3 * Math.abs(v3 / vmax) * 0.8);
            bL.setPower(v4 * Math.abs(v4 / vmax) * 0.8);

        }

        return new double[] {-forward, -strafe, v, theta*180/Math.PI, v1 * Math.abs(v1/vmax), v2 * Math.abs(v2/vmax), v3 * Math.abs(v3/vmax), Math.abs(v4 * v4/vmax)};
    }

    public void stop(){
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }

    public SystemState[] getIntakeState() {
        return new SystemState[] { intakeLeftState, intakeRightState };
    }

    public SystemState[] getConveyorState() {
        return new SystemState[] { conveyorLeftState, conveyorLeftState };
    }
    /*
    public SystemState getLiftState() {
        return liftState;
    }
    */

    public void setConveyorLeft(SystemState st){
        conveyorLeftState = st;
        switch(st){
            case OFF:
                conveyorLeft.setPower(0);
                break;
            case FORWARD:
                conveyorLeft.setPower(CONVEYOR_POWER);
                break;
            case BACKWARD:
                conveyorLeft.setPower(-CONVEYOR_POWER);
                break;
        }
    }

    public void setConveyorRight(SystemState st){
        conveyorRightState = st;
        switch(st){
            case OFF:
                conveyorRight.setPower(0);
                break;
            case FORWARD:
                conveyorRight.setPower(CONVEYOR_POWER);
                break;
            case BACKWARD:
                conveyorRight.setPower(-CONVEYOR_POWER);
                break;
        }
    }

    public void setIntakeLeft(SystemState st){
        intakeLeftState = st;
        switch(st){
            case OFF:
                intakeLeft.setPower(0);
                break;
            case FORWARD:
                intakeLeft.setPower(INTAKE_POWER);
                break;
            case BACKWARD:
                intakeLeft.setPower(-INTAKE_POWER);
                break;
        }
    }

    public void setIntakeRight(SystemState st){
        intakeRightState = st;
        switch(st){
            case OFF:
                intakeRight.setPower(0);
                break;
            case FORWARD:
                intakeRight.setPower(INTAKE_POWER);
                break;
            case BACKWARD:
                intakeRight.setPower(-INTAKE_POWER);
                break;
        }
    }

    /*
    public void setLift(SystemState st){
        liftState = st;
        switch(st){
            case OFF:
                lift.setPower(0);
                break;
            case FORWARD:
                lift.setPower(LIFT_POWER);
                break;
            case BACKWARD:
                lift.setPower(-LIFT_POWER);
                break;
        }
    }
    */

    public void jewelDown(){
        jewel.setPosition(JEWEL_DOWN);
    }

    public void jewelUp(){
        jewel.setPosition(JEWEL_UP);
    }

}
