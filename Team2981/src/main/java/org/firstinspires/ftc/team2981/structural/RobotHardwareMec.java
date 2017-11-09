package org.firstinspires.ftc.team2981.structural;

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
    private DcMotor intakeLeft = null, intakeRight = null;
    private DcMotor conveyor = null;
    private DcMotor lift = null;
    private Servo jewel = null;

    private SystemState intakeState = SystemState.OFF;
    private SystemState conveyorState = SystemState.OFF;
    private SystemState liftState = SystemState.OFF;

    private final double CONVEYOR_POWER = 0.4;
    private final double INTAKE_POWER = 0.4;
    private final double LIFT_POWER = 0.4;
    private final double JEWEL_UP = 0.12;
    private final double JEWEL_DOWN = 0.65;

    public RobotHardwareMec(HardwareMap hwMap){
        map = hwMap;
    }

    public void init(){
        fL = map.get(DcMotor.class, "frontLeft");
        fR = map.get(DcMotor.class, "frontRight");
        bL = map.get(DcMotor.class, "backLeft");
        bR = map.get(DcMotor.class, "backRight");

        intakeLeft = map.get(DcMotor.class, "intakeLeft");
        intakeRight = map.get(DcMotor.class, "intakeRight");

        conveyor = map.get(DcMotor.class, "conveyor");

        lift = map.get(DcMotor.class, "lift");

        jewel = map.get(Servo.class, "jewel");

        fR.setDirection(DcMotorSimple.Direction.FORWARD);
        bR.setDirection(DcMotorSimple.Direction.FORWARD);
        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);

        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeRight.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        conveyor.setDirection(DcMotorSimple.Direction.FORWARD);

        lift.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void drive(Gamepad gp){
        double leftY = gp.left_stick_y;
        double leftX = gp.left_stick_x;
        double rightX = gp.right_stick_x;
        leftY = Math.abs(leftY) > 0.05 ? leftY : 0;
        leftX = Math.abs(leftX) > 0.05 ? leftX : 0;
        rightX = Math.abs(rightX) > 0.05 ? rightX : 0;

        drive(leftY, leftX, rightX);
    }

    public void drive(double forward, double strafe, double turn){
        forward = Range.clip(forward, -1, 1);
        strafe = Range.clip(strafe, -1, 1);
        turn = Range.clip(turn, -1, 1);

        double v = Math.hypot(strafe, forward);
        double theta = Math.atan2(forward, strafe) - Math.PI/4;
        final double v1 = v * Math.cos(theta) + turn;
        final double v2 = v * Math.sin(theta) - turn;
        final double v3 = v * Math.sin(theta) + turn;
        final double v4 = v * Math.cos(theta) - turn;

        final double vmax = Math.max(Math.max(v1, v2), Math.max(v3,v4));

        fL.setPower(v1/vmax);
        fR.setPower(v2/vmax);
        bL.setPower(v3/vmax);
        bR.setPower(v4/vmax);
    }

    public SystemState getIntakeState() {
        return intakeState;
    }

    public SystemState getConveyorState() {
        return conveyorState;
    }

    public SystemState getLiftState() {
        return liftState;
    }

    public void setConveyor(SystemState st){
        conveyorState = st;
        switch(st){
            case OFF:
                conveyor.setPower(0);
                break;
            case FORWARD:
                conveyor.setPower(CONVEYOR_POWER);
                break;
            case BACKWARD:
                conveyor.setPower(-CONVEYOR_POWER);
                break;
        }
    }

    public void setIntake(SystemState st){
        intakeState = st;
        switch(st){
            case OFF:
                intakeLeft.setPower(0);
                intakeRight.setPower(0);
                break;
            case FORWARD:
                intakeLeft.setPower(INTAKE_POWER);
                intakeRight.setPower(INTAKE_POWER);
                break;
            case BACKWARD:
                intakeLeft.setPower(-INTAKE_POWER);
                intakeRight.setPower(-INTAKE_POWER);
                break;
        }
    }

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

    public void jewelDown(){
        jewel.setPosition(JEWEL_DOWN);
    }

    public void jewelUp(){
        jewel.setPosition(JEWEL_UP);
    }

}
