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
    private DcMotor liftLeft = null;
    private DcMotor liftRight = null;
    private Servo jewel = null;

    private SystemState intakeLeftState = SystemState.OFF;
    private SystemState intakeRightState = SystemState.OFF;
    private SystemState conveyorLeftState = SystemState.OFF;
    private SystemState conveyorRightState = SystemState.OFF;
    private SystemState liftLeftState = SystemState.OFF;
    private SystemState liftRightState = SystemState.OFF;

    private final double CONVEYOR_POWER = 1;
    private final double INTAKE_POWER = 0.5;
    private final double LIFT_POWER = 0.7;
    private final double LIFT_RATIO = 0.95;     //right over left
    private final double JEWEL_UP = 0.8;
    private final double JEWEL_DOWN = 0.5;
    private final double DRIVE_SCALE = 0.9;

    public RobotHardwareMec(HardwareMap hwMap) {
        map = hwMap;
    }

    public void init() {
        fL = map.get(DcMotor.class, "frontLeft");
        fR = map.get(DcMotor.class, "frontRight");
        bL = map.get(DcMotor.class, "backLeft");
        bR = map.get(DcMotor.class, "backRight");

        intakeLeft = map.get(CRServo.class, "intakeLeft");
        intakeRight = map.get(CRServo.class, "intakeRight");

        conveyorLeft = map.get(DcMotor.class, "conveyorLeft");
        conveyorRight = map.get(DcMotor.class, "conveyorRight");

        liftLeft = map.get(DcMotor.class, "liftLeft");
        liftRight = map.get(DcMotor.class, "liftRight");

        jewel = map.get(Servo.class, "jewel");

        fR.setDirection(DcMotorSimple.Direction.FORWARD);
        bR.setDirection(DcMotorSimple.Direction.FORWARD);
        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);

        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeRight.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        conveyorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        conveyorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        conveyorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public double[] drive(Gamepad gp) {
        double leftY = -gp.left_stick_y;
        double leftX = gp.left_stick_x;
        double rightX = gp.right_stick_x;
        leftY = Math.abs(leftY) > 0.03 ? leftY : 0;
        leftX = Math.abs(leftX) > 0.03 ? leftX : 0;
        rightX = Math.abs(rightX) > 0.03 ? rightX : 0;
        double a = 2.0 / 3;

        return drive(scale(leftY, a), scale(leftX, a), scale(rightX, a));
    }

    private double scale(double x, double a) {
        return x * ((a * x * x) - a + 1);
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

    public void stop() {
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }

    public SystemState[] getIntakeState() {
        return new SystemState[]{intakeLeftState, intakeRightState};
    }

    public SystemState[] getConveyorState() {
        return new SystemState[]{conveyorLeftState, conveyorRightState};
    }

    public SystemState[] getLiftState() {
        return new SystemState[] {liftLeftState, liftRightState};
    }

    public void setConveyorLeft(SystemState st) {
        conveyorLeftState = st;
        switch (st) {
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

    public void setConveyorRight(SystemState st) {
        conveyorRightState = st;
        switch (st) {
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

    public void setIntakeLeft(SystemState st) {
        intakeLeftState = st;
        switch (st) {
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

    public void setIntakeRight(SystemState st) {
        intakeRightState = st;
        switch (st) {
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

    public void setLiftLeft(SystemState st) {
        liftLeftState = st;
        switch (st) {
            case OFF:
                liftLeft.setPower(0);
                break;
            case FORWARD:
                liftLeft.setPower(LIFT_POWER);
                break;
            case BACKWARD:
                liftLeft.setPower(-LIFT_POWER);
                break;
        }
    }

    public void setLiftRight(SystemState st) {
        liftRightState = st;
        switch (st) {
            case OFF:
                liftRight.setPower(0);
                break;
            case FORWARD:
                liftRight.setPower(LIFT_POWER);
                break;
            case BACKWARD:
                liftRight.setPower(-LIFT_POWER);
                break;
        }
    }

    public void jewelDown() {
        jewel.setPosition(JEWEL_DOWN);
    }

    public void jewelUp() {
        jewel.setPosition(JEWEL_UP);
    }

}
