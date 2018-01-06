package org.firstinspires.ftc.team2993.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;



public class RobotHardware
{
    //
    // Configuration variables
    //

    public final double INTAKE_SPEED = .25d;

    public final double CLAW_OPEN_POSITION    = 0d;
    public final double CLAW_CLOSED_POSITION  = .1d;

    public final double SIDEARM_UP_POSITION   = 0d;
    public final double SIDEARM_DOWN_POSITION = .5d;



    //
    // Hardware variables
    //

    private HardwareMap Map;

    public DcMotor fL, fR, bL, bR;
    public DcMotor liftL, liftR;
    public DcMotor arm;
    public CRServo intakeL, intakeR;
    public Servo   clawL, clawR;
    public Servo   sideArm;

    public Sensors color;

    public RobotHardware(HardwareMap map)
    {
        Map = map;
        Initialize();
    }

    public void Initialize()
    {
        fL    = Map.get(DcMotor.class, "fl");
        fR    = Map.get(DcMotor.class, "fr");
        bL    = Map.get(DcMotor.class, "bl");
        bR    = Map.get(DcMotor.class, "br");
        arm   = Map.get(DcMotor.class, "arm");
        liftL = Map.get(DcMotor.class, "liftl");
        liftR = Map.get(DcMotor.class, "liftr");

        fL.setDirection(DcMotorSimple.Direction.FORWARD);
        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.FORWARD);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        liftL.setDirection(DcMotorSimple.Direction.FORWARD);
        liftR.setDirection(DcMotorSimple.Direction.REVERSE);

        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        intakeL = Map.get(CRServo.class, "intakel");
        intakeR = Map.get(CRServo.class, "intaker");
        clawL   = Map.get(Servo.class, "clawl");
        clawR   = Map.get(Servo.class, "clawr");
        sideArm = Map.get(Servo.class, "sidearm");

        clawL.scaleRange(CLAW_OPEN_POSITION, CLAW_CLOSED_POSITION);
        clawR.scaleRange(CLAW_OPEN_POSITION, CLAW_CLOSED_POSITION);
        sideArm.scaleRange(SIDEARM_UP_POSITION, SIDEARM_DOWN_POSITION);
        sideArm.setPosition(0);
    }



    public void SetMotors(double speed)
    {
        SetMotors(speed,speed,speed,speed);
    }

    public void SetMotors(double l, double r)
    {
        SetMotors(l,r,l,r);
    }

    public void SetMotors(double fl, double fr, double bl, double br)
    {
        fl = Range.clip(fl, -1d, 1d);
        fr = Range.clip(fr, -1d, 1d);
        bl = Range.clip(bl, -1d, 1d);
        br = Range.clip(br, -1d, 1d);

        fL.setPower(fl);
        fR.setPower(fr);
        bL.setPower(bl);
        bR.setPower(br);
    }

    public void SetArm(double speed)
    {
        speed = Range.clip(speed, -1d, 1d);

        arm.setPower(speed);
    }

    public void SetLift(double speed)
    {
        speed = Range.clip(speed, -1d, 1d);

        liftL.setPower(speed);
        liftR.setPower(speed);
    }

    public void SetClaw(boolean clawClosed)
    {
        if (clawClosed)
            SetClaw(1d);
        else
            SetClaw(0d);
    }

    public void SetClaw(double position)
    {
        clawL.setPosition(position);
        clawR.setPosition(-position);
    }

    public void SetIntake(boolean intakeOn)
    {
        if (intakeOn)
        {
            intakeL.setPower(INTAKE_SPEED);
            intakeR.setPower(INTAKE_SPEED);
        }
        else
        {
            intakeL.setPower(0d);
            intakeR.setPower(0d);
        }
    }
}