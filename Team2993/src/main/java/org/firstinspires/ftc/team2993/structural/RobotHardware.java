package org.firstinspires.ftc.team2993.structural;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class RobotHardware
{
    private HardwareMap map;

    public DcMotor fR, fL, bR, bL;
    public DcMotor armR, armL;
    public DcMotor clawR, clawL;
    public Servo   sideArm;

    public Sensors color;

    public RobotHardware(HardwareMap _map)
    {
        map = _map;
        init();
    }

    public void init()
    {
        fR = map.get(DcMotor.class, "fr");
        fL = map.get(DcMotor.class, "fl");
        bR = map.get(DcMotor.class, "br");
        bL = map.get(DcMotor.class, "bl");
        armR = map.get(DcMotor.class, "ar");
        armL = map.get(DcMotor.class, "al");
        clawR = map.get(DcMotor.class, "cr");
        clawL = map.get(DcMotor.class, "cl");

        fR.setDirection(DcMotorSimple.Direction.FORWARD);
        bR.setDirection(DcMotorSimple.Direction.FORWARD);
        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        armR.setDirection(DcMotorSimple.Direction.FORWARD);
        armL.setDirection(DcMotorSimple.Direction.REVERSE);
        clawR.setDirection(DcMotorSimple.Direction.FORWARD);
        clawL.setDirection(DcMotorSimple.Direction.REVERSE);

        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clawR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clawL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sideArm = map.get(Servo.class, "sidearm");

        sideArm.scaleRange(.52, 1);
        sideArm.setPosition(0);

        color = new Sensors(map);
    }

    public void SetClaw(double power)
    {
        power = Range.clip(power, -1d, 1d);
        clawR.setPower(power);
        clawL.setPower(power);
    }

    public void SetArm(double power)
    {
        power = Range.clip(power, -1d, 1d);
        armL.setPower(power);
        armR.setPower(power);
    }

    public void Turn(double power)
    {
        SetDrive(power, -power);
    }

    public void Drive(double power)
    {
        SetDrive(power, power);
    }

    public void SetDrive(double powerLeft, double powerRight)
    {
        SetLeft(powerLeft);
        SetRight(powerRight);
    }

    public void SetLeft(double power)
    {
        power = Range.clip(power, -1d, 1d);
        fL.setPower(power);
        bL.setPower(power);
    }

    public void SetRight(double power)
    {
        power = Range.clip(power, -1d, 1d);

        fR.setPower(power);
        bR.setPower(power);
    }

    public void clear()
    {
        SetDrive(0, 0);
    }
}