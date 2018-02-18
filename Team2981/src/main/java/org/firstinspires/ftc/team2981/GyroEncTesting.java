package org.firstinspires.ftc.team2981;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by 200462069 on 2/7/2018.
 */

@TeleOp(name="GyroEnc")
public class GyroEncTesting extends OpMode {

    private DcMotor fL = null, fR = null, bL = null, bR = null;
    private BNO055IMU sensor = null;

    @Override
    public void init() {
        fL = hardwareMap.get(DcMotor.class, "frontLeft");
        fR = hardwareMap.get(DcMotor.class, "frontRight");
        bL = hardwareMap.get(DcMotor.class, "backLeft");
        bR = hardwareMap.get(DcMotor.class, "backRight");

        sensor = hardwareMap.get(BNO055IMU.class, "sensor");

        fR.setDirection(DcMotorSimple.Direction.FORWARD);
        bR.setDirection(DcMotorSimple.Direction.FORWARD);
        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);

        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        sensor.initialize(param);

    }

    public void start(){
        sensor.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    @Override
    public void loop() {
        Orientation orient = sensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        double[] angles = {AngleUnit.DEGREES.normalize(orient.firstAngle), AngleUnit.DEGREES.normalize(orient.secondAngle), AngleUnit.DEGREES.normalize(orient.thirdAngle)};
        double temp = sensor.getTemperature().toUnit(TempUnit.FARENHEIT).temperature;

        telemetry.addData("Orientation: ", "(%.1f, %.1f %.1f)", angles[0], angles[1], angles[2]);
        telemetry.addData("Temperature: ", temp);
        telemetry.addData("Encoders [fL,fR,bL,bR]: ", "[%d, %d, %d, %d]", fL.getCurrentPosition(), fR.getCurrentPosition(),
                bL.getCurrentPosition(), bR.getCurrentPosition());

    }
}
