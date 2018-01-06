package org.firstinspires.ftc.team2981.structural;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by 200462069 on 10/14/2017.
 */

public class Sensors {

    //private BNO055IMU gyro = null;
    private ModernRoboticsI2cColorSensor color = null;
    private HardwareMap map = null;

    public Sensors(HardwareMap map) {
        this.map = map;
    }

    public void init() {
        //gyro = (BNO055IMU) map.gyroSensor.get("gyro");
        color = (ModernRoboticsI2cColorSensor) map.colorSensor.get("color");
    }

    public void calibrate() {
        //gyro.calibrate();
    }

    public boolean isGyroCalibrating() {
        //return gyro.isCalibrating();
        return false;
    }

    public void resetGyro() {
        //gyro.resetZAxisIntegrator();
    }

    //public int getGyroHeading(){
    //return gyro.getIntegratedZValue();

    //}

    public int[] getRGB() {
        return new int[]{color.red(), color.green(), color.blue()};
    }


}
