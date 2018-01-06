package org.firstinspires.ftc.team2981.structural;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;

/**
 * Created by 200462069 on 10/14/2017.
 */

public class Vision {

    private VuforiaLocalizer vuforia;
    private VuforiaTrackables trackables;
    private VuforiaTrackable relic;
    private VuforiaLocalizer.Parameters parameters;
    private OpenGLMatrix pose, phone, blueBox, blueRelic, redBox, redRelic, last;
    private final float inBotWidth = 18f;
    private final float mmPerInch = 25.4f;
    private final float mmBotWidth = mmPerInch * inBotWidth;
    private final float mmFtcFieldWidth = ((12 * 12) - 2) * mmPerInch;
    private int loc;

    public Vision(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        //parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AaDxUz3/////AAAAGSJYyeGWDU3Im6rx85GbYy5gq8Raja7mkDAmDCUs6BvNAusEF4omrCaZUFeEG7gyW/Sq1exxlBmowJ4IY2ICrleyyxb1XJaFw0IsYhuBzESI/duL9SW2gVXcULoqBd7q4wniSHWZNNlkMYiuSbaW6z7299VJzV0QEi0HugnY+5PhZHUts9CU+lGIukrkAIDWP5bXOEmERBRpl4XKWIviWeCGHiVQVwAjeBEPnX1fsqRf+178gAoXEXDanp9cHriUGyU4a0vqhvJyb2LoQG5NrNLoFGUMU45pTWdjjY8TuVv9sfYSVwcboP2vzFeh8TVBbQRJrNrdWiRbw35nn+JSrZY6ulR5ZSDTq7l1apzxTy/s\n";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        trackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relic = trackables.get(0);
        relic.setName("relicVuMarkTemplate");

        phone = OpenGLMatrix.translation(5.75f * mmPerInch, -6.625f * mmPerInch, 5.25f * mmPerInch).multiplied(OpenGLMatrix.rotation(
                AxesReference.EXTRINSIC, AxesOrder.XZX, AngleUnit.DEGREES, 90, 0, 0));

        blueBox = OpenGLMatrix.translation(15.25f * mmPerInch, mmFtcFieldWidth / 2, 5.75f).multiplied(OpenGLMatrix.rotation(
                AxesReference.EXTRINSIC, AxesOrder.XZX, AngleUnit.DEGREES, 90, 0, 0));
        blueRelic = OpenGLMatrix.translation(-55.625f * mmPerInch, mmFtcFieldWidth / 2, 5.75f).multiplied(OpenGLMatrix.rotation(
                AxesReference.EXTRINSIC, AxesOrder.XZX, AngleUnit.DEGREES, 90, 0, 0));

        redBox = OpenGLMatrix.translation(32.25f * mmPerInch, -mmFtcFieldWidth / 2, 5.75f).multiplied(OpenGLMatrix.rotation(
                AxesReference.EXTRINSIC, AxesOrder.XZX, AngleUnit.DEGREES, 90, 180, 0));
        redRelic = OpenGLMatrix.translation(-38.625f * mmPerInch, -mmFtcFieldWidth / 2, 5.75f).multiplied(OpenGLMatrix.rotation(
                AxesReference.EXTRINSIC, AxesOrder.XZX, AngleUnit.DEGREES, 90, 180, 0));

        ((VuforiaTrackableDefaultListener) relic.getListener()).setPhoneInformation(phone, parameters.cameraDirection);

    }

    public void start(int loc) {
        trackables.activate();
        switchLocation(loc);
    }

    public void stop() {
        trackables.deactivate();
    }

    public void switchLocation(int loc) {
        this.loc = loc;
        switch (loc) {
            case 1:
                relic.setLocation(blueBox);
                break;
            case 2:
                relic.setLocation(blueRelic);
                break;
            case 3:
                relic.setLocation(redBox);
                break;
            case 4:
                relic.setLocation(redRelic);
                break;
        }
    }

    public int getLoc() {
        return loc;
    }

    public RelicRecoveryVuMark trackMark() {
        pose = ((VuforiaTrackableDefaultListener) relic.getListener()).getPose();
        return RelicRecoveryVuMark.from(relic);
    }

    public OpenGLMatrix trackLocation() {
        OpenGLMatrix robotLocTrans = ((VuforiaTrackableDefaultListener) relic.getListener()).getUpdatedRobotLocation();
        if (robotLocTrans != null) last = robotLocTrans;
        if (last != null) return last;
        else return null;
    }

    public float[] getTrans() {
        if (pose != null) {
            VectorF trans = pose.getTranslation();
            return new float[]{trans.get(0), trans.get(1), trans.get(2)};
        } else return null;
    }

    public float[] getRot() {
        if (pose != null) {
            Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            return new float[]{rot.firstAngle, rot.secondAngle, rot.thirdAngle};
        } else return null;
    }
}
