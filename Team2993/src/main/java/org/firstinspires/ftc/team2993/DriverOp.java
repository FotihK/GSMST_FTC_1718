package org.firstinspires.ftc.team2993;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team2993.Hardware.*;



@TeleOp(name = "TeleOp - 2993", group="Regular")
public class DriverOp extends OpMode
{
    //
    // Configuration variables
    //

    public final double JOYSTICK_THRESHOLD = .1d;
    public final double MOTOR_SPEED        = .5d;

    public final double LIFT_SPEED_UP      = .75d;
    public final double LIFT_SPEED_DOWN    = .5d;

    public final double ARM_SPEED_UP       = .2d;
    public final double ARM_SPEED_DOWN     = .5d;



    //
    // Program variables
    //

    private RobotHardware Robot;
    
    private double  liftSpeed = 0.5d;
    private double  armSpeed  = 0.5d;

    private boolean intakeOn         = false;
    private boolean intakeButtonHeld = false;

    private boolean clawClosed     = false;
    private boolean clawButtonHeld = false;



    @Override
    public void init()
    {
        Robot = new RobotHardware(hardwareMap);

        Robot.sideArm.setPosition(0);
    }

    @Override
    public void loop()
    {
        DriverOne();
        DriverTwo();

        telemetry.update();
    }

    @Override
    public void stop()
    {
        Robot.SetMotors(0);
    }

    public void DriverOne()
    {
        double leftX  = gamepad1.left_stick_x;
        double leftY  = gamepad1.left_stick_y;
        double rightX = gamepad1.right_stick_x;

        double leftMagnitude = Math.sqrt(leftX * leftX + leftY * leftY);

        if      (leftMagnitude > JOYSTICK_THRESHOLD)
        {
            double leftTheta = Math.atan2(leftY, leftX) * 180 / Math.PI;
            leftTheta -= 22.5d;                                                                     // Rotate configuration 1/16 rotation clockwise
            leftTheta = 360 - leftTheta;
            leftTheta = (leftTheta + 360) % 360;

            int sector = (int) Math.floor(leftTheta / 45d);
            AddTelemetry(sector);
            switch (sector)
            {
                case 0:
                    Robot.SetMotors(MOTOR_SPEED, -MOTOR_SPEED, -MOTOR_SPEED, MOTOR_SPEED);
                    break;
                case 1:
                    Robot.SetMotors(MOTOR_SPEED, 0d, 0d, MOTOR_SPEED);
                    break;
                case 2:
                    Robot.SetMotors(MOTOR_SPEED);
                    break;
                case 3:
                    Robot.SetMotors(0d, MOTOR_SPEED, MOTOR_SPEED, 0d);
                    break;
                case 4:
                    Robot.SetMotors(-MOTOR_SPEED, MOTOR_SPEED, MOTOR_SPEED, -MOTOR_SPEED);
                    break;
                case 5:
                    Robot.SetMotors(0d, -MOTOR_SPEED, -MOTOR_SPEED, 0d);
                    break;
                case 6:
                    Robot.SetMotors(-MOTOR_SPEED);
                    break;
                case 7:
                    Robot.SetMotors(-MOTOR_SPEED, 0d, 0d, -MOTOR_SPEED);
                    break;
            }
        }
        else if (Math.abs(rightX) > JOYSTICK_THRESHOLD)
            Robot.SetMotors(MOTOR_SPEED  * rightX, -MOTOR_SPEED * rightX);
        else
            Robot.SetMotors(0d);
    }

    public void DriverTwo()
    {
        double leftY  = gamepad2.left_stick_y;
        double rightY = gamepad2.right_stick_y;



        if      (leftY > JOYSTICK_THRESHOLD)
            armSpeed = ARM_SPEED_UP * leftY;
        else if (leftY < -JOYSTICK_THRESHOLD)
            armSpeed = ARM_SPEED_DOWN * leftY;
        else
            armSpeed = 0d;

        Robot.SetArm(armSpeed);



        if      (gamepad2.b)
            liftSpeed = LIFT_SPEED_UP;
        else if (gamepad2.x)
            liftSpeed = -LIFT_SPEED_DOWN;
        else if (rightY > JOYSTICK_THRESHOLD)
            liftSpeed = LIFT_SPEED_UP * rightY;
        else if (rightY < -JOYSTICK_THRESHOLD)
            liftSpeed = LIFT_SPEED_DOWN * rightY;
        else
            liftSpeed = 0d;

        Robot.SetLift(liftSpeed);



        if ((gamepad2.right_bumper || gamepad2.y) && !clawButtonHeld)                               // Toggle claw
        {
            clawClosed = !clawClosed;
            Robot.SetClaw(clawClosed);
            clawButtonHeld = true;
        }
        else if (!(gamepad2.right_bumper || gamepad2.y))
            clawButtonHeld = false;



        if      (gamepad2.dpad_left)
                Robot.SetIntake(1);
        else if (gamepad2.dpad_down)
            Robot.SetIntake(0);
        else if (gamepad2.dpad_right)
            Robot.SetIntake(-1);



        if (gamepad2.dpad_up)
            Robot.sideArm.setPosition(0d);
    }

    public void AddTelemetry (double value)
    {
        telemetry.addLine(String.valueOf(value));
    }
}
