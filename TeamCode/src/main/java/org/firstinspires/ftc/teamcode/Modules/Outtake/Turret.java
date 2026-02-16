package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;
import org.opencv.core.Mat;

@Config
public class Turret {

    private static double normalizeRadians(double angle) {
        angle %= (2.0 * Math.PI);
        if (angle < 0) angle += (2.0 * Math.PI);
        return angle;
    }

    private static final double SERVO_MAX_ANGLE = Math.toRadians(360);

    ServoImplEx servo1, servo2;

    public static double redX=0, redY=-840;
    public static double blueX=0, blueY=840;

    public static double turretX = 17, turretY = 0;

    public static double zeroPosition=0.5;

    public static double targetAngle;

    public static boolean Servo1REVERSED=true, Servo2REVERSED=true;

    public enum State{
        RED( redX, redY) , BLUE( blueX, blueY) , Middle(0.5 , 0.5);

        public double targetX, targetY;
        State(double targetX , double targetY)
        {
            this.targetX = targetX;
            this.targetY = targetY;
        }

    }
    public State state;

    public Turret(State state)
    {
        servo1 = Hardware.ssh1;
        servo2 = Hardware.ssh2;

        servo1.setPwmRange(new PwmControl.PwmRange(505 , 2495) );
        servo2.setPwmRange(new PwmControl.PwmRange(505 , 2495) );


        if(Servo1REVERSED)servo1.setDirection(Servo.Direction.REVERSE);
        if(Servo2REVERSED)servo2.setDirection(Servo.Direction.REVERSE);



        this.state = state;
    }

    public void setState(State state)
    {
        this.state = state;
    }

    private void updateStatePositions()
    {
        State.RED.targetX = redX;
        State.RED.targetY = redY;

        State.BLUE.targetX = blueX;
        State.BLUE.targetY = blueY;
    }
    public double angle=0;
    private void updateServosPosition()
    {
        switch (state) {
            case RED:
            case BLUE:


                targetAngle-=Odo.getHeading();
                targetAngle=normalizeRadians(targetAngle);

                targetAngle= targetAngle/SERVO_MAX_ANGLE;

                double targetPosition = targetAngle;

                targetPosition=Math.max(targetPosition, 0);
                targetPosition=Math.min(targetPosition , 1);

                servo1.setPosition(targetPosition);
                servo2.setPosition(targetPosition);
                break;

            case Middle:


                servo1.setPosition(zeroPosition);
                servo2.setPosition(zeroPosition);
                break;
        }
    }

    private void updateTargetAngle()
    {
        double x = Odo.getX();
        double y = Odo.getY();
        double yaw = Odo.getHeading();

        Pose2D offset = new Pose2D(turretX, turretY);
        offset.rotate(yaw);

        x += offset.x;
        y += offset.y;

        targetAngle = Math.atan2( ( state.targetY-y ) , ( state.targetX-x )  );
    }

    public void update()
    {
        updateStatePositions();
        updateTargetAngle();
        updateServosPosition();
    }
}