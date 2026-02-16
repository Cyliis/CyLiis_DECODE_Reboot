package org.firstinspires.ftc.teamcode.Wrappers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Math.LowPassFilter;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.opencv.core.Mat;

import java.lang.Math;

@Config
public class Odo {

    public  static org.firstinspires.ftc.teamcode.Wrappers.GoBildaPinpointDriver odo;
    public static double heading,x ,y, xVelocity, yVelocity, predictedX, predictedY;
    public static boolean INIT=false;
    public static Telemetry telemetry;
    public  static void init(HardwareMap hardwareMap)
    {
        if(INIT)return;
        INIT=true;
        odo=hardwareMap.get(org.firstinspires.ftc.teamcode.Wrappers.GoBildaPinpointDriver.class , "odo");
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED , GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.setEncoderResolution(org.firstinspires.ftc.teamcode.Wrappers.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setOffsets(131.45 , -59.201);
        odo.resetPosAndIMU();
    }
    //public static IMU imu;

    public  static void initForAuto(HardwareMap hardwareMap)
    {
        if(INIT){odo.setPosition(new Pose2D(DistanceUnit.MM , 0 , 0 , AngleUnit.RADIANS , 0));return;}
        INIT=true;
        odo=hardwareMap.get(org.firstinspires.ftc.teamcode.Wrappers.GoBildaPinpointDriver.class , "odo");

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED , GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.setEncoderResolution(org.firstinspires.ftc.teamcode.Wrappers.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setOffsets(131.45 , -59.201);
        odo.resetPosAndIMU();
    }

    public static void calibrate()
    {
        odo.resetPosAndIMU();
    }

    public static double getHeading()
    {
        return heading;
    }

    public static double getX()
    {
        return predictedX;
    }

    public static double getY() {return predictedY;}

    public static void reset()
    {
        odo.setPosition(new Pose2D(DistanceUnit.MM , 0 , 0 , AngleUnit.RADIANS , 0));
    }

    public static double filterParameter = 0.8;
    private static final LowPassFilter xVelocityFilter = new LowPassFilter(filterParameter, 0);
    private static final LowPassFilter yVelocityFilter = new LowPassFilter(filterParameter, 0);


    public static double xDeceleration = 120 * 25.4, yDeceleration = 150 * 25.4;
    public static double xRobotVelocity, yRobotVelocity;
    public static double forwardGlide, lateralGlide;
    public static double xGlide, yGlide;


    private static void updateGlide(){

        xRobotVelocity = xVelocity * Math.cos(-heading) - yVelocity * Math.sin(-heading);
        yRobotVelocity = xVelocity * Math.sin(-heading) + yVelocity * Math.cos(-heading);

        forwardGlide = Math.signum(xRobotVelocity) * xRobotVelocity * xRobotVelocity / (2.0 * xDeceleration);
        lateralGlide = Math.signum(yRobotVelocity) * yRobotVelocity * yRobotVelocity / (2.0 * yDeceleration);

        xGlide = forwardGlide * Math.cos(heading) - lateralGlide * Math.sin(heading);
        yGlide = forwardGlide * Math.sin(heading) + lateralGlide * Math.cos(heading);
    }

    public  static void update()
    {
        odo.update();

        if(Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(heading))return;


            heading=odo.getHeading();

            x=odo.getPosX();

            y=odo.getPosY();

            if(!Double.isNaN(x) && !Double.isNaN(y))
            {
            xVelocity = xVelocityFilter.getValue(odo.getVelocity().getX(DistanceUnit.MM));
            yVelocity = yVelocityFilter.getValue(odo.getVelocity().getY(DistanceUnit.MM));
            updateGlide();
            predictedX = x + xGlide;
            predictedY = y + yGlide;}
        }
}