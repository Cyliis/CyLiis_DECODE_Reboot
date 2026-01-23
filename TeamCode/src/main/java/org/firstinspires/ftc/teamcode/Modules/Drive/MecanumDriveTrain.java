package org.firstinspires.ftc.teamcode.Modules.Drive;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevSPARKMini;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Math.PIDController;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;

@Config
public class MecanumDriveTrain {




    public enum State{
        DRIVE , PID , CLIMB;
    }
    State state;

    DcMotorEx frontLeft , frontRight;
    DcMotorEx backLeft , backRight;

    public  double targetX , targetY ,x=0 ,y=0 ;
    public static double targetHeading;
    public static double error;
    double rotation;
    public boolean usingTargetHeading=true;

    public static boolean frontLeftreversed=true , frontRightreversed=false , backLeftreversed=true , backRightreversed=false;

    public static double lateralMultiplier=2.5;
    public static  double realHeading;

    public static double kp=0.0095 , ki=0 , kd=0;
    public static double KP=1.78 , KI , KD=0.16;
    public PIDController controllerX=new PIDController(kp , ki , kd) , controllerY=new PIDController(kp , ki , kd) , controllerHeading=new PIDController(KP , KI , KD);

    public MecanumDriveTrain(State initialState)
    {
        state=initialState;

        frontLeft= Hardware.mch2;
        frontRight=Hardware.mch3;

        backLeft=Hardware.mch0;
        backRight=Hardware.mch1;

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //setTargetVector(0 , 0 , 0);

    }
    public boolean inPosition()
    {
        double heading= Odo.getHeading();
        if(heading<0)realHeading=Math.abs(heading);
        else realHeading=2*Math.PI-heading;

        error=targetHeading-realHeading;
        if(Math.abs(error)>Math.PI)error=-Math.signum(error)*(2*Math.PI-Math.abs(error));

        if(Math.abs(targetX-Odo.getX())<25 && Math.abs(targetY-Odo.getY())<25 && Math.abs(error)<0.1 && Math.abs(Odo.odo.getHeadingVelocity())<0.5)return true;
        return false;
    }
    public boolean inPosition( double x , double y , double Error)
    {
        double heading= Odo.getHeading();
        if(heading<0)realHeading=Math.abs(heading);
        else realHeading=2*Math.PI-heading;

        error=targetHeading-realHeading;
        if(Math.abs(error)>Math.PI)error=-Math.signum(error)*(2*Math.PI-Math.abs(error));

        if(Math.abs(targetX-Odo.getX())<x && Math.abs(targetY-Odo.getY())<y && Math.abs(error)<Error)return true;
        return false;
    }

    public void setTargetVector(double x , double y , double rx)
    {
        x*=lateralMultiplier;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx) , 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

    }
    public void setMode(State state)
    {
        this.state=state;
    }
    public void setTargetPosition(double x , double y , double heading)
    {
        targetX=x;
        targetY=y;
        targetHeading=heading-Math.floor((heading/ (Math.PI*2)))*Math.PI*2;
        usingTargetHeading=true;

    }

    public void setTargetSpecialPosition(double targetX , double targetY , double targetHeading)
    {
        this.targetX=targetX;
        this.targetY=targetY;
        rotation=targetHeading;
        usingTargetHeading=false;
    }
    public void setTargetPosition(Pose2D position)
    {
        targetX=position.x;
        targetY=position.y;
        targetHeading=position.heading;
        usingTargetHeading=true;
    }


    public void update()
    {
        if (state !=State.PID) {return;
        }
        controllerX.kp=kp;
        controllerY.kp=kp;

        controllerX.ki=ki;
        controllerY.ki=ki;

        controllerX.kd=kd;
        controllerY.kd=kd;

        controllerHeading.kp=KP;
        controllerHeading.ki=KI;
        controllerHeading.kd=KD;

        if(Double.isNaN(Odo.x) || Double.isNaN(Odo.y) || Double.isNaN(Odo.heading))
        {
            return;
        }
        x = controllerX.calculate(targetX, Odo.predictedX);

        y=-controllerY.calculate(targetY , Odo.predictedY);

        double heading= Odo.getHeading();
        if(heading<0)realHeading=Math.abs(heading);
        else realHeading=2*Math.PI-heading;

        if(usingTargetHeading==true)
        {error=targetHeading-realHeading;
            if(Math.abs(error)>Math.PI)error=-Math.signum(error)*(2*Math.PI-Math.abs(error));
            rotation= controllerHeading.calculate(error , 0);}


        setTargetVector(y*Math.cos(-heading) - x*Math.sin(-heading) , y*Math.sin(-heading)+x*Math.cos(-heading) , rotation);


    }


}