package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Math.PIDController;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@Config
public class Shooter {

    CRServo motor1;
    CRServo motor2;
    public DcMotorEx encoder;

    public static double closeVelocity=1750, farVelocity=2050;

    public static double velocityNorm = 2400;

    PIDController controller;
    public static double KP=0, KI, KD=0;
    public static double Ks=0, Ka =0.004, Kv=0.00041;


    public static double threshold=80;
    public static boolean motorReversed=true;

    public enum State{
        Pause(1400), Close(closeVelocity), Far(farVelocity);
        public double targetVelocity;
        State(double targetVelocity)
        {
            this.targetVelocity=targetVelocity;
        }
    }
    public State state=State.Pause;

    public Shooter()
    {
        motor1 = Hardware.sch1;
        motor2 = Hardware.sch0;

        if(motorReversed)motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        if(motorReversed)motor2.setDirection(DcMotorSimple.Direction.REVERSE);


        encoder = Hardware.mch3;
        controller = new PIDController(KP, KI, KD);
    }

    public void setState(State state)
    {
        this.state=state;
    }

    public boolean isVelocityOptimal()
    {
        return Math.abs(state.targetVelocity-encoder.getVelocity()) < threshold;
    }

    double power=0;
    private void updateHardware()
    {
        //if(state==State.Pause)
        //{
        //    motor1.setPower(0);
        //    motor2.setPower(0);
        //    return;
        //}
        double feedforward = Ks + Ka * (state.targetVelocity - encoder.getVelocity()) + Kv * (state.targetVelocity);
        Pitch.correction=state.targetVelocity - encoder.getVelocity();
        power = controller.calculate(state.targetVelocity / velocityNorm, encoder.getVelocity() / velocityNorm) + feedforward;

        motor1.setPower(power);
        motor2.setPower(power);
    }

    private void updatePID()
    {
        controller.kp=KP;
        controller.ki=KI;
        controller.kd=KD;

        //State.Far.targetVelocity=farVelocity;
        //State.Close.targetVelocity=closeVelocity;
    }

    public void update()
    {
        updateHardware();
        updatePID();
    }

}
