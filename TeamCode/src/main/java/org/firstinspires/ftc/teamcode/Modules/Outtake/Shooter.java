package org.firstinspires.ftc.teamcode.Modules.Outtake;

import android.text.style.IconMarginSpan;

import androidx.annotation.CallSuper;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Math.PIDController;
import org.firstinspires.ftc.teamcode.OpModes.TeleOp;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@Config
public class Shooter {

    CRServo motor;
    public DcMotorEx encoder;

    public static double closeVelocity=2400, farVelocity=2400;

    PIDController controller;
    public static double KP, KI, KD;
    public static double Ks, Kp, Kv=0.00055;


    public static double threshold=400;
    public static boolean motorReversed=true;

    public enum State{
        Pause(0), Close(closeVelocity), Far(farVelocity);
        public double targetVelocity;
        State(double targetVelocity)
        {
            this.targetVelocity=targetVelocity;
        }
    }
    public State state=State.Pause;

    public Shooter()
    {
        motor = Hardware.sch1;

        if(motorReversed)motor.setDirection(DcMotorSimple.Direction.REVERSE);

        encoder = Hardware.mch3;
        controller = new PIDController(KP, KI, KD);
    }

    public void setState(State state)
    {
        this.state=state;
    }

    public boolean isVelocityOptimal()
    {
        return Math.abs(state.targetVelocity+encoder.getVelocity()) < threshold;
    }

    double power=0;
    private void updateHardware()
    {
        double feedforward = Ks + Kp * (state.targetVelocity + encoder.getVelocity()) + Kv * (state.targetVelocity);
        power = controller.calculate(state.targetVelocity, -encoder.getVelocity())+feedforward;


        motor.setPower(power);
    }

    private void updatePID()
    {
        controller.kp=KP;
        controller.ki=KI;
        controller.kd=KD;

        State.Far.targetVelocity=farVelocity;
        State.Close.targetVelocity=closeVelocity;
    }

    public void update()
    {
        updateHardware();
        updatePID();
    }

}
