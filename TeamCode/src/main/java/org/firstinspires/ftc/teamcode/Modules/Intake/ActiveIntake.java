package org.firstinspires.ftc.teamcode.Modules.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot.Hardware;

@Config
public class ActiveIntake {


    public static double intakePower = 1, reversePower = -0.8, transferPower = 0.2;
    public static boolean motorReversed=true;

    CRServo motor;

    public enum State{
        INTAKE(intakePower), REVERSE(reversePower), PAUSE(0), TRANSFER(transferPower);

        double power;
        State(double power)
        {
            this.power=power;
        }
    }
    State state=State.PAUSE;

    public ActiveIntake()
    {
        motor= Hardware.sch3;
        if(motorReversed)motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setState(State state)
    {
        this.state=state;
        motor.setPower(state.power);
    }

    private void updatePower()
    {
        State.INTAKE.power=intakePower;
        State.REVERSE.power=reversePower;
        State.TRANSFER.power=transferPower;
    }

    public void update()
    {
        updatePower();
    }
}
