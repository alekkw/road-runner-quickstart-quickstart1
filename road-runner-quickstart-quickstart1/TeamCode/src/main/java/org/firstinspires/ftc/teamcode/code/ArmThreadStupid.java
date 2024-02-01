package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmThreadStupid implements Runnable{

    DcMotor motor;
    double target, threshold;

    boolean run = true;

    public ArmThreadStupid(DcMotor motor, double target, double threshold){
        this.motor = motor;
        this.target = target;
        this.threshold = threshold;
    }

    public void run(){
        double i_error = motor.getCurrentPosition() - target;
        double pow = 1;
        while(run){
            motor.setPower(pow);
            double c_error = motor.getCurrentPosition() - target;
            pow = (Math.abs(c_error) / Math.abs(i_error)) + 0.2;
            if(Math.abs(c_error) > Math.abs(i_error)) pow *= -1;
        }
    }

    public void stop(){
        run = false;
    }

    public boolean isBusy(){
        return !(Math.abs(motor.getCurrentPosition()) < target - threshold || Math.abs(motor.getCurrentPosition()) > target + threshold);
    }

}
