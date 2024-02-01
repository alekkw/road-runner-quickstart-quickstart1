package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.function.BiFunction;

public class PIDController implements Runnable{

    private double timestep;
    private LinearOpMode time;
    private DcMotor motor;

    private double target;
    private double kp = 0, ki = 0, kd = 0;

    private double e = 0, i = 0, d = 0;

    private boolean run = true;

    private int reversed = 1;

    private boolean dynamic = false;

    public PIDController(double timestep, LinearOpMode time, DcMotor motor){
        this.timestep = timestep;
        this.time = time;
        this.motor = motor;
        this.target = motor.getCurrentPosition();
    }

    public void setCoeffs(double kp, double ki, double kd){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public void setCoeffs(double[][] coeffs){
        this.coeffs = coeffs;
    }

    public void setTarget(double target){
        this.target = target;
        e = 0;
        i = 0;
        d = 0;
    }

    public void run(){
        double ctime = time.getRuntime();
        while(run){
            if(time.getRuntime() > ctime + timestep){
                double new_error = motor.getCurrentPosition() - target;
                d = (new_error - e) / timestep;
                i += ((new_error + e) / 2) * timestep;
                e = new_error;
                ctime = time.getRuntime();
            }
        }
    }

    // manual way of running controller if multithreading is not needed
    public void sudo_run(){
        double new_error = motor.getCurrentPosition() - target;
        d = (new_error - e) / timestep;
        i += ((new_error + e) / 2) * timestep;
        e = new_error;
    }

    public void stop(){
        run = false;
    }

    public void reverse(){
        reversed = -1;
    }

    public double getControl(){
        if(dynamic){
            int control_id = conditional.apply((double)motor.getCurrentPosition(), target);
            return (coeffs[control_id][0] * e + coeffs[control_id][1] * i + coeffs[control_id][2] * d) * reversed;

        }else{
            return (kp * e + ki * i + kd * d) * reversed;
        }
    }

    public double getTarget(){
        return target;
    }

    //for dynamic systems
    private BiFunction<Double, Double, Integer> conditional;
    private double coeffs[][];
    public PIDController(double timestep, LinearOpMode time, DcMotor motor,double[][] coeffs, BiFunction<Double, Double, Integer> conditional){
        this.timestep = timestep;
        this.time = time;
        this.motor = motor;
        this.target = motor.getCurrentPosition();
        this.coeffs = coeffs;
        this.conditional = conditional;
        dynamic = true;
    }




}
