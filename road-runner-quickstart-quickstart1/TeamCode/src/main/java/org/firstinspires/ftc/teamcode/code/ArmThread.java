package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmThread implements Runnable{

    DcMotor armMotor;
    double targetPos, epsilon;

    boolean run = true;

    public ArmThread(DcMotor armMotor, double position, double epsilon){
        this.armMotor = armMotor;
        this.targetPos = position;
        this.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.epsilon = epsilon;
    }

    public void setTarget(double target){
        this.targetPos = target;
    }

    public void stop(){
        run = false;
    }

    @Override
    public void run() {
        while(run){
            this.armMotor.setPower(1);
            this.armMotor.setTargetPosition((int)targetPos);
            this.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public boolean isBusy(){
        return Math.abs(armMotor.getCurrentPosition()) > targetPos - epsilon && Math.abs(armMotor.getCurrentPosition()) < targetPos + epsilon;
    }
}
