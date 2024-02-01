package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmThreadFull implements Runnable{

    private ArmThreadStupid slide;
    private ArmThread arm;

    public ArmThreadFull(DcMotor armMotor, DcMotor slideMotor){
        arm = new ArmThread(armMotor, armMotor.getCurrentPosition(), 10);
        slide = new ArmThreadStupid(slideMotor, slideMotor.getCurrentPosition(), 10);
    }

    @Override
    public void run() {

    }
}
