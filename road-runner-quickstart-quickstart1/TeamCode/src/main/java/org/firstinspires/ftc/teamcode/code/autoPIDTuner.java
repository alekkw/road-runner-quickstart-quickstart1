package org.firstinspires.ftc.teamcode.code;

public class autoPIDTuner {

    static double timestep = 1; //seconds

    private PIDController controller;
    private double p, i, d;

    public autoPIDTuner(PIDController controller, double p_guess, double i_guess, double d_guess){
        this.controller = controller;
        p = p_guess;
        i = i_guess;
        d = d_guess;
    }




}
