package org.firstinspires.ftc.teamcode.code;

public class Interpolator {

    private double i_l, i_u, o_l, o_u;

    private Interpolator[] points;

    public Interpolator(double input_lb, double input_ub, double output_lb, double output_ub){
        i_l = input_lb;
        i_u = input_ub;
        o_l = output_lb;
        o_u = output_ub;
    }

    public Interpolator(Interpolator... breakpoints){
        points = breakpoints;
    }

    public double calculate(double input){
        if(points == null){
            if(input > o_u) return o_u;
            return ((input - i_l) / (i_u - i_l)) * (o_u - o_l) + o_l;
        }else{
            int n = points.length;
            if(input > points[n - 1].getInputUpper()) return points[n - 1].getOutputUpper();
            if(input <= points[0].getInputLower()) return points[0].getOutputLower();

            //binary search
            int i = points.length / 2;
            int i_lb = 0;
            int i_ub = n-1;

            boolean isLow = input < points[i].getInputLower();
            boolean isHigh = input > points[i].getInputUpper();
            while(isLow || isHigh){
                if(input >= points[i].getInputLower()){
                    i_lb = i;
                    i = (i + i_ub + 1) / 2;
                    isHigh = input > points[i].getInputUpper();
                }else{
                    i_ub = i;
                    i = (i + i_lb) / 2;
                    isLow = input < points[i].getInputLower();
                }
            }
            return ((input - points[i].getInputLower()) / (points[i].getInputUpper() - points[i].getInputLower())) * (points[i].getOutputUpper() - points[i].getOutputLower()) + points[i].getOutputLower();
        }
    }

    private double getInputLower(){
        return i_l;
    }

    private double getInputUpper(){
        return i_u;
    }

    private double getOutputLower(){
        return o_l;
    }

    private double getOutputUpper(){
        return o_u;
    }



    /**
     * @param points points[0] = input_ub, points[1] = output_ub
     */
    public static Interpolator[] generateBreakpoints(double input_lb, double output_lb, double[]... points){
        Interpolator[] breakpoints = new Interpolator[points.length];
        breakpoints[0] = new Interpolator(input_lb, points[0][0], output_lb, points[0][1]);
        for(int i = 1; i < points.length; i++){
            breakpoints[i] = new Interpolator(points[i - 1][0], points[i][0], points[i - 1][1], points[i][1]);
        }
        return breakpoints;
    }

}
