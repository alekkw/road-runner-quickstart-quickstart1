package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FullArmController implements Runnable{

    private PIDController arm_control, slide_control;
    private Thread arm_thd, slide_thd;

    private DcMotor slide, arm;

    private Telemetry telemetry;

    private double arm_target_true, slide_target_true;

    private boolean run = true;

    double min_arm_pos_for_extend = -1460;
    final double min_slide_pos_to_down = -25;

    final static int threshold = 25;

    private boolean pause_arm = false, pause_slide = false;

    boolean condition1 = false, condition2 = false;

    private final double min_arm_pow = 0.021, min_slide_up_pow = 0.03, min_slide_down_pow = 0.023;

    LinearOpMode opModeReference;

    public FullArmController(double timestep, LinearOpMode time, DcMotor arm, DcMotor slide){
        this.arm_control = new PIDController(timestep, time, arm);
        arm_control.setCoeffs(RobotConstants.arm_kp, RobotConstants.arm_ki, RobotConstants.arm_kd);
        arm_control.reverse();
        arm_thd = new Thread(arm_control);
        this.arm = arm;
        arm_target_true = arm.getCurrentPosition();
        opModeReference = time;

        slide_control = new PIDController(timestep, time, slide,
                new double[][] {{RobotConstants.slide_kp_up, RobotConstants.slide_ki_up, RobotConstants.slide_kd_up},
                                {RobotConstants.slide_kp_down, RobotConstants.slide_ki_down, RobotConstants.slide_kd_down}},
                (Double current_pos, Double target_pos) -> (current_pos > target_pos) ? 0 : 1);
        slide_control.reverse();
        slide_thd = new Thread(slide_control);
        this.slide = slide;
        slide_target_true = slide.getCurrentPosition();

        this.telemetry = time.telemetry;
    }

    public void set_arm_target(double target){
        if(!(arm.getCurrentPosition() < arm_control.getTarget() && slide.getCurrentPosition() <= min_slide_pos_to_down && arm_control.getTarget() >= min_arm_pos_for_extend) || target < min_arm_pos_for_extend){
            arm_control.setTarget(target);
        }
        arm_target_true = target;
    }

    public void set_slide_target(double target){
        if(!(slide.getCurrentPosition() >= slide_control.getTarget() && arm.getCurrentPosition() >= min_arm_pos_for_extend)){
            slide_control.setTarget(target);
        }
        slide_target_true = target;
    }

    public void run(){
        arm_thd.start();
        slide_thd.start();

        while(run && opModeReference.opModeIsActive()){
            
            //sudo run threads to not waste threads
            

            //slide wants to extend but arm is too low
            if(slide.getCurrentPosition() >= slide_control.getTarget() && arm.getCurrentPosition() >= min_arm_pos_for_extend){//isInThreshold(slide.getCurrentPosition(), (int)slide_control.getTarget()) && isInThreshold(arm.getCurrentPosition(), (int)min_arm_pos_for_extend)
                if(!condition1){
                    condition2 = false;
                    condition1 = true;
                    slide_control.setTarget(slide.getCurrentPosition());
                }
                //telemetry.addData("slide wants to extend but arm is too low", "");
            }
            //arm wants to go down but slide is extended
            else if(arm.getCurrentPosition() < arm_control.getTarget() && slide.getCurrentPosition() <= min_slide_pos_to_down && arm_control.getTarget() >= min_arm_pos_for_extend){ //isInThreshold(arm.getCurrentPosition(), (int)arm_control.getTarget()) && isInThreshold(slide.getCurrentPosition(), (int)min_slide_pos_to_down) && isInThreshold(arm.getCurrentPosition(), (int)min_arm_pos_for_extend)
                if(!condition2){
                    condition1 = false;
                    condition2 = true;
                    arm_control.setTarget(min_arm_pos_for_extend);
                }
               // telemetry.addData("arm wants to go down but slide is extended", "");
            }else{
                if(condition1){
                    slide_control.setTarget(slide_target_true);
                    condition1 = false;
                }else if(condition2){
                    arm_control.setTarget(arm_target_true);
                    condition2 = false;
                }
               // telemetry.addData("All systems are good", "");
            }
            if(!pause_arm){
                double controlPow = arm_control.getControl();
                arm.setPower((Math.abs(controlPow) > min_arm_pow) ? controlPow : min_arm_pow - 0.03);
            }
            if(!pause_slide){
                double controlPow = slide_control.getControl();
                if(controlPow < 0){
                    //up
                    slide.setPower((Math.abs(controlPow) > min_slide_up_pow) ? controlPow : min_slide_up_pow - 0.03);
                }else{
                    //down
                    slide.setPower((Math.abs(controlPow) > min_slide_down_pow) ? controlPow : min_slide_down_pow - 0.03);
                }
            }
//            telemetry.addData("Arm Power: ", arm.getPower());
//            telemetry.addData("Slide Power: ", slide.getPower());
//            telemetry.addData("Arm Pos: ", arm.getCurrentPosition());
//            telemetry.addData("Slide Pos: ", slide.getCurrentPosition());
//            telemetry.addData("Arm Target: ", arm_control.getTarget());
//            telemetry.addData("Slide Target: ", slide_control.getTarget());
//            telemetry.addData("Is At Targets: ", isAtTargets());
//            telemetry.update();
        }
        arm_control.stop();
        slide_control.stop();
    }

    public void pause_arm(){
        pause_arm = true;
    }

    public void unpause_arm(){
        pause_arm = false;
    }

    public void pause_slide(){
        pause_slide = true;
    }

    public void unpause_slide(){
        pause_slide = false;
    }

    public void stop(){
        arm_control.stop();
        slide_control.stop();
        run = false;
    }

    public boolean isAtTargets(){
        return isInThreshold(arm.getCurrentPosition(), (int)arm_target_true) && isInThreshold(slide.getCurrentPosition(), (int)slide_target_true);
    }

    public void set_min_arm_pos_for_extend(double newVal){
        this.min_arm_pos_for_extend = newVal;
    }

    public static boolean isInThreshold(int value, int target){
        return value < target + FullArmController.threshold && value > target - FullArmController.threshold;
    }

}
