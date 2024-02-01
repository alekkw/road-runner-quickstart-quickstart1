package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {

    public static double flp = 0.8, frp = 0.8, blp = 1, brp = 1;
    public static double p = 1, i = 0, d = 0, f = 0;
    public static double startPos = 0.75;
    public static double c_60_const = 80;

    public static double slide_kp = 0.006;
    public static double slide_ki = 0.002;
    public static double slide_kd = 0.0002;

    public static double arm_kp = 0.0047;
    public static double arm_ki = 0.00015;
    public static double arm_kd = 0.00008;

    public static double slide_kp_up = 0.0035;
    public static double slide_ki_up = 0.0008;
    public static double slide_kd_up = 0.00009;

    public static double slide_kp_down = 0.0018;
    public static double slide_ki_down = 0.00027;
    public static double slide_kd_down = 0.00013;

    public static int red_hue_low = 0, red_sat_low = 90, red_val_low = 80;
    public static int red_hue_high = 4, red_sat_high = 255, red_val_high = 255;
    public static int blue_hue_low = 104, blue_sat_low = 175, blue_val_low = 50;
    public static int blue_hue_high = 150, blue_sat_high = 255, blue_val_high = 180;

    public static double effectiveTrackWidth = 17;
}
