package frc.lib.math;

public class Color {

    double k_r,k_g,k_b;
    double k_h,k_s,k_v;
    double max,min;

    public Color(double red, double green, double blue) {
        k_r = red;
        k_g = green;
        k_b = blue;
        max = Math.max(k_r, Math.max(k_g, k_b));
        min = Math.min(k_r, Math.min(k_g, k_b));

        if(max==min) k_h = 0;
        else if(max == k_r && k_g >= k_b) k_h = 60*(k_g-k_b)/(max-min);
        else if(max == k_r && k_g < k_b) k_h = 60*(k_g-k_b) + 360;
        else if (max == k_g) k_h = 60*(k_b-k_r)/(max-min) + 120;
        else if(max == k_b) k_h = 60*(k_r-k_g)/(max-min) + 240;
        else k_h = 0;

        k_s = max == 0 ? 0 : 1-min/max;
        k_v = max;
    }

    public double getHue() {
        return k_h;
    }

    public double getSaturation() {
        return k_s;
    }

    public double getValue() {
        return k_v;
    }
}
