package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    public double kp, ki, kd;
    private double lastError = 0, integralSum = 0;
    private ElapsedTime timer;
    private double prevTime;

    public PIDController(double p, double i, double d){
        kp = p;
        ki = i;
        kd = d;

        timer = new ElapsedTime();
    }

    public double calculateResponse(double error){
        double d = (error - lastError);
        integralSum += error * ((timer.milliseconds()/1000.0) - prevTime);

        double output = (kp * error) + (ki * integralSum) + (kd * d);

        lastError = error;
        prevTime = timer.milliseconds()/1000.0;

        return output;
    }

    public void updateCoefficients(double p, double i, double d){
        kp = p;
        ki = i;
        kd = d;
    }

    public void reset(){
        lastError = 0;
        integralSum = 0;
        timer.reset();
    }

    public double getIntegralSum() {
        return integralSum;
    }
}
