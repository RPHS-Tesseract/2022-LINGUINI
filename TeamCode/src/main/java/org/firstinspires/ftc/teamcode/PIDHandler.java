package org.firstinspires.ftc.teamcode;

public class PIDHandler {
    public double PIDTO(double Target, double Time, double OldTime, double Reference, double Integral, double LastError, double kP, double kI, double kD) {
        OldTime = Time - OldTime;
        double Error = Reference - Target;
        Integral = Integral + (Error * OldTime);
        double leftCraneDerivative = (Error - LastError) / OldTime;
        double Output = (kP * Error) + (kI * Integral) + (kD * leftCraneDerivative);
        LastError = Error;
        return (Output);
    }
}
