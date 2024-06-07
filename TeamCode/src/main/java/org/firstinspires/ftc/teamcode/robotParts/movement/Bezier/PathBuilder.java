package org.firstinspires.ftc.teamcode.robotParts.movement.Bezier;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class PathBuilder {
    public PathFollower myOpMode;
    private double Bx = 0, By = 0, dBx = 0, dBy = 0, d2Bx = 0, d2By = 0;
    private double pow = 0, dpow = 0, d2pow = 0;
    private double[] path_coordinate = {}, path_derivative = {}, path_sec_derivative = {};
    public double[][] coordinate = {{}}, derivative = {{}}, sec_derivative = {{}};
    public double[] r_circle = {};

    public PathBuilder(PathFollower opmode) {myOpMode = opmode;}

    public void buildPath(double[][] controlPoints) {
        int n = controlPoints.length;
        int i = 0;
        for (double t = 0; t <= 1; t += 0.01) {
            i = (int)(t*100);
            coordinate[i] = get_path_coordinate(controlPoints, n, t);
            derivative[i] = get_derivative(controlPoints, n, t);
            sec_derivative[i] = get_sec_derivative(controlPoints, n, t);
            r_circle[i] = (Math.pow(Math.pow(derivative[i][0],2) + Math.pow(derivative[i][1],2),1.5))/
                    (derivative[i][0]*sec_derivative[i][1]-derivative[i][1] *sec_derivative[i][0]);
        }
    }

    private double[] get_path_coordinate(double[][] controlPoints, int n, double t) {
        for (int i = 0; i <= n; i++) {
            pow = nCr(n, i) * Math.pow(1 - t, n - i) * Math.pow(t, i);
            Bx = Bx + pow * controlPoints[i][0];
            By = By + pow * controlPoints[i][1];
        }
        path_coordinate = new double[]{Bx, By};
        Bx = 0; By = 0;
        return path_coordinate;
    }

    private double[] get_derivative(double[][] controlPoints, int n, double t) {
        for (int i = 0; i <= n - 1; i++) {
            dpow = nCr(n - 1, i) * Math.pow(1 - t, n - i - 1) * Math.pow(t, i);
            dBx = dBx + dpow * (controlPoints[i + 1][0] - controlPoints[i][0]);
            dBy = dBy + dpow * (controlPoints[i + 1][1] - controlPoints[i][1]);
        }
        dBx = n * dBx;
        dBy = n * dBy;
        path_derivative = new double[]{dBx, dBy};
        dBx = 0; dBy = 0;
        return path_derivative;
    }
    private double[] get_sec_derivative(double[][] controlPoints, int n, double t) {
        for (int i = 0; i <= n-2; i++) {
            d2pow = nCr(n-2,i) * Math.pow(1-t,n-i-2) * Math.pow(t,i);
            d2Bx = d2Bx + d2pow * (controlPoints[i+2][0]-2*controlPoints[i+1][0] + controlPoints[i][0]);
            d2By = d2By + d2pow * (controlPoints[i+2][1]-2*controlPoints[i+1][1] + controlPoints[i][1]);
        }
        d2Bx = 2*n * d2Bx; d2By = 2*n * d2By;
        path_sec_derivative = new double[]{d2Bx,d2By};
        d2Bx = 0; d2By = 0;
        return path_sec_derivative;
    }
    static int nCr(int n, int r)
    {
        return fact(n) / (fact(r) * fact(n - r));
    }
    static int fact(int n)
    {
        int res = 1;
        for (int i = 2; i <= n; i++)
            res = res * i;
        return res;
    }
}
