package org.firstinspires.ftc.teamcode.robotParts.movement.Bezier;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class PathFollower {
    PathBuilder path = new PathBuilder(this);
    double[] coordinate, derivative, sec_derivative;
    double r_circle;
    double trans_dist, shortest_dist, relative_pos;
    int shortest_time;
    double[] robot_position = new double[]{80,80}; //TODO: Omzetten naar realtime positie
    double d2;
    double Fcent, Ftrans, Fcorr;
    double Fcent_weight, Ftrans_weigth;
    double v_robot = 5; //TODO: Omzetten naar realtime snelheid
    double robotHeading = 45; //TODO: Omzetten naar realtime heading
    public void followPath(double[][] controlPoints) {
        path.buildPath(controlPoints);
        double coordinateLength = path.coordinate.length;
        while (shortest_time < 100) {
            for (int i = shortest_time; i < coordinateLength; i++) {
                coordinate = path.coordinate[i];
                trans_dist = Math.pow(coordinate[0] - robot_position[0], 2) + Math.pow(coordinate[1] - robot_position[1], 2);
                if (i != 0) {
                    if ((shortest_dist == 0) || (Math.abs(trans_dist) < shortest_dist) && !Double.isNaN(trans_dist)) {
                        shortest_dist = Math.abs(trans_dist);
                        shortest_time = i;
                    }
                } else {
                    shortest_dist = Math.abs(trans_dist);
                    shortest_time = i;
                }
            }
            coordinate = path.coordinate[shortest_time];
            sec_derivative = path.sec_derivative[shortest_time];
            derivative = path.derivative[shortest_time];
            r_circle = path.r_circle[shortest_time];
            d2 = (sec_derivative[1] * derivative[0] - sec_derivative[0] * derivative[1]) / Math.pow(sec_derivative[0], 2);
            relative_pos = (coordinate[0] - robot_position[0]) + (coordinate[1] - robot_position[1]);
            Fcent = Fcent_weight * Math.pow(v_robot, 2) / r_circle;
            if (d2 < 0) Fcent *= -1;
            Ftrans = -Math.sqrt(shortest_dist);
            if ((d2 < 0 && relative_pos < 0) || (d2 > 0 && relative_pos > 0)) Ftrans *= -1;
            Fcorr = Ftrans + Fcent;
            if (Fcorr > 1 || Fcorr < -1) {
                if (Fcorr > 1) Fcorr = 1;
                else Fcorr = -1;
            }
            double theta1 = Math.asin(Fcorr/*/motor.speed*/);
            double theta2 = Math.toRadians(robotHeading);
            double theta4 = Math.atan(derivative[1] / derivative[0]);
            double driveDirection = 90 - theta1 - theta2 + theta4;
        }
    }
}
