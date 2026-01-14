package org.teamtitanium.optimization;

import java.util.function.Consumer;

import org.ejml.simple.SimpleMatrix;
import org.wpilib.math.autodiff.Variable;
import org.wpilib.math.autodiff.VariableMatrix;
import org.wpilib.math.optimization.Problem;

public class ShooterOptimization {
    static double fieldWidth = 8.2296;
    static double fieldLength = 16.4592;
    static SimpleMatrix targetWrtField = new SimpleMatrix(
            new double[][] { { fieldLength / 2 }, { fieldWidth / 2 }, { 2.64 }, { 0.0 }, { 0.0 }, { 0.0 } });
    static SimpleMatrix robotToField = new SimpleMatrix(
            new double[][] { { fieldLength / 4.0 }, { fieldWidth / 4.0 }, { 0.0 }, { 1.524 }, { -1.524 }, { 0.0 } });
    static SimpleMatrix shooterToRobot = new SimpleMatrix(
            new double[][] { { 0.0 }, { 0.0 }, { 1.2 }, { 0.0 }, { 0.0 }, { 0.0 } });
    static SimpleMatrix shooterToField = robotToField.plus(shooterToRobot);
    double targetRadius = 0.61;
    Variable g = new Variable(9.806);

    public static void main(String[] args) {
        try (var problem = new Problem()) {
            double maxInitialVelocity = 10.0;

            int N = 10;
            var T = problem.decisionVariable();
            T.setValue(1);
            var dt = T.div(N);

            var X = problem.decisionVariable(6, N);

            var p = X.block(0, 0, 3, N);
            var p_x = X.row(0);
            var p_y = X.row(1);
            var p_z = X.row(2);

            var v = X.block(3, 0, 3, N);
            var v_x = X.row(3);
            var v_y = X.row(4);
            var v_z = X.row(5);

            // var v0_wrt_shooter = X.block(3, 0, 3, 1).minus(shooterToField.)
        }
    }

    private double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private VariableMatrix f(VariableMatrix x) {
        var v_x = x.get(3, 0);
        var v_y = x.get(4, 0);
        var v_z = x.get(5, 0);
        return new VariableMatrix(new Variable[][] { { v_x }, { v_y }, { v_z }, { dragForce(v_x).times(-1) },
                { dragForce(v_y).times(-1) }, { g.times(-1).minus(dragForce(v_z)) } });
    }

    private Variable dragForce(Variable velocity) {
        double rho = 1.204;
        double Cd = 0.45;
        double radius = 0.15;
        double A = Math.PI * radius * radius;
        double mass = 2.0;
        return velocity.times(velocity).times(0.5 * rho * Cd * A / mass);
    }
}
