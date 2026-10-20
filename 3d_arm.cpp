#include <iostream>
#include <cmath>
#include <utility>
using namespace std;

class RoboticArm {
private:
    double L1, L2;

public:
    RoboticArm(double length1, double length2) {
        L1 = length1;
        L2 = length2;
    }

    // Returns {{theta0, theta1_1, theta2_1}, {theta0, theta1_2, theta2_2}}
    // theta0 = base rotation, theta1 = shoulder, theta2 = elbow
    pair<pair<double,double>, pair<double,double>> calculateAngles(double x, double y, double z) {

        // Feature #1: Base joint from horizontal rotation
        const double PI = acos(-1.0);
        double theta0 = atan2(y, x) * (180.0 / PI);

        // Horizontal projection distance (replaces x in 2D IK)
        double r = sqrt(x*x + y*y);

        // Use r as horizontal reach and z as vertical (replaces old x, y)
        double dist = sqrt(r*r + z*z);

        if (dist > (L1 + L2) || dist < abs(L1 - L2)) {
            throw runtime_error("Target is out of arm range!");
        }

        double D = (r*r + z*z - L1*L1 - L2*L2) / (2 * L1 * L2);
        D = max(-1.0, min(1.0, D));

        double theta2_1 = acos(D)  * (180.0 / PI);
        double theta2_2 = -acos(D) * (180.0 / PI);

        double theta1_1 = (atan2(z, r) - atan2(L2 * sin(acos(D)), L1 + L2 * cos(acos(D)))) * (180.0 / PI);
        double theta1_2 = (atan2(z, r) - atan2(L2 * sin(-acos(D)), L1 + L2 * cos(-acos(D)))) * (180.0 / PI);

        return {{theta1_1, theta2_1}, {theta1_2, theta2_2}};
        // Note: theta0 is the same for both solutions; caller prints it separately
        (void)theta0;
    }

    // Prints theta0 separately since it doesn't split into two solutions
    double getBaseAngle(double x, double y) {
        const double PI = acos(-1.0);
        return atan2(y, x) * (180.0 / PI);
    }

    // Feature #2: Linear path generation using LERP
    void moveLinearly(double sx, double sy, double sz,
                      double ex, double ey, double ez,
                      int num_steps) {

        cout << "\n=== Linear Path: (" << sx << "," << sy << "," << sz << ")"
             << " -> (" << ex << "," << ey << "," << ez << ")"
             << " | Steps: " << num_steps << " ===\n\n";

        for (int i = 0; i < num_steps; i++) {
            double t = (num_steps == 1) ? 1.0 : (double)i / (num_steps - 1);

            double px = sx + t * (ex - sx);
            double py = sy + t * (ey - sy);
            double pz = sz + t * (ez - sz);

            cout << "Step " << (i + 1) << "/" << num_steps
                 << " | t=" << t
                 << " | Target: (" << px << ", " << py << ", " << pz << ")\n";

            try {
                double theta0  = getBaseAngle(px, py);
                auto solutions = calculateAngles(px, py, pz);

                cout << "  Base    (theta0) = " << theta0 << " deg\n";
                cout << "  [Sol 1 - Elbow Up]   Shoulder = " << solutions.first.first
                     << " deg  |  Elbow = " << solutions.first.second << " deg\n";
                cout << "  [Sol 2 - Elbow Down] Shoulder = " << solutions.second.first
                     << " deg  |  Elbow = " << solutions.second.second << " deg\n";
            } catch (exception &e) {
                cout << "  ERROR: " << e.what() << "\n";
            }
            cout << "\n";
        }
    }
};

int main() {
    RoboticArm arm(10.0, 10.0);

    cout << "=== Robotic Arm Simulator (3D + Path Planning) ===\n\n";

    // --- Demo 1: Single 3D point ---
    cout << "--- Demo 1: Single 3D Target ---\n";
    double tx = 8.0, ty = 6.0, tz = 3.0;
    try {
        double theta0  = arm.getBaseAngle(tx, ty);
        auto solutions = arm.calculateAngles(tx, ty, tz);

        cout << "Target: (" << tx << ", " << ty << ", " << tz << ")\n";
        cout << "Base    (theta0) = " << theta0 << " deg\n";
        cout << "Solution 1 (Elbow Up):\n";
        cout << "  Shoulder (theta1) = " << solutions.first.first  << " deg\n";
        cout << "  Elbow    (theta2) = " << solutions.first.second << " deg\n";
        cout << "Solution 2 (Elbow Down):\n";
        cout << "  Shoulder (theta1) = " << solutions.second.first  << " deg\n";
        cout << "  Elbow    (theta2) = " << solutions.second.second << " deg\n";
    } catch (exception &e) {
        cout << "ERROR: " << e.what() << "\n";
    }

    // --- Demo 2: Linear path ---
    arm.moveLinearly(5.0, 0.0, 0.0,   // start
                     8.0, 6.0, 4.0,   // end
                     5);              // steps

    // --- Demo 3: Interactive mode ---
    cout << "--- Interactive Mode (enter x y z, or -1 -1 -1 to exit) ---\n\n";
    while (true) {
        double x, y, z;
        cout << "Enter Target (x y z): ";
        cin >> x >> y >> z;

        if (x == -1 && y == -1 && z == -1) break;

        try {
            double theta0  = arm.getBaseAngle(x, y);
            auto solutions = arm.calculateAngles(x, y, z);

            cout << "\nTarget: (" << x << ", " << y << ", " << z << ")\n";
            cout << "Base    (theta0) = " << theta0 << " deg\n";
            cout << "Solution 1 (Elbow Up):\n";
            cout << "  Shoulder (theta1) = " << solutions.first.first  << " deg\n";
            cout << "  Elbow    (theta2) = " << solutions.first.second << " deg\n";
            cout << "Solution 2 (Elbow Down):\n";
            cout << "  Shoulder (theta1) = " << solutions.second.first  << " deg\n";
            cout << "  Elbow    (theta2) = " << solutions.second.second << " deg\n";
            cout << "-----------------------------\n\n";
        } catch (exception &e) {
            cout << "ERROR: " << e.what() << "\n\n";
        }
    }

    cout << "Program Ended.\n";
    return 0;
}
