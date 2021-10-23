#include "pro_geometry.h"
using namespace std;

int main() {
    Node3d point, temp;
    Polyline3d poly;
    vector<ProjectReport> reports;
    point._x = 0;
    point._y = 0;
    point._z = 0;
    reports = poly.FindProjects(point);
    cout << "Number of projections: " << reports.size() << endl;
    for (ProjectReport x : reports) {
    	cout << x << "\n";
    }
    return 0;
}