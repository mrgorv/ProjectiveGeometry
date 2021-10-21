#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <string>
using namespace std;

const float EPS = 1e-7;

/// 3-dimensional node structure.
struct Node3d {
    float _x = 0;
    float _y = 0;
    float _z = 0;
};

/// Equality of nodes is defined by coordinate difference.
bool operator==(Node3d lhs, Node3d rhs) {
    bool xeq = false, yeq = false, zeq = false;
    xeq = abs(lhs._x - rhs._x) < EPS;
    yeq = abs(lhs._y - rhs._y) < EPS;
    zeq = abs(lhs._z - rhs._z) < EPS;
    return xeq && yeq && zeq;
}

/// Node projection result structure.
struct ProjectReport {
    size_t _seg = 1;  /// segment number;
    float _dist = 0;  /// distance to segment;
    float _param = 0; /// distance from segment beginning to node projection relative to segment length;
    Node3d _node;     /// projection coordinate.

    ///  Projections can be compared and therefore sorted by distance.
    bool operator<(ProjectReport other) {
        return _dist < other._dist;
    }
};

/// Vector, only direction and module, no fixed nodes.
struct Vector3d {
    Vector3d(float x, float y, float z) : _x(x), _y(y), _z(z) {
        _norm = sqrt(_x*_x + _y*_y + _z*_z);
    }
    /// Can be constructed directly by coordinates or from two nodes.
    Vector3d(Node3d begin, Node3d end) {
        _x = end._x - begin._x;
        _y = end._y - begin._y;
        _z = end._z - begin._z;
        _norm = sqrt(_x*_x + _y*_y + _z*_z);
    }

    /// Cross product. This vector is considered first.
    Vector3d cross(const Vector3d& other) const {
        return Vector3d(_y*other._z - _z*other._y, _z*other._x - _x*other._z, _x*other._y - _y*other._x);
    }

    /// Dot product. This vector is considered first.
    float dot(const Vector3d& other) const {
        return (_x*other._x + _y*other._y + _z*other._z);
    }

    /// Multiply vector by number.
    Vector3d operator*(float a) {
    	return Vector3d(_x*a, _y*a, _z*a);
    }

    /// Divide vector by number.
    Vector3d operator/(float a) {
        return Vector3d(_x/a, _y/a, _z/a);
    }

    /// Assignment operator.
    void operator=(const Vector3d& other) {
        _x = other._x;
        _y = other._y;
        _z = other._z;
        _norm = other._norm;
    }

    float _x = 0;
    float _y = 0;
    float _z = 0;
    float _norm = 0;
};

/// Segment of a straight line. Two fixed nodes.
struct Segment3d {
    Node3d _begin;
    Node3d _end;

    /// Finds projection of a given node on this segment.
    ProjectReport FindProjectOn(Node3d point) {
        ProjectReport result;
        Vector3d forward(_begin, _end);     /// vector, collinear with this segment, pointing from beginning to end (AB);
        Vector3d backward(_end, _begin);    /// vector, collinear with this segment, pointing from end to beginning (BA);
        Vector3d from_begin(_begin, point); /// vector, pointing from beginning of this segment to projected node (AM);
        Vector3d from_end(_end, point);     /// vector, pointing from end of this segment to projected node (BM);

        /// If dot product of two vectors is < 0, then angle between them is > 90 deg.
        /// Beginning of this segment is the closest point.
        if (from_begin.dot(forward) < 0) {
            result._node = _begin;
            result._dist = from_begin._norm;
            result._param = 0;
        }
        /// End of this segment is the closest point.
        else if (from_end.dot(backward) < 0) {
            result._node = _end;
            result._dist = from_end._norm;
            result._param = 1;
        }
        /** Norm of cross product of two vectors is equal to doubled square of a triangle, defined by these vectors.
            It is also equal to half a distance between one of the verticles and the opposide segment * length of such a segment.
            For point M, segment AB and distance MH :
            2S = |AB x AM|  == |MH|*|AB|;
            sin(MAB) = sqrt(1 - cos(MAB)^2);
            cos(MAB) = |MH|/|AM|;
            AH = AM*cos(MAB).
        */
        else {
            float dS = forward.cross(from_begin)._norm;                                                             /// doubled triangle square;
            result._dist = dS / forward._norm;                                                                      /// find triangle height;
            result._param = sqrtf(1 - pow(result._dist / from_begin._norm, 2)) * from_begin._norm / forward._norm;  /// param = |AH|/|AB|;
            Vector3d begin_to_perp = forward * result._param;                                                       /// find H as A + param*AB.
            result._node._x = _begin._x + begin_to_perp._x;
            result._node._y = _begin._y + begin_to_perp._y;
            result._node._z = _begin._z + begin_to_perp._z;
        }
        return result;
    }
};

/// Output operators for nodes and full projection result.
ostream& operator<<(ostream& os, const Node3d& node) {
        os << node._x << ' ' << node._y << ' ' << node._z;
        return os;
}
ostream& operator<<(ostream& os, ProjectReport r) {
        os << "segment " << r._seg << " parameter " << r._param << " point " << r._node;
        return os;
}

/// Polyline is defined as a vector of nodes.
class Polyline {
public:
    void AddNode(Node3d node) {
        _nodes.push_back(node);
    }

    /// Returns const reference to nodes.
    const vector<Node3d>& GetNodes() {
        return _nodes;
    }

    /// Find projections of a given node on polyline.
    vector<ProjectReport> FindProjects(Node3d point) {
        vector<ProjectReport> result;
        /// Find projection on every segment.
        for (size_t i = 0; i < _nodes.size() - 1; ++i) {
        	Segment3d s = {_nodes[i], _nodes[i + 1]};
            ProjectReport report = s.FindProjectOn(point);
            report._seg = i + 1;
            if (i != 0) {
                if (!(report._node == result[result.size() - 1]._node)) result.push_back(report); /// if this projection has already been found before, it won't be added;
            }
            else result.push_back(report);
            
        }
        sort(result.begin(), result.end());
        size_t j = 1;
        while (j < result.size() && result[j]._dist - result[0]._dist < EPS) ++j;                /// find closest projections in a sorted vector;
        result.resize(j);                                                                        /// remove all the others.
        return result;
    }

private:
    vector<Node3d> _nodes;
};

int main() {
    Node3d point, temp;
    Polyline poly;
    vector<ProjectReport> reports;
    string filename;
    cin >> filename;
    ifstream in(filename);
    if (in.is_open()) {
    	while (in >> temp._x >> temp._y >> temp._z) {
    		poly.AddNode(temp);
    	}
    }
    in.close();
    cin >> point._x >> point._y >> point._z;
    reports = poly.FindProjects(point);
    // cout << "Number of projections: " << reports.size() << endl;
    for (ProjectReport x : reports) {
    	cout << x << "\n";
    }
    return 0;
}
