#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <string>
using namespace std;

const float eps = 1e-7;

struct Node3d {
    float _x = 0;
    float _y = 0;
    float _z = 0;
};

struct Line {
    Node3d _begin;
    Node3d _end;
};

struct Vector3d {
    Vector3d(float x, float y, float z) : _x(x), _y(y), _z(z) {}
    Vector3d(Node3d begin, Node3d end) {
        _x = end._x - begin._x;
        _y = end._y - begin._y;
        _z = end._z - begin._z;
    }

    Vector3d prod(const Vector3d& other) const {
        Vector3d(_y*other._z - _z*other._y, _z*other._x - _x*other._z, _x*other._y - _y*other._x);
    }

    float dot(const Vector3d& other) const {
        return (_x*other._x + _y*other._y + _z*other._z);
    }

    float norm() {
        return sqrt(_x*_x + _y*_y + _z*_z);
    }

    Vector3d operator*(float a) {
        _x *= a;
        _y *= a;
        _z *= a;
    }

    Vector3d operator/(float a) {
        _x /= a;
        _y /= a;
        _z /= a;
    }

    void operator=(const Vector3d& other) {
        _x = other._x;
        _y = other._y;
        _z = other._z;
    }

    float _x = 0;
    float _y = 0;
    float _z = 0;
};

class Polyline {
public:
    struct ProjectReport {
        size_t _seg = 1;
        float _dist = 0;
        float _param = 0;
        Node3d _node;

        bool operator<(ProjectReport other) {
            return _dist < other._dist;
        }

        ostream& operator<<(ostream& os) {
            os << "segment " << _seg << " parameter " << _param << " point " << _node;
        }
    };

    void AddNode(Node3d node) {
        _nodes.push_back(node);
    }

    const vector<Node3d>& GetNodes() {
        return _nodes;
    }

    vector<ProjectReport> FindProjects(Node3d point) {
        vector<ProjectReport> result;
        for (size_t i = 0; i < _nodes.size() - 1; ++i) {
            ProjectReport report = FindClosest(point, {_nodes[i], _nodes[i + 1]});
            report._seg = i + 1;
            result.push_back(report);
        }
        sort(result.begin(), result.end());
        size_t j = 1;
        while (result[j]._dist - result[0]._dist < eps) ++j;
        result.resize(j);
        return result;
    }

private:
    vector<Node3d> _nodes;
};

Polyline::ProjectReport FindClosest(Node3d point, Line segment) {
    Polyline::ProjectReport result;

    Vector3d forward(segment._begin, segment._end);
    Vector3d backward(segment._end, segment._begin);
    Vector3d from_begin(segment._begin, point);
    Vector3d from_end(segment._end, point);

    if (from_begin.dot(forward) < 0) {
        result._node = segment._begin;
        result._dist = from_begin.norm();
        result._param = 0;
    }
    else if (from_end.dot(backward) < 0) {
        result._node = segment._end;
        result._dist = from_end.norm();
        result._param = 1;
    }
    else {
        float S = forward.prod(from_begin).norm() / 2;
        result._dist = S / forward.norm();
        result._param = (1 - pow(result._dist / (from_begin.norm()), 2)) * from_begin.norm() / forward.norm();
        Vector3d begin_to_perp = forward * result._param;
        result._node._x = segment._begin._x + begin_to_perp._x;
        result._node._y = segment._begin._y + begin_to_perp._y;
        result._node._y = segment._begin._z + begin_to_perp._z;
    }
    return result;
}

ostream& operator<<(ostream& os, const Node3d& node) {
        os << node._x << ' ' << node._y << ' ' << node._z;
}

void main() {
    Node3d point;
    string filename;
    cin >> filename;
    cin >> point._x >> point._y >> point._z;
}