#pragma once
#include <cmath>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

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
    
    /// Cross product. This vector is considered first.
    Vector3d cross(const Vector3d& other) const;

    /// Dot product. This vector is considered first.
    float dot(const Vector3d& other) const;

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
    ProjectReport FindProjectOn(Node3d point);
};

/// Output operators for nodes and full projection result.
std::ostream& operator<<(std::ostream& os, const Node3d& node) {
        os << node._x << ' ' << node._y << ' ' << node._z;
        return os;
}
std::ostream& operator<<(std::ostream& os, ProjectReport r) {
        os << "segment " << r._seg << " parameter " << r._param << " point " << r._node;
        return os;
}

/// Polyline is defined as a vector of nodes.
class Polyline3d {
public:
    void AddNode(Node3d node);
    
    /// Returns const reference to nodes.
    const std::vector<Node3d>& GetNodes();

    /// Find projections of a given node on polyline.
    std::vector<ProjectReport> FindProjects(Node3d point);

private:
    std::vector<Node3d> _nodes;
};
