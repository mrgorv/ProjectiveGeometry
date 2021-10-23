#include "pro_geometry.h"
using namespace std;

Vector3d Vector3d::cross(const Vector3d &other) const {
    return Vector3d(_y * other._z - _z * other._y, _z * other._x - _x * other._z, _x * other._y - _y * other._x);
}

float Vector3d::dot(const Vector3d &other) const {
    return (_x * other._x + _y * other._y + _z * other._z);
}

ProjectReport Segment3d::FindProjectOn(Node3d point) {
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
        float dS = forward.cross(from_begin)._norm;                                                            /// doubled triangle square;
        result._dist = dS / forward._norm;                                                                     /// find triangle height;
        result._param = sqrtf(1 - pow(result._dist / from_begin._norm, 2)) * from_begin._norm / forward._norm; /// param = |AH|/|AB|;
        Vector3d begin_to_perp = forward * result._param;                                                      /// find H as A + param*AB.
        result._node._x = _begin._x + begin_to_perp._x;
        result._node._y = _begin._y + begin_to_perp._y;
        result._node._z = _begin._z + begin_to_perp._z;
    }
    return result;
}

/// Output operators for nodes and full projection result.
ostream &operator<<(ostream &os, const Node3d &node) {
    os << node._x << ' ' << node._y << ' ' << node._z;
    return os;
}
ostream &operator<<(ostream &os, ProjectReport r) {
    os << "segment " << r._seg << " parameter " << r._param << " point " << r._node;
    return os;
}

void Polyline3d::AddNode(Node3d node) {
    _nodes.push_back(node);
}

const vector<Node3d>& Polyline3d::GetNodes() {
    return _nodes;
}
vector<ProjectReport> Polyline3d::FindProjects(Node3d point) {
    vector<ProjectReport> result;
    for (size_t i = 0; i < _nodes.size() - 1; ++i)
    {
        Segment3d s = {_nodes[i], _nodes[i + 1]};
        ProjectReport report = s.FindProjectOn(point);
        report._seg = i + 1;
        if (i != 0)
        {   
            /// if this projection has already been found before, it won't be added;
            if (!(report._node == result[result.size() - 1]._node)) result.push_back(report); 
        }
        else result.push_back(report);
    }
    sort(result.begin(), result.end());
    size_t j = 1;
    while (j < result.size() && result[j]._dist - result[0]._dist < EPS) ++j; /// find closest projections in a sorted vector;
    result.resize(j);                                                         /// remove all the others.
    return result;
}