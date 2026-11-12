import 'dart:math' as math;

/// Lightweight platform-neutral 2D vector for planar IK and spring bone math.
final class IkVec2 {
  final double x;
  final double y;

  const IkVec2(this.x, this.y);

  static const IkVec2 zero = IkVec2(0.0, 0.0);

  IkVec2 operator +(IkVec2 other) => IkVec2(x + other.x, y + other.y);
  IkVec2 operator -(IkVec2 other) => IkVec2(x - other.x, y - other.y);
  IkVec2 operator *(double s) => IkVec2(x * s, y * s);
  IkVec2 operator -() => IkVec2(-x, -y);

  double dot(IkVec2 other) => x * other.x + y * other.y;
  double get lengthSquared => x * x + y * y;
  double get length => math.sqrt(lengthSquared);

  IkVec2 get perpLeft => IkVec2(-y, x);

  IkVec2 normalized() {
    final l = length;
    if (l <= 1e-9) return const IkVec2(1.0, 0.0);
    final inv = 1.0 / l;
    return IkVec2(x * inv, y * inv);
  }
}
