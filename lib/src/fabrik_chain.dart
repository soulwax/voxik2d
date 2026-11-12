import 'dart:math' as math;

import 'math.dart';

/// Forward And Backward Reaching Inverse Kinematics (FABRIK) solver for multi-bone 2D chains.
final class FabrikChainSolver {
  const FabrikChainSolver._();

  static IkVec2 _clampDirectionAngle(
    IkVec2 dir,
    IkVec2 refDir,
    double maxAngleRad,
  ) {
    final cosMax = math.cos(maxAngleRad);
    final d = dir.dot(refDir).clamp(-1.0, 1.0).toDouble();
    if (d >= cosMax) return dir;
    final cross = refDir.x * dir.y - refDir.y * dir.x;
    final sign = cross >= 0.0 ? 1.0 : -1.0;
    final baseAngle = math.atan2(refDir.y, refDir.x) + sign * maxAngleRad;
    return IkVec2(math.cos(baseAngle), math.sin(baseAngle));
  }

  /// Solves joint positions for [lengths] anchored at [root] reaching toward [target].
  static List<IkVec2> solve({
    required IkVec2 root,
    required IkVec2 target,
    required List<double> lengths,
    List<IkVec2>? initialJoints,
    int maxIterations = 12,
    double tolerance = 1e-3,
    double? maxJointAngleRadians,
  }) {
    if (lengths.isEmpty) return [root];

    final n = lengths.length;
    final joints = List<IkVec2>.generate(n + 1, (i) {
      if (initialJoints != null && i < initialJoints.length) {
        return initialJoints[i];
      }
      var x = root.x;
      for (var k = 0; k < i; k++) {
        x += lengths[k];
      }
      return IkVec2(x, root.y);
    });

    final totalLength = lengths.fold<double>(0.0, (a, b) => a + math.max(1e-4, b));
    final toTarget = target - root;
    final distToTarget = toTarget.length;

    // Out of reach: stretch straight along target line
    if (distToTarget >= totalLength) {
      final dir = toTarget.normalized();
      joints[0] = root;
      for (var i = 0; i < n; i++) {
        joints[i + 1] = joints[i] + dir * math.max(1e-4, lengths[i]);
      }
      return joints;
    }

    for (var iter = 0; iter < math.max(1, maxIterations); iter++) {
      if ((joints.last - target).length <= tolerance) break;

      // Forward pass: pull tip to target
      joints[n] = target;
      for (var i = n - 1; i >= 0; i--) {
        final len = math.max(1e-4, lengths[i]);
        final dir = (joints[i] - joints[i + 1]).normalized();
        joints[i] = joints[i + 1] + dir * len;
      }

      // Backward pass: anchor root & propagate outward
      joints[0] = root;
      for (var i = 0; i < n; i++) {
        final len = math.max(1e-4, lengths[i]);
        var dir = (joints[i + 1] - joints[i]).normalized();
        if (maxJointAngleRadians != null && i > 0) {
          final prevDir = (joints[i] - joints[i - 1]).normalized();
          dir = _clampDirectionAngle(dir, prevDir, maxJointAngleRadians);
        }
        joints[i + 1] = joints[i] + dir * len;
      }
    }

    return joints;
  }
}
