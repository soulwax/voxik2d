import 'dart:math' as math;

import 'math.dart';

/// Solved joint positions and angles for a 2-segment planar limb.
final class PlanarTwoBoneIkResult {
  final IkVec2 root;
  final IkVec2 midJoint;
  final IkVec2 endEffector;
  final double upperAngleRad;
  final double lowerAngleRad;
  final bool targetClamped;

  const PlanarTwoBoneIkResult({
    required this.root,
    required this.midJoint,
    required this.endEffector,
    required this.upperAngleRad,
    required this.lowerAngleRad,
    required this.targetClamped,
  });
}

/// Analytic law-of-cosines two-bone 2D IK solver with bend polarity and reach clamping.
final class PlanarTwoBoneIkSolver {
  const PlanarTwoBoneIkSolver._();

  static PlanarTwoBoneIkResult solve({
    required IkVec2 root,
    required IkVec2 target,
    required double upperLength,
    required double lowerLength,
    double bendSign = 1.0,
    double softReachFraction = 0.999,
  }) {
    final l1 = math.max(1e-4, upperLength);
    final l2 = math.max(1e-4, lowerLength);
    final minReach = (l1 - l2).abs() + 1e-4;
    final maxReach = math.max(minReach + 1e-4, (l1 + l2) * softReachFraction.clamp(0.5, 1.0));

    final toTarget = target - root;
    final rawDist = toTarget.length;
    final dir = rawDist <= 1e-8 ? const IkVec2(1.0, 0.0) : toTarget * (1.0 / rawDist);
    final dist = rawDist.clamp(minReach, maxReach).toDouble();
    final clamped = (rawDist < minReach) || (rawDist > maxReach);

    final cosAlpha = ((l1 * l1 + dist * dist - l2 * l2) / (2.0 * l1 * dist))
        .clamp(-1.0, 1.0)
        .toDouble();
    final alpha = math.acos(cosAlpha);

    final baseAngle = math.atan2(dir.y, dir.x);
    final sign = bendSign >= 0.0 ? 1.0 : -1.0;
    final upperAngle = baseAngle + sign * alpha;

    final mid = IkVec2(
      root.x + math.cos(upperAngle) * l1,
      root.y + math.sin(upperAngle) * l1,
    );
    final clampedEnd = root + dir * dist;
    final forearm = clampedEnd - mid;
    final lowerAngle = math.atan2(forearm.y, forearm.x);

    return PlanarTwoBoneIkResult(
      root: root,
      midJoint: mid,
      endEffector: clampedEnd,
      upperAngleRad: upperAngle,
      lowerAngleRad: lowerAngle,
      targetClamped: clamped,
    );
  }
}
