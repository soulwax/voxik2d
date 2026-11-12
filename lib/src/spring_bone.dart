import 'dart:math' as math;

import 'math.dart';

/// Single particle bone node in a Verlet-integrated secondary-motion chain.
final class SpringBoneNode {
  IkVec2 position;
  IkVec2 prevPosition;
  final double boneLength;
  final IkVec2 restLocalDir;

  SpringBoneNode({
    required this.position,
    required this.boneLength,
    required this.restLocalDir,
  }) : prevPosition = position;
}

/// Damped Verlet spring-bone chain for secondary hair, scarf, cape, and antenna lag.
final class SpringBoneChain {
  final List<SpringBoneNode> nodes;
  double stiffness;
  double damping;
  IkVec2 gravity;

  SpringBoneChain._(
    this.nodes, {
    required this.stiffness,
    required this.damping,
    required this.gravity,
  });

  factory SpringBoneChain.linear({
    required IkVec2 root,
    int boneCount = 4,
    double boneLength = 0.5,
    IkVec2 restDirection = const IkVec2(-1.0, 0.0),
    double stiffness = 18.0,
    double damping = 0.88,
    IkVec2 gravity = const IkVec2(0.0, -9.8),
  }) {
    final dir = restDirection.normalized();
    final nodes = <SpringBoneNode>[
      SpringBoneNode(position: root, boneLength: 0.0, restLocalDir: dir),
    ];
    for (var i = 1; i <= math.max(1, boneCount); i++) {
      nodes.add(
        SpringBoneNode(
          position: root + dir * (boneLength * i),
          boneLength: math.max(0.05, boneLength),
          restLocalDir: dir,
        ),
      );
    }
    return SpringBoneChain._(
      nodes,
      stiffness: stiffness,
      damping: damping,
      gravity: gravity,
    );
  }

  /// Steps spring physics given updated [rootPosition] and external [windForce].
  void step(
    double dt, {
    required IkVec2 rootPosition,
    IkVec2 windForce = IkVec2.zero,
  }) {
    if (dt <= 0.0 || nodes.isEmpty) return;
    nodes[0].position = rootPosition;
    nodes[0].prevPosition = rootPosition;

    final damp = damping.clamp(0.0, 0.999).toDouble();
    final dtSq = dt * dt;

    for (var i = 1; i < nodes.length; i++) {
      final node = nodes[i];
      final parent = nodes[i - 1];
      final vel = (node.position - node.prevPosition) * damp;
      final restTarget = parent.position + node.restLocalDir * node.boneLength;
      final springForce = (restTarget - node.position) * stiffness;
      final accel = gravity + windForce + springForce;

      final nextPos = node.position + vel + accel * dtSq;
      node.prevPosition = node.position;
      node.position = nextPos;
    }

    // Distance constraints from root outward
    for (var iter = 0; iter < 3; iter++) {
      nodes[0].position = rootPosition;
      for (var i = 1; i < nodes.length; i++) {
        final parent = nodes[i - 1];
        final node = nodes[i];
        final dir = (node.position - parent.position).normalized();
        node.position = parent.position + dir * node.boneLength;
      }
    }
  }
}
