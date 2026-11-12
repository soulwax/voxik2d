import 'package:test/test.dart';
import 'package:voxik2d/voxik2d.dart';

void main() {
  test('PlanarTwoBoneIkSolver reaches target accurately and respects bendSign polarity', () {
    final posBend = PlanarTwoBoneIkSolver.solve(
      root: IkVec2.zero,
      target: const IkVec2(1.4, 0.0),
      upperLength: 1.0,
      lowerLength: 1.0,
      bendSign: 1.0,
    );
    final negBend = PlanarTwoBoneIkSolver.solve(
      root: IkVec2.zero,
      target: const IkVec2(1.4, 0.0),
      upperLength: 1.0,
      lowerLength: 1.0,
      bendSign: -1.0,
    );

    expect((posBend.endEffector - const IkVec2(1.4, 0.0)).length, lessThan(1e-4));
    expect((posBend.midJoint - posBend.root).length, closeTo(1.0, 1e-4));
    expect((posBend.endEffector - posBend.midJoint).length, closeTo(1.0, 1e-4));
    expect(posBend.midJoint.y, greaterThan(0.5));
    expect(negBend.midJoint.y, lessThan(-0.5));
  });

  test('FabrikChainSolver converges multi-link chain onto reachable targets while preserving lengths', () {
    final joints = FabrikChainSolver.solve(
      root: IkVec2.zero,
      target: const IkVec2(2.2, 1.5),
      lengths: const [1.0, 1.0, 1.0],
      maxIterations: 20,
      tolerance: 1e-3,
    );

    expect(joints.length, 4);
    expect((joints.last - const IkVec2(2.2, 1.5)).length, lessThan(2e-3));
    for (var i = 0; i < 3; i++) {
      expect((joints[i + 1] - joints[i]).length, closeTo(1.0, 1e-3));
    }
  });

  test('SpringBoneChain integrates gravity/wind lag while preserving rigid bone segment lengths', () {
    final chain = SpringBoneChain.linear(
      root: IkVec2.zero,
      boneCount: 3,
      boneLength: 0.6,
      stiffness: 12.0,
      damping: 0.9,
      gravity: const IkVec2(0.0, -9.8),
    );

    for (var i = 0; i < 10; i++) {
      chain.step(0.02, rootPosition: IkVec2(i * 0.15, 0.0), windForce: const IkVec2(-4.0, 0.0));
    }

    expect(chain.nodes.last.position.y, lessThan(0.0));
    for (var i = 1; i < chain.nodes.length; i++) {
      expect(
        (chain.nodes[i].position - chain.nodes[i - 1].position).length,
        closeTo(0.6, 1e-4),
      );
    }
  });
}
