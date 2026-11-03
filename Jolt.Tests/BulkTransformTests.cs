using System;
using NUnit.Framework;
using Unity.Mathematics;

namespace Jolt.Tests
{
    public class BulkTransformTests
    {
        private const int BodyCount = 8;

        private PhysicsSystem system;
        private BodyInterface bodies;

        [SetUp]
        public void SetUp()
        {
            Assert.IsTrue(Jolt.Initialize());

            var objectLayerPairFilter = ObjectLayerPairFilterTable.Create(1);
            objectLayerPairFilter.EnableCollision(0, 0);

            var broadPhaseLayerInterface = BroadPhaseLayerInterfaceTable.Create(1, 1);
            broadPhaseLayerInterface.MapObjectToBroadPhaseLayer(0, 0);

            var objectVsBroadPhaseLayerFilter = ObjectVsBroadPhaseLayerFilterTable.Create(broadPhaseLayerInterface, 1, objectLayerPairFilter, 1);

            system = new PhysicsSystem(new PhysicsSystemSettings
            {
                MaxBodies = 64,
                MaxBodyPairs = 64,
                MaxContactConstraints = 64,
                ObjectLayerPairFilter = objectLayerPairFilter,
                BroadPhaseLayerInterface = broadPhaseLayerInterface,
                ObjectVsBroadPhaseLayerFilter = objectVsBroadPhaseLayerFilter,
            });

            bodies = system.GetBodyInterface();
        }

        [TearDown]
        public void TearDown()
        {
            system.Destroy();
        }

        private BodyID[] CreateBodies()
        {
            var ids = new BodyID[BodyCount];

            for (var i = 0; i < ids.Length; i++)
            {
                var settings = BodyCreationSettings.Create(BoxShapeSettings.Create(new float3(0.5f)), new float3(i * 2, 0, 0), quaternion.identity, MotionType.Dynamic, 0);
                ids[i] = bodies.CreateAndAddBody(settings, Activation.DontActivate);
            }

            return ids;
        }

        [Test]
        public void TestGetPositionsAndRotationsMatchesSingleCalls()
        {
            var ids = CreateBodies();
            var positions = new rvec3[BodyCount];
            var rotations = new quaternion[BodyCount];

            bodies.GetPositionsAndRotations(ids, positions, rotations);

            for (var i = 0; i < ids.Length; i++)
            {
                bodies.GetPositionAndRotation(ids[i], out var position, out var rotation);

                Assert.AreEqual(position, positions[i]);
                Assert.AreEqual(rotation.value, rotations[i].value);
            }
        }

        [Test]
        public void TestSetPositionsAndRotationsRoundTrip()
        {
            var ids = CreateBodies();
            var positions = new rvec3[BodyCount];
            var rotations = new quaternion[BodyCount];

            for (var i = 0; i < BodyCount; i++)
            {
                positions[i] = new float3(i, i * 3, -i);
                rotations[i] = quaternion.RotateY(i * 0.1f);
            }

            bodies.SetPositionsAndRotations(ids, positions, rotations, Activation.DontActivate);

            var readPositions = new rvec3[BodyCount];
            var readRotations = new quaternion[BodyCount];

            bodies.GetPositionsAndRotations(ids, readPositions, readRotations);

            for (var i = 0; i < BodyCount; i++)
            {
                Assert.AreEqual(positions[i], readPositions[i]);
                Assert.IsTrue(math.all(math.abs(rotations[i].value - readRotations[i].value) < 1e-5f));
            }
        }

        [Test]
        public void TestShortOutputSpanThrows()
        {
            var ids = CreateBodies();

            Assert.Throws<ArgumentException>(() => bodies.GetPositionsAndRotations(ids, new rvec3[BodyCount - 1], new quaternion[BodyCount]));
            Assert.Throws<ArgumentException>(() => bodies.GetPositionsAndRotations(ids, new rvec3[BodyCount], new quaternion[BodyCount - 1]));
        }
    }
}
