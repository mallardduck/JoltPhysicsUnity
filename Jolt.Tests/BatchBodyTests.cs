using NUnit.Framework;
using Unity.Mathematics;

namespace Jolt.Tests
{
    public class BatchBodyTests
    {
        private const int BodyCount = 16;

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
                ids[i] = bodies.CreateBody(settings).GetID();
            }

            return ids;
        }

        [Test]
        public void TestAddBodiesFinalizeThenRemoveBodies()
        {
            var ids = CreateBodies();

            var state = bodies.AddBodiesPrepare(ids);
            bodies.AddBodiesFinalize(ids, state, Activation.Activate);

            foreach (var id in ids)
            {
                Assert.IsTrue(bodies.IsAdded(id));
                Assert.IsTrue(bodies.IsActive(id));
            }

            bodies.DeactivateBodies(ids);

            foreach (var id in ids)
            {
                Assert.IsFalse(bodies.IsActive(id));
            }

            bodies.ActivateBodies(ids);

            foreach (var id in ids)
            {
                Assert.IsTrue(bodies.IsActive(id));
            }

            bodies.RemoveBodies(ids);

            foreach (var id in ids)
            {
                Assert.IsFalse(bodies.IsAdded(id));
            }
        }

        [Test]
        public void TestAddBodiesAbort()
        {
            var ids = CreateBodies();

            var state = bodies.AddBodiesPrepare(ids);
            bodies.AddBodiesAbort(ids, state);

            foreach (var id in ids)
            {
                Assert.IsFalse(bodies.IsAdded(id));
            }
        }
    }
}
