
namespace AirSimUnity
{
    /**
     * An interface to Unity client into AirLib wrapper.
     */
    public interface IAirSimInterface
    {
        bool StartVehicleServer(string hostIP);

        void StopVehicleServer();

        KinemticState GetKinematicState();

        void InvokeTickInAirSim(float deltaSecond);

        void InvokeCollisionDetectionInAirSim(CollisionInfo collisionInfo);
    }
}
