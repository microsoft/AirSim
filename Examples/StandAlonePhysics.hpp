#pragma once

#include <memory>
#include "common/SteppableClock.hpp"
#include "physics/FastPhysicsEngine.hpp"
#include "physics/DebugPhysicsBody.hpp"


class StandAlonePhysics {
public:
    static void testCollison()
    {
        using namespace msr::airlib;

        std::shared_ptr<SteppableClock> clock = std::make_shared<SteppableClock>();
        ClockFactory::get(clock);

        //init physics state
        auto initial_kinematics = Kinematics::State::zero();
        initial_kinematics.pose = Pose::zero();
        initial_kinematics.pose.position.z() = -1;
        msr::airlib::Environment::State initial_environment;
        initial_environment.position = initial_kinematics.pose.position;
        Environment environment(initial_environment);
        Kinematics kinematics(initial_kinematics);

        DebugPhysicsBody body;
        body.initialize(&kinematics, &environment);
        body.reset();

        //create physics engine
        FastPhysicsEngine physics;
        physics.insert(&body);
        physics.reset();

        //run
        unsigned int i = 0;
        while (true) {
            clock->step();

            environment.update();
            body.update();
            physics.update();
            ++i;

            CollisionInfo col;

            constexpr real_T ground_level = -0.8f;
            const auto& pos = body.getKinematics().pose.position;

            Quaternionr orientation = body.getKinematics().pose.orientation;
            Vector3r lowest_contact = pos + VectorMath::transformToWorldFrame(body.getShapeVertex(0), orientation);
            for (uint svi = 1; svi < body.shapeVertexCount(); ++svi) {
                Vector3r contact = pos + VectorMath::transformToWorldFrame(body.getShapeVertex(svi), orientation);
                if (lowest_contact.z() < contact.z())
                    lowest_contact = contact;
            }

            real_T penetration = lowest_contact.z() - ground_level;
            if (penetration >= 0) {
                col.has_collided = true;

                col.normal = Vector3r(0, 0, -1);
                Vector3r r = lowest_contact - pos;

                col.penetration_depth = penetration;
                col.position = pos;
                col.impact_point = lowest_contact;

                std::cout << "Col: " << VectorMath::toString(col.impact_point) << std::endl;
            }
            else {
                col.has_collided = false;
            }

            //col.has_collided = false;
            body.setCollisionInfo(col);
        }
    }
};