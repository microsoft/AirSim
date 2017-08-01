// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_SensorCollection_hpp
#define msr_airlib_SensorCollection_hpp

#include <unordered_map>
#include "sensors/SensorBase.hpp"
#include "common/UpdatableContainer.hpp"
#include "common/Common.hpp"


namespace msr { namespace airlib {

class SensorCollection : UpdatableObject {
public: //types
    enum class SensorType : uint {
        Barometer = 1,
        Imu = 2,
        Gps = 3,
        Magnetometer = 4
    };
    typedef SensorBase* SensorBasePtr;
public:
    void insert(SensorBasePtr sensor, SensorType type)
    {
        auto type_int = static_cast<uint>(type);
        const auto& it = sensors_.find(type_int);
        if (it == sensors_.end()) {
            const auto& pair = sensors_.emplace(type_int, unique_ptr<SensorBaseContainer>(new SensorBaseContainer()));
            pair.first->second->insert(sensor);
        }
        else {
            it->second->insert(sensor);
        }
    }

    const SensorBase* getByType(SensorType type, uint index = 0) const
    {
        auto type_int = static_cast<uint>(type);
        const auto& it = sensors_.find(type_int);
        if (it == sensors_.end()) {
            return nullptr;
        }
        else {
            return it->second->at(index);
        }
    }

    uint size(SensorType type) const
    {
        auto type_int = static_cast<uint>(type);
        const auto& it = sensors_.find(type_int);
        if (it == sensors_.end()) {
            return 0;
        }
        else {
            return it->second->size();
        }
    }

    void initialize(const Kinematics::State* kinematics, const Environment* environment)
    {
        for (auto& pair : sensors_) {
            for (auto& sensor : *pair.second) {
                sensor->initialize(kinematics, environment);
            }
        }
    }


    //*** Start: UpdatableState implementation ***//
    virtual void reset() override
    {
        UpdatableObject::reset();

        for (auto& pair : sensors_) {
            pair.second->reset();
        }
    }

    virtual void update() override
    {
        UpdatableObject::update();

        for (auto& pair : sensors_) {
            pair.second->update();
        }
    }

    virtual void reportState(StateReporter& reporter) override
    {
        for (auto& pair : sensors_) {
            pair.second->reportState(reporter);
        }
    }
    //*** End: UpdatableState implementation ***//

private:
    typedef UpdatableContainer<SensorBasePtr> SensorBaseContainer;
    unordered_map<uint, unique_ptr<SensorBaseContainer>> sensors_;
};

}} //namespace
#endif
