// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_PhysicsEngineBase_hpp
#define airsim_core_PhysicsEngineBase_hpp

#include "common/UpdatableContainer.hpp"
#include "common/Common.hpp"
#include "PhysicsBody.hpp"

namespace msr { namespace airlib {

class PhysicsEngineBase : public UpdatableObject {
public:
    virtual void update() override
    {
        UpdatableObject::update();
    }

    virtual void reportState(StateReporter& reporter) override
    {
        unused(reporter);
        //default nothing to report for physics engine
    }

    //TODO: reduce copy-past from UpdatableContainer which has same code
    /********************** Container interface **********************/
    typedef PhysicsBody* TUpdatableObjectPtr;
    typedef vector<TUpdatableObjectPtr> MembersContainer;
    typedef typename MembersContainer::iterator iterator;
    typedef typename MembersContainer::const_iterator const_iterator;
    typedef typename MembersContainer::value_type value_type;

    iterator begin() { return members_.begin(); }
    iterator end() { return members_.end(); }
    const_iterator begin() const { return members_.begin(); }
    const_iterator end() const { return members_.end(); }
    uint size() const { return static_cast<uint>(members_.size()); }
    const TUpdatableObjectPtr &at(uint index) const { return members_.at(index);  }
    TUpdatableObjectPtr &at(uint index) { return members_.at(index);  }
    //allow to override membership modifications
    virtual void clear() { members_.clear(); }
    virtual void insert(TUpdatableObjectPtr member) { members_.push_back(member);  }
    virtual void erase_remove(TUpdatableObjectPtr obj) { 
        members_.erase(std::remove(members_.begin(), members_.end(), obj), members_.end()); }

private:
    MembersContainer members_;
};


}} //namespace
#endif
