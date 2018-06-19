#ifndef CommonUtils_UniqueValueMap_hpp
#define CommonUtils_UniqueValueMap_hpp
#include <chrono>
#include "Utils.hpp"

#include <map>
#include <set>
#include "Utils.hpp"

namespace common_utils {

//This class allows to maintain unique set of values while 
//still allowing key to value maps
template <class TKey, class TVal>
class UniqueValueMap {
public:
    void insert(const TKey& key, const TVal& val)
    {
        map_.insert(key, val);
        vals_.insert(val);
    }
    void insert_or_assign(const TKey& key, const TVal& val)
    {
        map_[key] = val;
        vals_.insert(val);
    }

    const TVal& findOrDefault(const TKey& key, const TVal& default_val = TVal()) const
    {
        return Utils::findOrDefault(map_, key, default_val);
    }

    typename std::map<TKey, TVal>::const_iterator find(const TKey& key) const
    {
        return map_.find(key);
    }

    const std::map<TKey, TVal>& getMap() const
    {
        return map_;
    }

    typename std::set<TVal>::const_iterator begin() const
    {
        return vals_.begin();
    }

    typename std::set<TVal>::const_iterator end() const
    {
        return vals_.end();
    }

    const TVal& at(const TKey& key) const
    {
        return map_.at(key);
    }

    void clear()
    {
        map_.clear();
        vals_.clear();
    }

    size_t mapSize() const
    {
        return map_.size();
    }

    size_t valsSize() const
    {
        return vals_.size();
    }

    //TODO: add erase methods

private:
    std::map<TKey, TVal> map_;
    std::set<TVal> vals_;
};


}
#endif