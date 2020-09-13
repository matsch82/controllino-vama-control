
#ifndef MAPPING_HPP
#define MAPPING_HPP

#include <limits.h>

/**
 * Helper class to map non linear poti values to linear ones.
 * 
 * performs linear interpolation between a array of support points.
 * 
 */

typedef struct {
    int16_t voltage;
    int16_t poti;
} mapentry_t;

using MappingEntry = mapentry_t;
using Mapping = MappingEntry[11];

class PotiMapping {
public:
    PotiMapping(const Mapping* mapping, const int size): mMappingSize(size), mMapping(mapping){
        findMinMax();
    }

    int getMappedValue(int potiValue) {
        // ensure that mMinPotiValue <= potiValue =< mMaxPotiValue
        potiValue = clamp(potiValue, mMinVoltageValue, mMaxVoltageValue);

        // find intervall to interpolate on:
        const MappingEntry* firstBigger = findFirstBigger(potiValue);
        if(firstBigger == &(*mMapping)[0]){
            return (*mMapping)[0].poti;
        }

        const MappingEntry* lastSmaller = firstBigger - 1;
        //
        auto result = linearInterpolate(potiValue, lastSmaller->voltage, firstBigger->voltage, lastSmaller->poti, firstBigger->poti);
        result = clamp(result, mMinPotiValue, mMaxPotiValue);
        return result;
    }
private:

    int linearInterpolate(int x, int in_min, int in_max, int out_min, int out_max) {
        auto result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        return result;
    }

    int clamp(int n, int lower, int upper) {
        return n <= lower ? lower : n >= upper ? upper : n;
    }

    int findMinMax(){
        mMinVoltageValue=INT_MAX;
        mMaxVoltageValue=INT_MIN;
        for (int i = 0; i < mMappingSize; ++i) {
            auto val = (*mMapping)[i].voltage;
            if(val < mMinVoltageValue) mMinVoltageValue = val;
            if(mMaxVoltageValue < val) mMaxVoltageValue = val;

            val = (*mMapping)[i].poti;
            if(val < mMinPotiValue) mMinPotiValue = val;
            if(mMaxPotiValue < val) mMaxPotiValue = val;
        }
    }

    const MappingEntry* findFirstBigger(int val){
        for (int i = 0; i < mMappingSize; ++i) {
            auto cur = (*mMapping)[i].voltage;
            if(val < cur) {
                return &(*mMapping)[i];
            }
        }
        return &((*mMapping)[mMappingSize]);
    }

    const Mapping* mMapping;
    int mMappingSize=0;
    int mMinVoltageValue=0;
    int mMaxVoltageValue=0;
    int mMinPotiValue=0;
    int mMaxPotiValue=0;
};

#endif // MAPPING_HPP
