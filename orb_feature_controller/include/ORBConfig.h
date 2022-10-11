#ifndef ORB_FEATURE_CONTROLLER_ORBCONFIG_H
#define ORB_FEATURE_CONTROLLER_ORBCONFIG_H

class ORBConfig{
public:
    int nFeatures;
    int normType;
    int distanceLimit;
    int goodMatchNum;

    ORBConfig(int nFeatures, int normType, int distanceLimit, int goodMatchNum)
    {
        this->nFeatures = nFeatures;
        this->normType = normType;
        this->distanceLimit = distanceLimit;
        this->goodMatchNum = goodMatchNum;
    }
};

#endif //ORB_FEATURE_CONTROLLER_ORBCONFIG_H