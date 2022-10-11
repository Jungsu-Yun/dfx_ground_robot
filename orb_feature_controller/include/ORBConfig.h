#ifndef ORB_FEATURE_CONTROLLER_ORBCONFIG_H
#define ORB_FEATURE_CONTROLLER_ORBCONFIG_H

class ORBConfig{
public:
    int nFeatures;
    int normType;
    double distanceLimit;
    int goodMatchNum;
    double error_limit;

    ORBConfig(int nFeatures, int normType, int distanceLimit, int goodMatchNum, double error_limit)
    {
        this->nFeatures = nFeatures;
        this->normType = normType;
        this->distanceLimit = distanceLimit;
        this->goodMatchNum = goodMatchNum;
        this->error_limit = error_limit;
    }
};

#endif //ORB_FEATURE_CONTROLLER_ORBCONFIG_H