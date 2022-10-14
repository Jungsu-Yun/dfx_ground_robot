#ifndef ORB_FEATURE_CONTROLLER_ORBCONFIG_H
#define ORB_FEATURE_CONTROLLER_ORBCONFIG_H

class ORBConfig{
public:
    int nFeatures;
    int normType;
    double distanceLimit;
    int goodMatchNum;
    double error_limit;
    int nGoodFollowed;

    ORBConfig(int nFeatures, int normType, int distanceLimit, int goodMatchNum, double error_limit, int nGoodFollowed)
    {
        this->nFeatures = nFeatures;
        this->normType = normType;
        this->distanceLimit = distanceLimit;
        this->goodMatchNum = goodMatchNum;
        this->error_limit = error_limit;
        this->nGoodFollowed = nGoodFollowed;
    }
};

#endif //ORB_FEATURE_CONTROLLER_ORBCONFIG_H