
#ifndef mathcer_bottom_up_implimentation 
    #define mathcer_bottom_up_implimentation
    #include <map>
    #include <cassert>
    #include <opencv2/core.hpp> // inside type,  Keypoints

using namespace std;
using namespace cv;

namespace bottom_up{ 
    int hammingDistance(const uchar code1, const uchar code2);
    

    struct FeatureMapping{
        FeatureMapping(int here, int there, double distance):here(here), there(there), distance(distance){;}
        int here, there;
        double distance;
    };


    multimap<int, pair<int,int> > bruteForceMatcher(const Mat & descriptor1, const Mat &descriptor2);
    vector<FeatureMapping> orbFeatureMatch(const Mat & descriptor1, const Mat &descriptor2);
    
    // TODO : FLANN matcher

}
#endif
