// WE DON'T USE THIS MODULE  
// BECAUSE I FAILED TO MAKE STITCH MODULE USING ORB FEATURE.
// BUT I TRIED :)
#include "matcher.hpp"

namespace bottom_up{ 
    int hammingDistance(const uchar code1, const uchar code2){
        int result = 0;
        int XOR = (code1 ^ code2);
        while(XOR >0){
            if(XOR%2 > 0)
                result++;
            XOR >>= 1;
        }
        return result;
    }

    multimap<int, pair<int,int> > bruteForceMatcher(const Mat & descriptor1, const Mat &descriptor2){
        assert(descriptor1.rows == descriptor2.rows && descriptor1.cols == descriptor2.cols);
        multimap<int, pair<int,int>> result_match;
        int num_features = descriptor1.rows;
        for(int i = 0; i < num_features; i++){
            for(int j = 0; j < num_features; j++){
                int distance=0;
                for(int bit_seq = 0; bit_seq < descriptor1.cols; bit_seq++){
                    uchar batch1 = descriptor1.at<uchar>(i,bit_seq);
                    uchar batch2 = descriptor2.at<uchar>(j,bit_seq);
                    distance += hammingDistance(batch1, batch2);
                }
                result_match.insert({distance, {i,j}});
            }
        }
        return result_match;
    }

    vector<FeatureMapping> orbFeatureMatch(const Mat & descriptor1, const Mat &descriptor2){
        assert(descriptor1.rows == descriptor2.rows && descriptor1.cols == descriptor2.cols);
        vector<FeatureMapping> result_match;
        int num_features = descriptor1.rows;
        for(int i = 0; i < num_features; i++){
            for(int j = 0; j < num_features; j++){
                int distance=0;
                for(int bit_seq = 0; bit_seq < descriptor1.cols; bit_seq++){
                    uchar batch1 = descriptor1.at<int>(i,bit_seq);
                    uchar batch2 = descriptor2.at<int>(j,bit_seq);
                    distance += hammingDistance(batch1, batch2);
                }
                result_match.push_back(FeatureMapping(i, j, distance));
            }
        }
        return result_match;
    }

}