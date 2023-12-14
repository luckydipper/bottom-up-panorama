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
                    uchar batch1 = descriptor1.at<int>(i,bit_seq);
                    uchar batch2 = descriptor2.at<int>(j,bit_seq);
                    distance += hammingDistance(batch1, batch2);
                }
                result_match.insert({distance, {i,j}});
            }
        }
        return result_match;
    }


    // TODO : select good feature

	// double dMaxDist = matches[test_idx_l][test_idx_r][0].distance;
	// double dMinDist = matches[test_idx_l][test_idx_r][0].distance;
	// double dDistance;

	// // 두 개의 keypoint 사이에서 min-max를 계산한다 (min값만 사용)
	// for (int i = 0; i < descriptors[test_idx_l].rows; i++) {
	// 	dDistance = matches[test_idx_l][test_idx_r][i].distance;

	// 	if (dDistance < dMinDist) dMinDist = dDistance;
	// 	if (dDistance > dMaxDist) dMaxDist = dDistance;
	// }
	// printf("max_dist : %f \n", dMaxDist);
	// printf("min_dist : %f \n", dMinDist);

	// //match의 distance 값이 작을수록 matching이 잘 된 것
	// //min의 값의 3배 또는 good_matches.size() > 60 까지만 goodmatch로 인정해준다.
	// vector<DMatch>good_matches;
	// int distance = 3;
	// do {
	// 	vector<DMatch>good_matches2;
	// 	for (int i = 0; i < descriptors[test_idx_l].rows; i++) {
	// 		if (matches[test_idx_l][test_idx_r][i].distance < distance * dMinDist)
	// 			good_matches2.push_back(matches[test_idx_l][test_idx_r][i]);
	// 	}
	// 	good_matches = good_matches2;
	// 	distance -= 1;
	// } while (distance != 2 && good_matches.size() > 60);

    // Mat matGoodMatches;
    // drawMatches(imgs[test_idx_l], keypoints[test_idx_l], imgs[test_idx_r], keypoints[test_idx_r], good_matches, matGoodMatches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    // resize(matGoodMatches,matGoodMatches, Size(matGoodMatches.cols/4,matGoodMatches.rows/4));

    // imshow("-1", matGoodMatches );
    // waitKey();

    // cout << matches[test_idx_l][test_idx_r].size();
}