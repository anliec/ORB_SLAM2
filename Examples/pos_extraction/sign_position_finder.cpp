#include "sign_position_finder.h"

#include <map>

struct SignRay
{
    cv::Mat p, v;
    SignRay(double x, double y, double z, double x2, double y2, double z2)
    {
        cv::Vec3d pVec(x,y,z);
        cv::Vec3d p2Vec(x2,y2,z2);
        p = cv::Mat(pVec);
        v = cv::Mat(p2Vec - pVec);
        v /= cv::norm(v);
    }
};

std::vector<SignCoordinate> findSignCoordinates(const std::vector<SignPointer> &rays, const std::vector<cv::Mat> &poses)
{
    std::map<int, std::vector<SignRay>> bySignRays;
    for(const SignPointer &p : rays){
        const cv::Mat &pose = poses[p.frameNumber];
        if(pose.empty()){
            continue;
        }
        bySignRays[p.signId].emplace_back(pose.at<float>(0, 3), pose.at<float>(1, 3), pose.at<float>(2, 3), p.v[0], p.v[1], p.v[2]);
    }

    std::vector<SignCoordinate> signPos;

    for(const std::pair<int, std::vector<SignRay>> pair : bySignRays){
        const std::vector<SignRay> &signRays = pair.second;
        const double &signId = pair.first;

        cv::Mat a = cv::Mat::zeros(cv::Size(3,3), CV_64F);
        cv::Mat b = cv::Mat::zeros(cv::Size(1,3), CV_64F);

        // formula from: https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#In_three_dimensions_2
        // compute both sum matrix
        for(auto ray = signRays.begin() ; ray!=signRays.end() ; ++ray){
            cv::Mat t = cv::Mat::eye(cv::Size(3,3), CV_64F) - (ray->v * ray->v.t());
            a += t;
            b += t * ray->p;
        }

        cv::Mat x = a.inv() * b;

        signPos.emplace_back(signId, x, signRays.size());
    }

    return signPos;
}
