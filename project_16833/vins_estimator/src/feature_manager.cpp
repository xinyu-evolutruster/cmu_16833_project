#include "feature_manager.h"

Vector2d image_uv1;
int lineFeaturePerId::endFrame()
{
    return start_frame + linefeature_per_frame.size() - 1;
}

int FeaturePerId::endFrame()
{
    return start_frame + feature_per_frame.size() - 1;
}

FeatureManager::FeatureManager(Matrix3d _Rs[])
    : Rs(_Rs)
{
    for (int i = 0; i < NUM_OF_CAM; i++)
        ric[i].setIdentity();
}

void FeatureManager::setRic(Matrix3d _ric[])
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ric[i] = _ric[i];
    }
}

void FeatureManager::clearState()
{
    feature.clear();
}

int FeatureManager::getFeatureCount()
{
    int cnt = 0;
    for (auto &it : feature)
    {

        it.used_num = it.feature_per_frame.size();

        if (it.used_num >= 2 && it.start_frame < WINDOW_SIZE - 2)
        {
            cnt++;
        }
    }
    return cnt;
}

//                                                                      featureId      cameraId, point
bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Vector3d>>> &image)
{
    ROS_DEBUG("input feature: %d", (int)image.size());
    ROS_DEBUG("num of feature: %d", getFeatureCount());
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    for (auto &id_pts : image)
    {
        FeaturePerFrame f_per_fra(id_pts.second[0].second);

        int feature_id = id_pts.first;
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          { return it.feature_id == feature_id; });

        if (it == feature.end())
        {
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;
        }
    }

    if (frame_count < 2 || last_track_num < 20)
        return true;

    for (auto &it_per_id : feature)
    {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}

// bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Matrix<double, 5, 1>>>> &image, const map<int, vector<pair<int, Vector4d>>> &lines)
bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 5, 1>>>> &image, const map<int, vector<pair<int, Vector4d>>> &lines)
{
    ROS_DEBUG("input point feature: %d", (int)image.size());
    ROS_DEBUG("input line feature: %d", (int)lines.size());
    ROS_DEBUG("num of feature: %d", getFeatureCount());
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    for (auto &id_pts : image)
    {
        FeaturePerFrame f_per_fra(id_pts.second[0].second);

        int feature_id = id_pts.first;
        // std::cout<<"id: " << feature_id<<"\n";
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          { return it.feature_id == feature_id; });

        if (it == feature.end())
        {
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;
        }
    }

    for (auto &id_line : lines)
    {
        lineFeaturePerFrame f_per_fra(id_line.second[0].second);

        int feature_id = id_line.first;
        // cout << "line id: "<< feature_id << "\n";
        auto it = find_if(linefeature.begin(), linefeature.end(), [feature_id](const lineFeaturePerId &it)
                          { return it.feature_id == feature_id; });

        if (it == linefeature.end())
        {
            linefeature.push_back(lineFeaturePerId(feature_id, frame_count));
            linefeature.back().linefeature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id)
        {
            it->linefeature_per_frame.push_back(f_per_fra);
            it->all_obs_cnt++;
        }
    }

    if (frame_count < 2 || last_track_num < 20)
        return true;

    for (auto &it_per_id : feature)
    {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}
bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Vector3d>>> &image, const map<int, vector<pair<int, Vector4d>>> &lines)
{
    ROS_DEBUG("input point feature: %d", (int)image.size());
    ROS_DEBUG("input line feature: %d", (int)lines.size());
    ROS_DEBUG("num of feature: %d", getFeatureCount());
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    for (auto &id_pts : image)
    {
        FeaturePerFrame f_per_fra(id_pts.second[0].second);

        int feature_id = id_pts.first;
        // std::cout<<"id: " << feature_id<<"\n";
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          { return it.feature_id == feature_id; });

        if (it == feature.end())
        {
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;
        }
    }

    for (auto &id_line : lines)
    {
        lineFeaturePerFrame f_per_fra(id_line.second[0].second);

        int feature_id = id_line.first;
        // cout << "line id: "<< feature_id << "\n";
        auto it = find_if(linefeature.begin(), linefeature.end(), [feature_id](const lineFeaturePerId &it)
                          { return it.feature_id == feature_id; });

        if (it == linefeature.end())
        {
            linefeature.push_back(lineFeaturePerId(feature_id, frame_count));
            linefeature.back().linefeature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id)
        {
            it->linefeature_per_frame.push_back(f_per_fra);
            it->all_obs_cnt++;
        }
    }

    if (frame_count < 2 || last_track_num < 20)
        return true;

    for (auto &it_per_id : feature)
    {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}

bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Vector3d>>> &image, const map<int, vector<pair<int, Vector8d>>> &lines)
{
    ROS_DEBUG("input point feature: %d", (int)image.size());
    ROS_DEBUG("input line feature: %d", (int)lines.size());
    ROS_DEBUG("num of feature: %d", getFeatureCount());
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    for (auto &id_pts : image)
    {
        FeaturePerFrame f_per_fra(id_pts.second[0].second);

        int feature_id = id_pts.first;
        // std::cout<<"id: " << feature_id<<"\n";
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          { return it.feature_id == feature_id; });

        if (it == feature.end())
        {
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;
        }
    }

    for (auto &id_line : lines)
    {
        lineFeaturePerFrame f_per_fra(id_line.second[0].second);

        int feature_id = id_line.first;
        // cout << "line id: "<< feature_id << "\n";
        auto it = find_if(linefeature.begin(), linefeature.end(), [feature_id](const lineFeaturePerId &it)
                          { return it.feature_id == feature_id; });

        if (it == linefeature.end())
        {
            linefeature.push_back(lineFeaturePerId(feature_id, frame_count));
            linefeature.back().linefeature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id)
        {
            it->linefeature_per_frame.push_back(f_per_fra);
            it->all_obs_cnt++;
        }
    }

    if (frame_count < 2 || last_track_num < 20)
        return true;

    for (auto &it_per_id : feature)
    {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}

void FeatureManager::debugShow()
{
    ROS_DEBUG("debug show");
    for (auto &it : feature)
    {
        ROS_ASSERT(it.feature_per_frame.size() != 0);
        ROS_ASSERT(it.start_frame >= 0);
        ROS_ASSERT(it.used_num >= 0);

        ROS_DEBUG("%d,%d,%d ", it.feature_id, it.used_num, it.start_frame);
        int sum = 0;
        for (auto &j : it.feature_per_frame)
        {
            ROS_DEBUG("%d,", int(j.is_used));
            sum += j.is_used;
            printf("(%lf,%lf) ", j.point(0), j.point(1));
        }
        ROS_ASSERT(it.used_num == sum);
    }
}

vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : feature)
    {
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a = it.feature_per_frame[idx_l].point;
            b = it.feature_per_frame[idx_r].point;

            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}

void FeatureManager::setDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        // ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
    }
}

void FeatureManager::removeFailures()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        if (it->solve_flag == 2)
            feature.erase(it);
    }
}

void FeatureManager::clearDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth = 1.0 / x(++feature_index);
    }
}

VectorXd FeatureManager::getDepthVector()
{
    VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
#if 1
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
#else
        dep_vec(++feature_index) = it_per_id->estimated_depth;
#endif
    }
    return dep_vec;
}

int FeatureManager::getLineFeatureCount()
{
    int cnt = 0;
    for (auto &it : linefeature)
    {

        it.used_num = it.linefeature_per_frame.size();

        if (it.used_num >= LINE_MIN_OBS && it.start_frame < WINDOW_SIZE - 2 && it.is_triangulation)
        {
            cnt++;
        }
    }
    return cnt;
}

MatrixXd FeatureManager::getLineOrthVectorInCamera()
{
    MatrixXd lineorth_vec(getLineFeatureCount(), 4);
    int feature_index = -1;
    for (auto &it_per_id : linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))
            continue;

        lineorth_vec.row(++feature_index) = plk_to_orth(it_per_id.line_plucker);
    }
    return lineorth_vec;
}
void FeatureManager::setLineOrthInCamera(MatrixXd x)
{
    int feature_index = -1;
    for (auto &it_per_id : linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))
            continue;

        // std::cout<<"x:"<<x.rows() <<" "<<feature_index<<"\n";
        Vector4d line_orth = x.row(++feature_index);
        it_per_id.line_plucker = orth_to_plk(line_orth); // transfrom to camera frame

        // ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        /*
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
         */
    }
}
MatrixXd FeatureManager::getLineOrthVector(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    MatrixXd lineorth_vec(getLineFeatureCount(), 4);
    int feature_index = -1;
    for (auto &it_per_id : linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))
            continue;

        int imu_i = it_per_id.start_frame;

        ROS_ASSERT(NUM_OF_CAM == 1);

        Eigen::Vector3d twc = Ps[imu_i] + Rs[imu_i] * tic[0]; // twc = Rwi * tic + twi
        Eigen::Matrix3d Rwc = Rs[imu_i] * ric[0];             // Rwc = Rwi * Ric

        Vector6d line_w = plk_to_pose(it_per_id.line_plucker, Rwc, twc); // transfrom to world frame
        // line_w.normalize();
        lineorth_vec.row(++feature_index) = plk_to_orth(line_w);
        // lineorth_vec.row(++feature_index) = plk_to_orth(it_per_id.line_plucker);
    }
    return lineorth_vec;
}

void FeatureManager::setLineOrth(MatrixXd x, Vector3d P[], Matrix3d R[], Vector3d tic[], Matrix3d ric[])
{
    int feature_index = -1;
    for (auto &it_per_id : linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))
            continue;

        Vector4d line_orth_w = x.row(++feature_index);
        Vector6d line_w = orth_to_plk(line_orth_w);

        int imu_i = it_per_id.start_frame;
        ROS_ASSERT(NUM_OF_CAM == 1);

        Eigen::Vector3d twc = P[imu_i] + R[imu_i] * tic[0]; // twc = Rwi * tic + twi
        Eigen::Matrix3d Rwc = R[imu_i] * ric[0];            // Rwc = Rwi * Ric

        it_per_id.line_plucker = plk_from_pose(line_w, Rwc, twc); // transfrom to camera frame
        // it_per_id.line_plucker = line_w; // transfrom to camera frame

        // ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        /*
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
         */
    }
}

double FeatureManager::reprojection_error(Vector4d obs, Matrix3d Rwc, Vector3d twc, Vector6d line_w)
{

    double error = 0;

    Vector3d n_w, d_w;
    n_w = line_w.head(3);
    d_w = line_w.tail(3);

    Vector3d p1, p2;
    p1 << obs[0], obs[1], 1;
    p2 << obs[2], obs[3], 1;

    Vector6d line_c = plk_from_pose(line_w, Rwc, twc);
    Vector3d nc = line_c.head(3);
    double sql = nc.head(2).norm();
    nc /= sql;

    error += fabs(nc.dot(p1));
    error += fabs(nc.dot(p2));

    return error / 2.0;
}
//
void FeatureManager::triangulateLine(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    // std::cout<<"linefeature size: "<<linefeature.size()<<std::endl;
    for (auto &it_per_id : linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        if (it_per_id.is_triangulation)
            continue;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        ROS_ASSERT(NUM_OF_CAM == 1);

        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0]; // twc = Rwi * tic + twi
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];             // Rwc = Rwi * Ric

        double d = 0, min_cos_theta = 1.0;
        Eigen::Vector3d tij;
        Eigen::Matrix3d Rij;
        Eigen::Vector4d obsi, obsj; // obs from two frame are used to do triangulation

        // plane pi from ith obs in ith camera frame
        Eigen::Vector4d pii;
        Eigen::Vector3d ni; // normal vector of plane
        for (auto &it_per_frame : it_per_id.linefeature_per_frame)
        {
            imu_j++;

            if (imu_j == imu_i)
            {
                obsi = it_per_frame.lineobs;
                Eigen::Vector3d p1(obsi(0), obsi(1), 1);
                Eigen::Vector3d p2(obsi(2), obsi(3), 1);
                pii = pi_from_ppp(p1, p2, Vector3d(0, 0, 0));
                ni = pii.head(3);
                ni.normalize();
                continue;
            }

            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];

            Eigen::Vector3d t = R0.transpose() * (t1 - t0); // tij
            Eigen::Matrix3d R = R0.transpose() * R1;        // Rij

            Eigen::Vector4d obsj_tmp = it_per_frame.lineobs;

            // plane pi from jth obs in ith camera frame
            Vector3d p3(obsj_tmp(0), obsj_tmp(1), 1);
            Vector3d p4(obsj_tmp(2), obsj_tmp(3), 1);
            p3 = R * p3 + t;
            p4 = R * p4 + t;
            Vector4d pij = pi_from_ppp(p3, p4, t);
            Eigen::Vector3d nj = pij.head(3);
            nj.normalize();

            double cos_theta = ni.dot(nj);
            if (cos_theta < min_cos_theta)
            {
                min_cos_theta = cos_theta;
                tij = t;
                Rij = R;
                obsj = obsj_tmp;
                d = t.norm();
            }
        }

        // if the distance between two frame is lower than 0.1m or the parallax angle is lower than 15deg , do not triangulate.
        // if(d < 0.1 || min_cos_theta > 0.998)
        if (min_cos_theta > 0.998)
            // if( d < 0.2 )
            continue;

        // plane pi from jth obs in ith camera frame
        Vector3d p3(obsj(0), obsj(1), 1);
        Vector3d p4(obsj(2), obsj(3), 1);
        p3 = Rij * p3 + tij;
        p4 = Rij * p4 + tij;
        Vector4d pij = pi_from_ppp(p3, p4, tij);

        Vector6d plk = pipi_plk(pii, pij);
        Vector3d n = plk.head(3);
        Vector3d v = plk.tail(3);

        // Vector3d cp = plucker_origin( n, v );
        // if ( cp(2) < 0 )
        {
            //  cp = - cp;
            //  continue;
        }

        // Vector6d line;
        // line.head(3) = cp;
        // line.tail(3) = v;
        // it_per_id.line_plucker = line;

        // plk.normalize();
        it_per_id.line_plucker = plk; // plk in camera frame
        it_per_id.is_triangulation = true;

        //  used to debug
        Vector3d pc, nc, vc;
        nc = it_per_id.line_plucker.head(3);
        vc = it_per_id.line_plucker.tail(3);

        Matrix4d Lc;
        Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;

        Vector4d obs_startframe = it_per_id.linefeature_per_frame[0].lineobs;
        Vector3d p11 = Vector3d(obs_startframe(0), obs_startframe(1), 1.0);
        Vector3d p21 = Vector3d(obs_startframe(2), obs_startframe(3), 1.0);
        Vector2d ln = (p11.cross(p21)).head(2);
        ln = ln / ln.norm();

        Vector3d p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0);
        Vector3d p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
        Vector3d cam = Vector3d(0, 0, 0);

        Vector4d pi1 = pi_from_ppp(cam, p11, p12);
        Vector4d pi2 = pi_from_ppp(cam, p21, p22);

        Vector4d e1 = Lc * pi1;
        Vector4d e2 = Lc * pi2;
        e1 = e1 / e1(3);
        e2 = e2 / e2(3);

        Vector3d pts_1(e1(0), e1(1), e1(2));
        Vector3d pts_2(e2(0), e2(1), e2(2));

        Vector3d w_pts_1 = Rs[imu_i] * (ric[0] * pts_1 + tic[0]) + Ps[imu_i];
        Vector3d w_pts_2 = Rs[imu_i] * (ric[0] * pts_2 + tic[0]) + Ps[imu_i];
        it_per_id.ptw1 = w_pts_1;
        it_per_id.ptw2 = w_pts_2;

        // if(isnan(cp(0)))
        {

            // it_per_id.is_triangulation = false;

            // std::cout <<"------------"<<std::endl;
            // std::cout << line << "\n\n";
            // std::cout << d <<"\n\n";
            // std::cout << Rij <<std::endl;
            // std::cout << tij <<"\n\n";
            // std::cout <<"obsj: "<< obsj <<"\n\n";
            // std::cout << "p3: " << p3 <<"\n\n";
            // std::cout << "p4: " << p4 <<"\n\n";
            // std::cout <<pi_from_ppp(p3, p4,tij)<<std::endl;
            // std::cout << pij <<"\n\n";
        }
    }

    //    removeLineOutlier(Ps,tic,ric);
}

/**
 *  @brief  stereo line triangulate
 */
void FeatureManager::triangulateLine(double baseline)
{
    for (auto &it_per_id : linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();

        if (it_per_id.is_triangulation || it_per_id.used_num < 2)
            continue;

        int imu_i = it_per_id.start_frame;

        Vector4d lineobs_l, lineobs_r;
        lineFeaturePerFrame it_per_frame = it_per_id.linefeature_per_frame.front();
        lineobs_l = it_per_frame.lineobs;
        lineobs_r = it_per_frame.lineobs_R;

        // plane pi from ith left obs in ith left camera frame
        Vector3d p1(lineobs_l(0), lineobs_l(1), 1);
        Vector3d p2(lineobs_l(2), lineobs_l(3), 1);
        Vector4d pii = pi_from_ppp(p1, p2, Vector3d(0, 0, 0));

        // plane pi from ith right obs in ith left camera frame
        Vector3d p3(lineobs_r(0) + baseline, lineobs_r(1), 1);
        Vector3d p4(lineobs_r(2) + baseline, lineobs_r(3), 1);
        Vector4d pij = pi_from_ppp(p3, p4, Vector3d(baseline, 0, 0));

        Vector6d plk = pipi_plk(pii, pij);
        Vector3d n = plk.head(3);
        Vector3d v = plk.tail(3);

        // Vector3d cp = plucker_origin( n, v );
        // if ( cp(2) < 0 )
        {
            //  cp = - cp;
            //  continue;
        }

        // Vector6d line;
        // line.head(3) = cp;
        // line.tail(3) = v;
        // it_per_id.line_plucker = line;

        // plk.normalize();
        it_per_id.line_plucker = plk; // plk in camera frame
        it_per_id.is_triangulation = true;

        // if(isnan(cp(0)))
        {

            // it_per_id.is_triangulation = false;

            // std::cout <<"------------"<<std::endl;
            // std::cout << line << "\n\n";
            // std::cout << d <<"\n\n";
            // std::cout << Rij <<std::endl;
            // std::cout << tij <<"\n\n";
            // std::cout <<"obsj: "<< obsj <<"\n\n";
            // std::cout << "p3: " << p3 <<"\n\n";
            // std::cout << "p4: " << p4 <<"\n\n";
            // std::cout <<pi_from_ppp(p3, p4,tij)<<std::endl;
            // std::cout << pij <<"\n\n";
        }
    }

    removeLineOutlier();
}
//*/

void FeatureManager::triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        if (it_per_id.estimated_depth > 0)
            continue;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        ROS_ASSERT(NUM_OF_CAM == 1);
        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;

        Eigen::Matrix<double, 3, 4> P0;
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0]; // twc = Rwi * tic + twi
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];             // Rwc = Rwi * Ric
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;

            // std::cout<< "point tri: "<<imu_j<<std::endl;
            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
            Eigen::Vector3d t = R0.transpose() * (t1 - t0); // tij
            Eigen::Matrix3d R = R0.transpose() * R1;        // Rij
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;
            Eigen::Vector3d f = it_per_frame.point.normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i == imu_j)
                continue;
        }
        ROS_ASSERT(svd_idx == svd_A.rows());
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double svd_method = svd_V[2] / svd_V[3];
        // it_per_id->estimated_depth = -b / A;
        // it_per_id->estimated_depth = svd_V[2] / svd_V[3];

        it_per_id.estimated_depth = svd_method;
        // it_per_id->estimated_depth = INIT_DEPTH;

        if (it_per_id.estimated_depth < 0.1)
        {
            it_per_id.estimated_depth = INIT_DEPTH;
        }
    }
}

void FeatureManager::removeOutlier()
{
    ROS_BREAK();
    int i = -1;
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        i += it->used_num != 0;
        if (it->used_num != 0 && it->is_outlier == true)
        {
            feature.erase(it);
        }
    }
}
void FeatureManager::removeLineOutlier()
{

    for (auto it_per_id = linefeature.begin(), it_next = linefeature.begin();
         it_per_id != linefeature.end(); it_per_id = it_next)
    {
        it_next++;
        it_per_id->used_num = it_per_id->linefeature_per_frame.size();

        if (it_per_id->is_triangulation || it_per_id->used_num < 2)
            continue;

        int imu_i = it_per_id->start_frame, imu_j = imu_i - 1;

        Vector3d pc, nc, vc;
        nc = it_per_id->line_plucker.head(3);
        vc = it_per_id->line_plucker.tail(3);

        Matrix4d Lc;
        Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;

        Vector4d obs_startframe = it_per_id->linefeature_per_frame[0].lineobs;
        Vector3d p11 = Vector3d(obs_startframe(0), obs_startframe(1), 1.0);
        Vector3d p21 = Vector3d(obs_startframe(2), obs_startframe(3), 1.0);
        Vector2d ln = (p11.cross(p21)).head(2);
        ln = ln / ln.norm();

        Vector3d p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0);
        Vector3d p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
        Vector3d cam = Vector3d(0, 0, 0);

        Vector4d pi1 = pi_from_ppp(cam, p11, p12);
        Vector4d pi2 = pi_from_ppp(cam, p21, p22);

        Vector4d e1 = Lc * pi1;
        Vector4d e2 = Lc * pi2;
        e1 = e1 / e1(3);
        e2 = e2 / e2(3);

        if (e1(2) < 0 || e2(2) < 0)
        {
            linefeature.erase(it_per_id);
            continue;
        }

        if ((e1 - e2).norm() > 10)
        {
            linefeature.erase(it_per_id);
            continue;
        }
        /*

                Vector3d Q = plucker_origin(nc,vc);
                if(Q.norm() > 5.0)
                {
                    linefeature.erase(it_per_id);
                    continue;
                }
        */
    }
}

void FeatureManager::removeLineOutlier(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{

    for (auto it_per_id = linefeature.begin(), it_next = linefeature.begin();
         it_per_id != linefeature.end(); it_per_id = it_next)
    {
        it_next++;
        it_per_id->used_num = it_per_id->linefeature_per_frame.size();
        if (!(it_per_id->used_num >= LINE_MIN_OBS && it_per_id->start_frame < WINDOW_SIZE - 2 && it_per_id->is_triangulation))
            continue;

        int imu_i = it_per_id->start_frame, imu_j = imu_i - 1;

        ROS_ASSERT(NUM_OF_CAM == 1);

        Eigen::Vector3d twc = Ps[imu_i] + Rs[imu_i] * tic[0]; // twc = Rwi * tic + twi
        Eigen::Matrix3d Rwc = Rs[imu_i] * ric[0];             // Rwc = Rwi * Ric

        Vector3d pc, nc, vc;
        nc = it_per_id->line_plucker.head(3);
        vc = it_per_id->line_plucker.tail(3);

        //       double  d = nc.norm()/vc.norm();
        //       if (d > 5.0)
        {
            //           std::cerr <<"remove a large distant line \n";
            //           linefeature.erase(it_per_id);
            //           continue;
        }

        Matrix4d Lc;
        Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;

        Vector4d obs_startframe = it_per_id->linefeature_per_frame[0].lineobs;
        Vector3d p11 = Vector3d(obs_startframe(0), obs_startframe(1), 1.0);
        Vector3d p21 = Vector3d(obs_startframe(2), obs_startframe(3), 1.0);
        Vector2d ln = (p11.cross(p21)).head(2);
        ln = ln / ln.norm();

        Vector3d p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0);
        Vector3d p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
        Vector3d cam = Vector3d(0, 0, 0);

        Vector4d pi1 = pi_from_ppp(cam, p11, p12);
        Vector4d pi2 = pi_from_ppp(cam, p21, p22);

        Vector4d e1 = Lc * pi1;
        Vector4d e2 = Lc * pi2;
        e1 = e1 / e1(3);
        e2 = e2 / e2(3);

        // std::cout << "line endpoint: "<<e1 << "\n "<< e2<<"\n";
        if (e1(2) < 0 || e2(2) < 0)
        {
            linefeature.erase(it_per_id);
            continue;
        }
        if ((e1 - e2).norm() > 10)
        {
            linefeature.erase(it_per_id);
            continue;
        }

        /*

                Vector3d Q = plucker_origin(nc,vc);
                if(Q.norm() > 5.0)
                {
                    linefeature.erase(it_per_id);
                    continue;
                }
        */

        Vector6d line_w = plk_to_pose(it_per_id->line_plucker, Rwc, twc); // transfrom to world frame

        int i = 0;
        double allerr = 0;
        Eigen::Vector3d tij;
        Eigen::Matrix3d Rij;
        Eigen::Vector4d obs;

        // std::cout<<"reprojection_error: \n";
        for (auto &it_per_frame : it_per_id->linefeature_per_frame)
        {
            imu_j++;

            obs = it_per_frame.lineobs;
            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];

            double err = reprojection_error(obs, R1, t1, line_w);

            if (allerr < err)
                allerr = err;
        }
        //        allerr = allerr / i;
        if (allerr > 3.0 / 500.0)
        {
            //            std::cout<<"remove a large error\n";
            linefeature.erase(it_per_id);
        }
    }
}

void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
{
    std::cout << "removeBackShiftDepth\n";

    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() < 2)
            {
                feature.erase(it);
                continue;
            }
            else
            {
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                double dep_j = pts_j(2);
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else
                    it->estimated_depth = INIT_DEPTH;
            }
        }
        // remove tracking-lost feature after marginalize
        /*
        if (it->endFrame() < WINDOW_SIZE - 1)
        {
            feature.erase(it);
        }
        */
    }

    for (auto it = linefeature.begin(), it_next = linefeature.begin();
         it != linefeature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
        {
            it->start_frame--;
        }
        else
        {
            it->linefeature_per_frame.erase(it->linefeature_per_frame.begin());
            if (it->linefeature_per_frame.size() < 2)
            {
                linefeature.erase(it);
                continue;
            }
            else
            {
                it->removed_cnt++;
                // transpose this line to the new pose
                Matrix3d Rji = new_R.transpose() * marg_R; // Rcjw * Rwci
                Vector3d tji = new_R.transpose() * (marg_P - new_P);
                Vector6d plk_j = plk_to_pose(it->line_plucker, Rji, tji);
                it->line_plucker = plk_j;
            }
        }
    }
}

void FeatureManager::removeBack()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }

    std::cout << "remove back" << std::endl;
    for (auto it = linefeature.begin(), it_next = linefeature.begin();
         it != linefeature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->linefeature_per_frame.erase(it->linefeature_per_frame.begin());
            if (it->linefeature_per_frame.size() == 0)
                linefeature.erase(it);
        }
    }
}

void FeatureManager::removeFront(int frame_count)
{
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count)
        {
            it->start_frame--;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;
            if (it->endFrame() < frame_count - 1)
                continue;
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }

    std::cout << "remove front \n";
    for (auto it = linefeature.begin(), it_next = linefeature.begin(); it != linefeature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count)
        {
            it->start_frame--;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;
            if (it->endFrame() < frame_count - 1)
                continue;
            it->linefeature_per_frame.erase(it->linefeature_per_frame.begin() + j);
            if (it->linefeature_per_frame.size() == 0)
                linefeature.erase(it);
        }
    }
}

double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count)
{
    // check the second last frame is keyframe or not
    // parallax betwwen seconde last frame and third last frame

    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    Vector3d p_j = frame_j.point;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.point;
    Vector3d p_i_comp;

    // int r_i = frame_count - 2;
    // int r_j = frame_count - 1;
    // p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}