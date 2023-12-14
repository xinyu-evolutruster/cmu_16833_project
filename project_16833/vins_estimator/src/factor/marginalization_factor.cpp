#include "marginalization_factor.h"

void ResidualBlockInfo::Evaluate()
{
    residuals.resize(cost_function->num_residuals());

    std::vector<int> block_sizes = cost_function->parameter_block_sizes();
    raw_jacobians = new double *[block_sizes.size()];
    jacobians.resize(block_sizes.size());

    for (int i = 0; i < static_cast<int>(block_sizes.size()); i++)
    {
        jacobians[i].resize(cost_function->num_residuals(), block_sizes[i]);
        raw_jacobians[i] = jacobians[i].data();
        // dim += block_sizes[i] == 7 ? 6 : block_sizes[i];
    }
    cost_function->Evaluate(parameter_blocks.data(), residuals.data(), raw_jacobians);

    // std::vector<int> tmp_idx(block_sizes.size());
    // Eigen::MatrixXd tmp(dim, dim);
    // for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++)
    //{
    //     int size_i = localSize(block_sizes[i]);
    //     Eigen::MatrixXd jacobian_i = jacobians[i].leftCols(size_i);
    //     for (int j = 0, sub_idx = 0; j < static_cast<int>(parameter_blocks.size()); sub_idx += block_sizes[j] == 7 ? 6 : block_sizes[j], j++)
    //     {
    //         int size_j = localSize(block_sizes[j]);
    //         Eigen::MatrixXd jacobian_j = jacobians[j].leftCols(size_j);
    //         tmp_idx[j] = sub_idx;
    //         tmp.block(tmp_idx[i], tmp_idx[j], size_i, size_j) = jacobian_i.transpose() * jacobian_j;
    //     }
    // }
    // Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(tmp);
    // std::cout << saes.eigenvalues() << std::endl;
    // ROS_ASSERT(saes.eigenvalues().minCoeff() >= -1e-6);

    if (loss_function)
    {
        double residual_scaling_, alpha_sq_norm_;

        double sq_norm, rho[3];

        sq_norm = residuals.squaredNorm();
        loss_function->Evaluate(sq_norm, rho);
        // printf("sq_norm: %f, rho[0]: %f, rho[1]: %f, rho[2]: %f\n", sq_norm, rho[0], rho[1], rho[2]);

        double sqrt_rho1_ = sqrt(rho[1]);

        if ((sq_norm == 0.0) || (rho[2] <= 0.0))
        {
            residual_scaling_ = sqrt_rho1_;
            alpha_sq_norm_ = 0.0;
        }
        else
        {
            const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
            const double alpha = 1.0 - sqrt(D);
            residual_scaling_ = sqrt_rho1_ / (1 - alpha);
            alpha_sq_norm_ = alpha / sq_norm;
        }

        for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++)
        {
            jacobians[i] = sqrt_rho1_ * (jacobians[i] - alpha_sq_norm_ * residuals * (residuals.transpose() * jacobians[i]));
        }

        residuals *= residual_scaling_;
    }
}

MarginalizationInfo::~MarginalizationInfo()
{
    // ROS_WARN("release marginlizationinfo");

    for (auto it = parameter_block_data.begin(); it != parameter_block_data.end(); ++it)
        delete it->second;

    for (int i = 0; i < (int)factors.size(); i++)
    {

        delete[] factors[i]->raw_jacobians;

        delete factors[i]->cost_function;

        delete factors[i];
    }
}

void MarginalizationInfo::addResidualBlockInfo(ResidualBlockInfo *residual_block_info)
{
    factors.emplace_back(residual_block_info);

    std::vector<double *> &parameter_blocks = residual_block_info->parameter_blocks;
    std::vector<int> parameter_block_sizes = residual_block_info->cost_function->parameter_block_sizes();

    for (int i = 0; i < static_cast<int>(residual_block_info->parameter_blocks.size()); i++)
    {
        double *addr = parameter_blocks[i];
        int size = parameter_block_sizes[i];
        parameter_block_size[reinterpret_cast<long>(addr)] = size;
    }

    for (int i = 0; i < static_cast<int>(residual_block_info->drop_set.size()); i++)
    {
        double *addr = parameter_blocks[residual_block_info->drop_set[i]];
        parameter_block_idx[reinterpret_cast<long>(addr)] = 0;
    }
}

void MarginalizationInfo::preMarginalize()
{
    for (auto it : factors)
    {
        it->Evaluate();

        std::vector<int> block_sizes = it->cost_function->parameter_block_sizes();
        for (int i = 0; i < static_cast<int>(block_sizes.size()); i++)
        {
            long addr = reinterpret_cast<long>(it->parameter_blocks[i]);
            int size = block_sizes[i];
            if (parameter_block_data.find(addr) == parameter_block_data.end())
            {
                double *data = new double[size];
                memcpy(data, it->parameter_blocks[i], sizeof(double) * size);
                parameter_block_data[addr] = data;
            }
        }
    }
}

int MarginalizationInfo::localSize(int size) const
{
    return size == 7 ? 6 : size;
}

int MarginalizationInfo::globalSize(int size) const
{
    return size == 6 ? 7 : size;
}

void *ThreadsConstructA(void *threadsstruct)
{
    ThreadsStruct *p = ((ThreadsStruct *)threadsstruct);
    for (auto it : p->sub_factors)
    {
        for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++)
        {
            int idx_i = p->parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[i])];
            int size_i = p->parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])];
            if (size_i == 7)
                size_i = 6;
            Eigen::MatrixXd jacobian_i = it->jacobians[i].leftCols(size_i);
            for (int j = i; j < static_cast<int>(it->parameter_blocks.size()); j++)
            {
                int idx_j = p->parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[j])];
                int size_j = p->parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[j])];
                if (size_j == 7)
                    size_j = 6;
                Eigen::MatrixXd jacobian_j = it->jacobians[j].leftCols(size_j);
                if (i == j)
                    p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                else
                {
                    p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                    p->A.block(idx_j, idx_i, size_j, size_i) = p->A.block(idx_i, idx_j, size_i, size_j).transpose();
                }
            }
            p->b.segment(idx_i, size_i) += jacobian_i.transpose() * it->residuals;
        }
    }
    return threadsstruct;
}

void MarginalizationInfo::marginalize()
{
    int pos = 0;
    int marg_pose_size = 0;
    std::map<int, int, std::greater<int>> marg_pose_index_size;
    for (auto &it : parameter_block_idx)
    {

        if (localSize(parameter_block_size[it.first]) > 4) // pose size is 6, velocity and bias size is 9
        {
            marg_pose_index_size.insert(std::make_pair(pos, localSize(parameter_block_size[it.first])));
            marg_pose_size += localSize(parameter_block_size[it.first]);
        }

        it.second = pos;
        pos += localSize(parameter_block_size[it.first]);
    }

    m = pos;

    for (const auto &it : parameter_block_size)
    {

        if (parameter_block_idx.find(it.first) == parameter_block_idx.end())
        {
            parameter_block_idx[it.first] = pos;
            pos += localSize(it.second);
        }
    }

    n = pos - m;

    ROS_INFO("marginalization, pos: %d, m: %d, n: %d, size: %d", pos, m, n, (int)parameter_block_idx.size());

    TicToc t_summing;
    Eigen::MatrixXd A(pos, pos);
    Eigen::VectorXd b(pos);
    A.setZero();
    b.setZero();

    // multi thread
    TicToc t_thread_summing;
    pthread_t tids[NUM_THREADS];
    ThreadsStruct threadsstruct[NUM_THREADS];
    int i = 0;
    for (auto it : factors)
    {
        threadsstruct[i].sub_factors.push_back(it);
        i++;
        i = i % NUM_THREADS;
    }
    for (int i = 0; i < NUM_THREADS; i++)
    {
        TicToc zero_matrix;
        threadsstruct[i].A = Eigen::MatrixXd::Zero(pos, pos);
        threadsstruct[i].b = Eigen::VectorXd::Zero(pos);
        threadsstruct[i].parameter_block_size = parameter_block_size;
        threadsstruct[i].parameter_block_idx = parameter_block_idx;
        int ret = pthread_create(&tids[i], NULL, ThreadsConstructA, (void *)&(threadsstruct[i]));
        if (ret != 0)
        {
            ROS_WARN("pthread_create error");
            ROS_BREAK();
        }
    }
    for (int i = NUM_THREADS - 1; i >= 0; i--)
    {
        pthread_join(tids[i], NULL);
        A += threadsstruct[i].A;
        b += threadsstruct[i].b;
    }
    // ROS_DEBUG("thread summing up costs %f ms", t_thread_summing.toc());
    // ROS_INFO("A diff %f , b diff %f ", (A - tmp_A).sum(), (b - tmp_b).sum());

    ///////////////// marg accletor code/////////////////////////
    // marg landmark firstly
    // TicToc tmarg_l;
    int m1 = m - marg_pose_size;
    int n1 = n + marg_pose_size;
    if (m1 > 0)
    {
        Eigen::MatrixXd A_reorder(A);
        Eigen::VectorXd b_reorder(b);

        for (auto iter : marg_pose_index_size)
        {
            int idx = iter.first;
            int size = iter.second;

            Eigen::MatrixXd temp_rows = A_reorder.block(idx, 0, size, pos);
            A_reorder.block(idx, 0, m - idx - size, pos) = A_reorder.block(idx + size, 0, m - idx - size, pos);
            A_reorder.block(m - size, 0, size, pos) = temp_rows;

            Eigen::MatrixXd temp_cols = A_reorder.block(0, idx, pos, size);
            A_reorder.block(0, idx, pos, m - idx - size) = A_reorder.block(0, idx + size, pos, m - idx - size);
            A_reorder.block(0, m - size, pos, size) = temp_cols;

            Eigen::VectorXd temp_b = b_reorder.segment(idx, size);
            b_reorder.segment(idx, m - idx - size) = b_reorder.segment(idx + size, m - idx - size);
            b_reorder.segment(m - size, size) = temp_b;
        }

        Eigen::MatrixXd Amm1 = A_reorder.block(0, 0, m1, m1);
        Eigen::VectorXd bmm1 = b_reorder.segment(0, m1);

        // TicToc t_inv;
        // Eigen::MatrixXd Amm_inv1 = Amm1.diagonal().asDiagonal().inverse();    // when only use point feature
        Eigen::MatrixXd Amm_inv1 = Amm1.inverse();
        // ROS_INFO("invese Amm costs %f ms", tmarg_l.toc());

        Eigen::MatrixXd Amr1 = A_reorder.block(0, m1, m1, n1);
        Eigen::MatrixXd Arm1 = A_reorder.block(m1, 0, n1, m1);
        Eigen::MatrixXd Arr1 = A_reorder.block(m1, m1, n1, n1);
        Eigen::VectorXd brr1 = b_reorder.segment(m1, n1);
        // TicToc t_mult;
        Eigen::MatrixXd tempA = Arm1 * Amm_inv1;
        A = Arr1 - tempA * Amr1;
        b = brr1 - tempA * bmm1;
    }

    // then marg pose
    // TicToc tmarg_pose;
    int m2 = m - m1;
    int n2 = n;
    Eigen::MatrixXd Amm = 0.5 * (A.block(0, 0, m2, m2) + A.block(0, 0, m2, m2).transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Amm);
    Eigen::MatrixXd Amm_inv = saes.eigenvectors() * Eigen::VectorXd((saes.eigenvalues().array() > eps).select(saes.eigenvalues().array().inverse(), 0)).asDiagonal() * saes.eigenvectors().transpose();
    // printf("error1: %f\n", (Amm * Amm_inv - Eigen::MatrixXd::Identity(m, m)).sum());

    Eigen::VectorXd bmm = b.segment(0, m2);
    Eigen::MatrixXd Amr = A.block(0, m2, m2, n2);
    Eigen::MatrixXd Arm = A.block(m2, 0, n2, m2);
    Eigen::MatrixXd Arr = A.block(m2, m2, n2, n2);
    Eigen::VectorXd brr = b.segment(m2, n2);
    Eigen::MatrixXd tempB = Arm * Amm_inv;
    A = Arr - tempB * Amr;
    b = brr - tempB * bmm;
    // ROS_INFO("marg pose costs %f ms", tmarg_pose.toc());
    /////////////////////////////////////////////////////////////

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(A);
    Eigen::VectorXd S = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array(), 0));
    Eigen::VectorXd S_inv = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array().inverse(), 0));

    Eigen::VectorXd S_sqrt = S.cwiseSqrt();
    Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();

    linearized_jacobians = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
    linearized_residuals = S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose() * b;
    // std::cout << A << std::endl
    //           << std::endl;
    // std::cout << linearized_jacobians << std::endl;
    // printf("error2: %f %f\n", (linearized_jacobians.transpose() * linearized_jacobians - A).sum(),
    //       (linearized_jacobians.transpose() * linearized_residuals - b).sum());
}

std::vector<double *> MarginalizationInfo::getParameterBlocks(std::unordered_map<long, double *> &addr_shift)
{
    std::vector<double *> keep_block_addr;
    keep_block_size.clear();
    keep_block_idx.clear();
    keep_block_data.clear();

    for (const auto &it : parameter_block_idx)
    {
        if (it.second >= m)
        {
            keep_block_size.push_back(parameter_block_size[it.first]);
            keep_block_idx.push_back(parameter_block_idx[it.first]);
            keep_block_data.push_back(parameter_block_data[it.first]);
            keep_block_addr.push_back(addr_shift[it.first]);
        }
    }
    sum_block_size = std::accumulate(std::begin(keep_block_size), std::end(keep_block_size), 0);

    return keep_block_addr;
}

MarginalizationFactor::MarginalizationFactor(MarginalizationInfo *_marginalization_info) : marginalization_info(_marginalization_info)
{
    int cnt = 0;
    for (auto it : marginalization_info->keep_block_size)
    {
        mutable_parameter_block_sizes()->push_back(it);
        cnt += it;
    }
    // printf("residual size: %d, %d\n", cnt, n);
    set_num_residuals(marginalization_info->n);
};

bool MarginalizationFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    // printf("internal addr,%d, %d\n", (int)parameter_block_sizes().size(), num_residuals());
    // for (int i = 0; i < static_cast<int>(keep_block_size.size()); i++)
    //{
    //     //printf("unsigned %x\n", reinterpret_cast<unsigned long>(parameters[i]));
    //     //printf("signed %x\n", reinterpret_cast<long>(parameters[i]));
    // printf("jacobian %x\n", reinterpret_cast<long>(jacobians));
    // printf("residual %x\n", reinterpret_cast<long>(residuals));
    // }
    int n = marginalization_info->n;
    int m = marginalization_info->m;
    Eigen::VectorXd dx(n);
    for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); i++)
    {
        int size = marginalization_info->keep_block_size[i];
        int idx = marginalization_info->keep_block_idx[i] - m;
        Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(parameters[i], size);
        Eigen::VectorXd x0 = Eigen::Map<const Eigen::VectorXd>(marginalization_info->keep_block_data[i], size);
        if (size != 7)
            dx.segment(idx, size) = x - x0; // bias , velocity etc.
        else
        {
            dx.segment<3>(idx + 0) = x.head<3>() - x0.head<3>();
            dx.segment<3>(idx + 3) = 2.0 * Utility::positify(Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).vec();
            if (!((Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).w() >= 0))
            {
                dx.segment<3>(idx + 3) = 2.0 * -Utility::positify(Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).vec();
            }
        }
    }

    Eigen::Map<Eigen::VectorXd>(residuals, n) = marginalization_info->linearized_residuals + marginalization_info->linearized_jacobians * dx;
    if (jacobians)
    {

        for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); i++)
        {
            if (jacobians[i])
            {
                int size = marginalization_info->keep_block_size[i], local_size = marginalization_info->localSize(size);
                int idx = marginalization_info->keep_block_idx[i] - m;
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[i], n, size);
                jacobian.setZero();
                jacobian.leftCols(local_size) = marginalization_info->linearized_jacobians.middleCols(idx, local_size);
            }
        }
    }
    return true;
}
