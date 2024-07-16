#include <utils/grid_utils.hpp>
#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/pose2D_t.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/geometric/angle_functions.hpp>
#include <cassert>


ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles),
  samplingAugmentation_(0.5, 0.9, numParticles),
  distribution_quality(1),
  quality_reinvigoration_percentage(0.1)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const mbot_lcm_msgs::pose2D_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    float xy_dev = 0.01;
    float theta_dev = 0.01;

    std::normal_distribution<> dist_xy(0, xy_dev);
    std::normal_distribution<> dist_theta(0, theta_dev);

    for (int j=0; j<kNumParticles_; ++j)
    {
        mbot_lcm_msgs::pose2D_t temp_p;
        mbot_lcm_msgs::particle_t particle;
        temp_p.utime = 0;
        temp_p.x = pose.x + dist_xy(num_gen);
        temp_p.y = pose.y + dist_xy(num_gen);
        temp_p.theta = wrap_to_pi(pose.theta + dist_theta(num_gen));
        
        particle.pose = temp_p;
        particle.weight = 1.0 / (double)kNumParticles_;
        particle.parent_pose = pose;

        posterior_.push_back(particle);
    }
    posterior_[posterior_.size() - 1].pose = pose;

}

void ParticleFilter::initializeFilterRandomly(const OccupancyGrid& map)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    RandomPoseSampler do_samp(&map);
    for (int j = 0; j < kNumParticles_; ++j)
    {
        mbot_lcm_msgs::particle_t particle = do_samp.get_particle();

        particle.weight = 1.0 / (double)kNumParticles_;

        posterior_.push_back(particle);
    }
    

}

void ParticleFilter::resetOdometry(const mbot_lcm_msgs::pose2D_t& odometry)
{
    actionModel_.resetPrevious(odometry);
}


mbot_lcm_msgs::pose2D_t ParticleFilter::updateFilter(const mbot_lcm_msgs::pose2D_t& odometry,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    auto part_prior = resamplePosteriorDistribution(map);
    auto part_porp = computeProposalDistribution(part_prior);
    posterior_ = computeNormalizedPosterior(part_porp, laser, map);
    /// TODO: Add reinvigoration step
    post_pose = estimatePosteriorPose(posterior_);

    post_pose.utime = odometry.utime;

    return post_pose;
}

mbot_lcm_msgs::pose2D_t ParticleFilter::updateFilterActionOnly(const mbot_lcm_msgs::pose2D_t& odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if(hasRobotMoved)
    {
        auto part_prior = resamplePosteriorDistribution();
        auto part_porp = computeProposalDistribution(part_prior);
        posterior_ = part_porp;
    }

    post_pose = odometry;

    return post_pose;
}


mbot_lcm_msgs::pose2D_t ParticleFilter::poseEstimate(void) const
{
    return post_pose;
}


mbot_lcm_msgs::particles_t ParticleFilter::particles(void) const
{
    mbot_lcm_msgs::particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


ParticleList ParticleFilter::resamplePosteriorDistribution(const OccupancyGrid& map,
                                                           const bool keep_best,
                                                           const bool reinvigorate)
{
    //////////// TODO: Implement your algorithm for resampling from the n_post distribution ///////////////////
    ParticleList part_prior;
    std::uniform_real_distribution<double> random_w(0, 1.0 / (double)kNumParticles_);

    int j = 0;
    double r = random_w(num_gen);
    double d = posterior_[0].weight;

    double n;

    for (int r = 0; r < kNumParticles_; ++r)
    {
        n = r + r * 1.0 / (double)kNumParticles_;

        while (n > d)
        {
            j++;

            d = d + posterior_[j].weight;
        }
        part_prior.push_back(posterior_[j]);
    }
    return part_prior;
    
}


ParticleList ParticleFilter::resamplePosteriorDistribution(const bool keep_best,
                                                           const bool reinvigorate)
{
    //////////// TODO: Implement your algorithm for resampling from the n_post distribution ///////////////////
    ParticleList part_prior;
    std::uniform_real_distribution<double> random_w(0, 1.0 / (double)kNumParticles_);
    int j = 0;
    double r = random_w(num_gen);
    double d = posterior_[0].weight;
    double n;
    for (int r = 0; r < kNumParticles_; ++r)
    {
        n = r + r * 1.0 / (double)kNumParticles_;
        while (n > d)
        {
            j++;

            d = d + posterior_[j].weight;
        }

        part_prior.push_back(posterior_[j]);
    }
    return part_prior;
    
}


void ParticleFilter::reinvigoratePriorDistribution(ParticleList& part_prior)
{
    // Augmentation: if sensor model suspects an average particle quality of
    //      less than 15%, invigorate
    if (distribution_quality < 0.15)  // TODO: make 0.15 a parameter
    {
        int count = 0;
        int max_count = floor(quality_reinvigoration_percentage * part_prior.size());

        std::random_device rd;
        std::default_random_engine generator(rd());
        auto ud01 = std::uniform_real_distribution<double>(0.0, 1.0);
        int step = std::max<int>(1, floor(ud01(generator) * part_prior.size() / max_count));

        for (int j = 0; j < max_count; j++)
        {
            part_prior[j*step] = randomPoseGen_.get_particle();
        }

    }
}


ParticleList ParticleFilter::computeProposalDistribution(const ParticleList& part_prior)
{
    //////////// TODO: Implement your algorithm for creating the part_porp distribution by sampling from the ActionModel
    ParticleList part_porp;
    for (auto particle : part_prior)
    {

        part_porp.push_back(actionModel_.applyAction(particle));
    }
    return part_porp;
}


ParticleList ParticleFilter::computeNormalizedPosterior(const ParticleList& part_porp,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    /////////// TODO: Implement your algorithm for computing the normalized n_post distribution using the
    ///////////       particles in the part_porp distribution

    ParticleList n_post;
    double weight_now;
    double nemp = 0;
    

    for (auto pro : part_porp)
    {
        mbot_lcm_msgs::particle_t new_par = pro;
       
        weight_now = sensorModel_.likelihood(new_par, laser, map);

        nemp += weight_now;
        
        new_par.weight = weight_now;
        
        n_post.push_back(new_par);
    }
    for (auto &particle : n_post)
    {

        particle.weight /= nemp;
    }
   
    return n_post;
}


mbot_lcm_msgs::pose2D_t ParticleFilter::estimatePosteriorPose(const ParticleList& n_post)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the n_post distribution
    // Figure out which pose to take for the n_post pose
    // Weighted average is simple, but could be very bad
    // Maybe only take the best x% and then average.
    ParticleList part_order = n_post;

    std::sort(part_order.begin(), part_order.end(),
              [](const mbot_lcm_msgs::particle_t &a, const mbot_lcm_msgs::particle_t &b)
              { return a.weight > b.weight; });

    size_t n_t_part = part_order.size() * 0.1;

    ParticleList t_part(part_order.begin(), part_order.begin() + n_t_part);

    mbot_lcm_msgs::pose2D_t pose = computeParticlesAverage(t_part);
    return pose;


}

mbot_lcm_msgs::pose2D_t ParticleFilter::computeParticlesAverage(const ParticleList& particles_to_average)
{
    mbot_lcm_msgs::pose2D_t avg_pose;
    avg_pose.x = 0.0;
    avg_pose.y = 0.0;
    avg_pose.theta = 0.0;
    double sum_weight = 0.0;

    // Aux variables to compute theta average
    double theta_x = 0.0;
    double theta_y = 0.0;
    for (auto &&temp_p : particles_to_average)
    {
        avg_pose.x += temp_p.weight * temp_p.pose.x;
        avg_pose.y += temp_p.weight * temp_p.pose.y;
        theta_x += temp_p.weight * std::cos(temp_p.pose.theta);
        theta_y += temp_p.weight * std::sin(temp_p.pose.theta);

        sum_weight += temp_p.weight;
    }
    avg_pose.x /= sum_weight;
    avg_pose.y /= sum_weight;
    theta_x /= sum_weight;
    theta_y /= sum_weight;
    avg_pose.theta = std::atan2(theta_y, theta_x);

    return avg_pose;
}