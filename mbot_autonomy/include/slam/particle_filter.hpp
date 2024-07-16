#ifndef SLAM_PARTICLE_FILTER_HPP
#define SLAM_PARTICLE_FILTER_HPP

#include <vector>
#include <algorithm>

#include <mbot_lcm_msgs/lidar_t.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <mbot_lcm_msgs/particles_t.hpp>
#include <mbot_lcm_msgs/pose2D_t.hpp>

#include <slam/occupancy_grid.hpp>
#include <slam/sensor_model.hpp>
#include <slam/action_model.hpp>

typedef std::vector<mbot_lcm_msgs::particle_t> ParticleList;

static ParticleList importanceSample(const int num_particles,
                                     const ParticleList& particles)
{
  ParticleList samples;

  if (num_particles < 1 || particles.size() < 1) return samples;

  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::uniform_real_distribution<float> distribution(0.0, 1.0);

  while (samples.size() < num_particles)
  {
    float r = distribution(gen);
    int idx = 0;
    float sum = particles[idx].weight;
    while (sum < r) {
      ++idx;
      sum += particles[idx].weight;
    }
    samples.push_back(particles[idx]);
  }

  return samples;
};


static ParticleList lowVarianceSample(const int num_particles,
                                      const ParticleList& particles)
{
    ParticleList samples;

    if (num_particles < 1 || particles.size() < 1) return samples;

    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::uniform_real_distribution<float> distribution(0.0, 1.0 / num_particles);

    float r = distribution(gen);
    size_t idx = 0;
    float s = particles[idx].weight;

    for (int i = 0; i < num_particles; ++i)
    {
        float u = r + i * 1. / num_particles;
        while (u > s) {
            ++idx;
            s += particles[idx].weight;
        }
        samples.push_back(particles[idx]);
    }

    return samples;
};


/**
 * @brief Used to track averages for augmenting MCL sampling with a randomness.
 *      Refer to the probabilistic robotics book section table 8.3 for more details.
 */
class SamplingAugmentation{
    public:
    SamplingAugmentation(float slow_rate, float fast_rate, int num_particles, float max_rate = 1.0)
    :   slow_rate_(slow_rate), fast_rate_(fast_rate),
        slow_average_weight_(1/num_particles), fast_average_weight_(1/num_particles),
        max_rate_(max_rate),
        generator(std::random_device()()), uniform_distribution(0.0, 1.0),
        rand_sample_prob_(0)
    { }

    bool sample_randomly()
    {
        return uniform_distribution(generator) < rand_sample_prob_;
    }

    void insert_average_weight(float avg_w)
    {
        slow_average_weight_ += slow_rate_*(avg_w - slow_average_weight_);
        fast_average_weight_ += fast_rate_*(avg_w - fast_average_weight_);
        rand_sample_prob_ = std::max<float>(0.0, 1 - fast_average_weight_/slow_average_weight_);
        rand_sample_prob_ = std::min<float>(rand_sample_prob_, max_rate_);
    }

    private:
    float slow_average_weight_, fast_average_weight_;
    float slow_rate_, fast_rate_, max_rate_;
    float rand_sample_prob_;
    std::default_random_engine generator;
    std::uniform_real_distribution<double> uniform_distribution;

};


/**
 * @brief Randomly generates poses in the free space
 * of an occupancy grid.
 */
class RandomPoseSampler
{
public:
    RandomPoseSampler() :
        max_attempts_(1000),
        start_x_(0),
        end_x_(1),
        start_y_(0),
        end_y_(1)
    {}

    RandomPoseSampler(const OccupancyGrid* map_in, int max_attempts=10000)
    : max_attempts_(max_attempts)
    {
        auto origin = map_in->originInGlobalFrame();
        double initial_x = origin.x;
        double end_x = origin.x + map_in->widthInMeters();
        double initial_y = origin.y;
        double end_y = origin.y + map_in->heightInMeters();

        distr_x = std::uniform_real_distribution<double>(initial_x, end_x);
        distr_y = std::uniform_real_distribution<double>(initial_y, end_y);
        distr_theta = std::uniform_real_distribution<double>(-M_PI, M_PI);
    }

    RandomPoseSampler(const std::vector<float>& map_bounds, const int max_attempts=1000)
    : max_attempts_(max_attempts)
    {
        if (map_bounds.size() != 4)
        {
            std::cout << "RandomPoseSampler: WARNING! Bounds should have length 4." << std::endl;
        }
        start_x_ = map_bounds[0];
        end_x_ = map_bounds[1];
        start_y_ = map_bounds[2];
        end_y_ = map_bounds[3];

        distr_x = std::uniform_real_distribution<double>(start_x_, end_x_);
        distr_y = std::uniform_real_distribution<double>(start_y_, end_y_);
        distr_theta = std::uniform_real_distribution<double>(-M_PI, M_PI);
    }

    mbot_lcm_msgs::pose2D_t get_pose()
    {
        std::random_device random_device;
        std::default_random_engine random_engine(random_device());

        mbot_lcm_msgs::pose2D_t pose;
        pose.x = distr_x(random_engine);
        pose.y = distr_y(random_engine);
        pose.theta = distr_theta(random_engine);
        pose.utime = 0;
        return pose;
    }

    mbot_lcm_msgs::pose2D_t get_pose(const OccupancyGrid& map)
    {
        // TODO: Get a list of free cells and randomly select one instead of retrying. This often fails.
        for (int i; i < max_attempts_; i++)
        {
            mbot_lcm_msgs::pose2D_t pose = get_pose();
            // Check if robot could be there
            auto cell = global_position_to_grid_cell(Point<double>(pose.x, pose.y), map);
            // TODO: reduce available space by incorporing robot radius
            if (!map.isCellOccupied(cell.x, cell.y))
            {
                return pose;
            }
        }
        // Uncomment for debugging. This error is too noisy.
        // std::cout << "ERROR: Particle filter failed to sample a random valid pose from map! ";
        // std::cout << "Returning a random pose in the bounds." << std::endl;
        return get_pose();
    }

    mbot_lcm_msgs::particle_t get_particle()
    {
        mbot_lcm_msgs::particle_t p;
        p.pose = p.parent_pose = get_pose();
        p.weight = 0;
        return p;
    }

    mbot_lcm_msgs::particle_t get_particle(const OccupancyGrid& map)
    {
        mbot_lcm_msgs::particle_t p;
        p.pose = p.parent_pose = get_pose(map);
        p.weight = 0;
        return p;
    }

private:
    int max_attempts_;
    float start_x_, end_x_, start_y_, end_y_;
    std::uniform_real_distribution<double> distr_x, distr_y, distr_theta;
};


/**
* ParticleFilter implements a standard SIR-based particle filter. The set of particles is initialized at some pose. Then
* on subsequent calls to updateFilter, a new pose estimate is computed using the latest odometry and laser measurements
* along with the current map of the environment.
*
* This implementation of the particle filter uses a fixed number of particles for each iteration. Each filter update is
* a simple set of operations:
*
*   1) Draw N particles from current set of weighted particles.
*   2) Sample an action from the ActionModel and apply it to each of these particles.
*   3) Compute a weight for each particle using the SensorModel.
*   4) Normalize the weights.
*   5) Use the max-weight or mean-weight pose as the estimated pose for this update.
*/
class ParticleFilter
{
public:

    /**
    * Constructor for ParticleFilter.
    *
    * \param    numParticles        Number of particles to use
    * \pre  numParticles > 1
    */
    ParticleFilter(int numParticles);

    /**
    * initializeFilterAtPose initializes the particle filter with the samples distributed according
    * to the provided pose estimate.
    *
    * \param    pose            Initial pose of the robot
    */
    void initializeFilterAtPose(const mbot_lcm_msgs::pose2D_t& pose);

    void initializeFilterRandomly(const OccupancyGrid& map);

    /**
    * updateFilter increments the state estimated by the particle filter. The filter update uses the most recent
    * odometry estimate and laser scan along with the occupancy grid map to estimate the new pose of the robot.
    *
    * \param    odometry        Calculated odometry at the time of the final ray in the laser scan
    * \param    laser           Most recent laser scan of the environment
    * \param    map             Map built from the maximum likelihood pose estimate
    * \return   Estimated robot pose.
    */
    mbot_lcm_msgs::pose2D_t updateFilter(const mbot_lcm_msgs::pose2D_t& odometry,
                                            const mbot_lcm_msgs::lidar_t& laser,
                                            const OccupancyGrid& map);

    /**
    * updateFilterActionOnly increments the state estimated by the particle filter but only applies the action model
    *
    * \param    odometry        Calculated odometry at the time of the final ray in the laser scan
    * \return   odometry robot pose.
    */
    mbot_lcm_msgs::pose2D_t updateFilterActionOnly(const mbot_lcm_msgs::pose2D_t& odometry);

    /**
    * poseEstimate retrieves the current pose estimate computed by the filter.
    */
    mbot_lcm_msgs::pose2D_t poseEstimate(void) const;

    /**
    * particles retrieves the n_post set of particles being used by the algorithm.
    */
    mbot_lcm_msgs::particles_t particles(void) const;

    void resetOdometry(const mbot_lcm_msgs::pose2D_t& odometry);

private:

    ParticleList posterior_;     // The n_post distribution of particles at the end of the previous update
    mbot_lcm_msgs::pose2D_t post_pose;  // Pose estimate associated with the n_post distribution

    ActionModel actionModel_;   // Action model to apply to particles on each update
    SensorModel sensorModel_;   // Sensor model to compute particle weights
    float distribution_quality;
    float quality_reinvigoration_percentage;

    int kNumParticles_;         // Number of particles to use for estimating the pose
    std::mt19937 num_gen;
    ParticleList resamplePosteriorDistribution(const bool keep_best = true,
                                               const bool reinvigorate = true);
    ParticleList resamplePosteriorDistribution(const OccupancyGrid& map,
                                               const bool keep_best = true,
                                               const bool reinvigorate = true);
    void reinvigoratePriorDistribution(ParticleList& part_prior);
    ParticleList computeProposalDistribution(const ParticleList& part_prior);
    ParticleList computeNormalizedPosterior(const ParticleList& part_porp,
                                            const mbot_lcm_msgs::lidar_t& laser,
                                            const OccupancyGrid& map);
    mbot_lcm_msgs::pose2D_t estimatePosteriorPose(const ParticleList& n_post);
    mbot_lcm_msgs::pose2D_t computeParticlesAverage(const ParticleList& particles_to_average);

    SamplingAugmentation samplingAugmentation_;
    RandomPoseSampler randomPoseGen_;
};

#endif // SLAM_PARTICLE_FILTER_HPP