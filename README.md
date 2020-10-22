# Robot Localization with Particle Filter
Anna Griffin, Sherrie Shen <br>
October 21, 2020

## Introduction
Determining where a robot is relative to its environment is a very important yet a challenging problem to solve. One way to go about localizing a robot is by using a particle filter. Assuming we have a map of the environment, we can make educated guesses about its location by creating a probability distribution of where it could be from the laser scans obtained by the robot. Initially the guesses are random but as we compare the hypothetical positions to the data the robot is actually receiving, we can resample the particles using the information from those differences. When successful, the estimates will converge to the robot's true position.

For this project, our goal was to create our own particle filter algorithm to approach the challenge of robot localization. Since this method of localization takes estimates and uses them to make additional educated guesses, we face a tradeoff between efficiency and accuracy. This is a particularly interesting challenge because there are so many different use cases for robot localization which makes it hard to draw a hard line between the two.


## Particle Filter
The particle filter algorithm we designed is comprised of a few steps, some of which are repeated to narrow down the estimate, honing in on the true position of the robot.

First we create a particle cloud. Theses particles are positioned randomly on the map and then normalized. Since there is no previous information about the robot's location initially, the particles are distributed randomly and all of the particles have equal weights.

The robot's LIDAR scan reading can be compared to the map to determine how close of a match they are by comparing the scan value to the nearest obstacle in the map. The particles are weighted depending on how much variance exists between the scan and map data. This process helps inform the future particle cloud predictions when resampling because the probability of each particle is proportional to its weight. Using this information, the robot's pose is updated and then the algorithm returns to reweighing the new particles based on the laser scan of the updated robot.

## Implementation

Below is our implementation of the particle filter tested with AC109_1 bag file after tuning. The particles were able to converge relatively quickly given a good initial pose estimate. The particles did not condense into one error due to the introduced random noise when updating the particle's position by our motion model. There is about a 10cm deviation from where the robot estimated pose and true pose in the end. We were able to find the pose of the robot in the map with 150 particles.

[AC109 1 full video (Youtube)](https://youtu.be/IlxChRHc4kA)
![alt text](images/ParticleFilterReallyGoodAC109-1.gif "AC109 1")

--------------------------------------------------------------------------------------

Below is our implementation of the particle filter tested with AC109_2 bag file after tuning. Due to the configuration of the map, we need to reduce the radius of the circular range around the robot pose where particles are spawned. This is because compared to ac109_1, the starting position in ac109_2 is a lot less feature rich. Particles spawned at the end of the corridor facing in the opposite direction will have similar scans as particles spawned at the other end of the corridor. By decreasing the radius of the particle spawn range, we are able to concentrate the particles to one side of the corridor and not have the robot pose estimation affected by false positives in parts of the map that are similar to where the robot currently is. We see a similar about 10cm discrepancy between the estimated pose of the robot and the true position at the end.

[AC109 2 full video (Youtube)](https://youtu.be/gIMAhhX1jnE)
> file was too large to convert and upload as one file

![](images/ParticleFilterReallyGoodAC109-2(1).gif)    |  ![](images/ParticleFilterReallyGoodAC109-2(2).gif)
:-------------------------:|:-------------------------:
First Half  |  Second Half


### Update Particle Weight (Sensor Model)
The core of the particle filter is to approximate the robot pose in the map by applying laser scan measurement from the robot to particles in the map with known locations and see if such scan matches with the obstacles and environment of the chosen particle. In other words, we are computing the probability of seeing a scan measure k at time step t given location of a particle at time step t and the map:
`P(z^k_t | x_t, m)`. The total probability `P(z_t | x_t, m)` that combines `P(z^k_t | x_t, m)` for all laser scan measurement k corresponds to the weight we assign to the particle.

To compute the weights, we chose to use a liklihood field model for this step of the process and leverage the `OccupancyField` class which is able to calculate the distance of the nearest obstacle to a given location. How we perform this computation is that for each particle, we map the scan measure to it's coordinate system. For example, for one laser beam measurement from the scan, we can compute the x,y coordinate of the endpoint in the particle's coordinate frame by the following:

`x = particle.x + scan*math.cos(particle.theta + scan_angle)`

`y = particle.y + scan*math.sin(particle.theta + scan_angle)`

We then model the distance returned by the `OccupancyField` class method `findClosestObstacle(x,y)` as a Gaussian distribution centered at zero with some standard deviation. The larger the absolute value of d, the less likely for the robot to be at the location of that particle resulting a lower weight for that particle. The closer d value is to 0, the more likely for the robot to be in that particle's position.

Below is a sketch of a possible map of the robot's scan data and a representation of the likelihood model with the gaussian distributions of closest objects.

![](images/scandata.jpg)    |  ![](images/likelihoodmap.jpg)
:-------------------------:|:-------------------------:
Figure 1: Scan Data  | Figure 2: Likelihood Map
<br>

Take one particular scan for example (illustrated in Figure 1). This value that is returned is the distance from the robot to whatever is intercepting the laser scan at the given angle, an obstacle in our case. At the position of the end of the scan, we know there is an object because of how the LIDAR scan works. This point can be imposed within each particle's coordinate frame and then used to compare the measurement of the closest object.

In Figure 2, the darker the shading is, the farther away the point is from a known object. When the distance is as close to 0 as possible, we can conclude that the position is a relatively good match resulting in a higher probability that it is a match.

The particle A in Figure 2 is in a particularly darkly shaded area because it is very close to an object that is known in the map. This particle would receive a higher probability since we'd expect an object to be there relative to the particle. In contrast, the point x distance away from particle B in Figure 2 is nowhere near an object in the map. This particle would have a very low weight since it is unlikely that if the robot was in that position it would read the scan value to true robot did.

#### Design Decisions for Implementation
To calculate `P(z_t | x_t, m)` for each particle, instead of multiplying `P(z^k_t | x_t, m)` for all k following the independence assumption of individual measurement, we summed up all the probabilities and raise the final sum to the power of 6 after tuning. The larger the power, the faster it took the particles to consolidate into clusters. However, we do not want too large of a power as it can give false positive particles (particles located at other parts of the map with similar obstacle configuration as where the robot is located) a high weight and have the robot and the particles set stuck in these false locations.

We have also tried average the sum after raising it to a power by the number of measurements but we found out through testing that not averaging allows the particles to consolidate into a few probable location for the robot faster.

### Resampling Particles
For resampling the particles for the next time step, we applied the low variance resampling algorithm introduced in Probabilistic Robotics p87 instead of randomly choosing the particles based on the probability distribution defined by the particle weights. The motivation for us to use this algorithm is that the resampling step reduces the diversity in the particle population. One scenario this would be useful is when the Neato does not move much, and the environment does not really change, what we want in that case is to maintain the original probability distribution of the particles instead of randomly resample based o the probability of each particle. Randomly resample would make us lose particles of low weight and get more concentrated particles of higher weight even though when the surrounding of the robot did not really change much. The particles tend to be more easily to be stuck. The error introduced by this randomness increases the variance of the particle set as an estimator of the true belief while reducing the variance of the particle set.

The intuition of the algorithm is for a given step m where 0 <= m < total number of particles, we pick the first particle such that the accumulated weighted of all m particles from the first one is greater or equal to some number u where u is approximately m/total number of particles. This relationship can be explained by the the formula below:

![alt text](./images/low_variance_sampler_equation.png "Figure 3")
Figure 3: Particle selection step of low variance resampling algorithm from Probabilistic Robotics p87.i represents the index of the particle being chosen and u is equal to a random number r between 0 and 1/M(total number of particles) plus m*1/total number of particles.

The image below illustrates each iteration of picking a particle. For each step m, we add an additional 1/M(total number of particles) to u which corresponds to the case where all the particles have equal weights. In this case, we still prioritize resampling particles with larger weights but the process is deterministic and we avoid error introduced by the randomness of the resampler and keep the variance of the particle set as an estimator of the true belief low.

![alt text](./images/low_variance_sampler.png "Figure 4")

Figure 4: Graphic Intuition of low variance resampling algorithm from Probabilistic Robotics p87. The size of each bin corresponds to the weight of the particle, particles with weight much higher than 1/M will definitely be included in the resampled set.

Another advantage of using the low variance resampler is that if all samples have the same weights, the resampled set is the same and we do not lose any samples.

#### Design Decisions for Implementation
While testing the resampler, we figured the particles converges faster if we sort all particles in descending order of weights before resampling. This increases the efficiency of the resampler as well as we interact through the entire particle set for each step m to find a particle with large enough weight to meet the criteria.

However, one pitfall we found with our implementation is that the particles can get stuck easily when they located in a location on the map where the obstacle configuration is very similar to where the robot is. If we had more time, we can look into stratified sampling, in which particles are grouped into subsets where the samples within a subset are all kept regardless its individual weight. (Introduced in Probabilistic Robotics p88).

### Update Particle Position with Motion Model
The particle cloud is comprised of many hypotheses of the robot's true location. Since the robot is moving and each particle is a representation of a possible pose, the movement of the robot must be propagated to the estimates. The particles are able to copy the movement since the transformation is relative to a `\base_link` frame that each robot has which is aligned with its pose. This is an important step since each scan the robot takes will reveal additional information about its specific location which can only be accurately compared if the hypotheses imitate the same movement. If a particle happens to be exactly where the robot is, then its movements should mirror those of the true robot. We introduced some noise to these projections to account for drift in the wheel encoders and prevent the particles from getting stuck in the same position.


![alt text](images/movement.jpg "Figure 1")
Figure 3: Updating each particle with robot's movement

## Other Design Decisions

### Random Noise When Updating Odom
Since the particles are estimates of the robot's true pose, they must be updated to mimic the movement of the robot as it is navigating through the environment. As the robot moves, the transformations get propagated to each particle with a certain amount of noise factored in to account for drift of the wheel encoders. At first, we added random noise that was chosen from a normal distribution that had a standard deviation of 0.01. This however turned out to be too large making our algorithm susceptible to false positives. Reducing the value to 0.008 gave us the best results after playing around with a variety of nearby values.

### Particle Cloud Initialization Radius
At the start, we initialized the particles randomly around the map. This method was not a very robust or accurate way to go about making theses initial predictions. Instead we improved this step of the process by containing the randomize particles within a circle with a specified radius centered at the robot. This had a tremendous affect on the performance of the filter which also makes sense with our logic. Picture a room that is rectangular without many other features. It would be more difficult to distinguish one corner from the other. In this case, a smaller radius about the robot is a better fit for the situation. In contrast, when the space is feature rich, the radius can be larger since the scan data will most likely be more unique and easier to identify or eliminate.


### Sorting Particles for Updating Robot Pose
Compared to the first iteration of our `update_robot_pose()` function, our current implementation incorporates sorting to amplify the close matches. We approached this part of the algorithm by selecting a certain number of the best particles and approximate the robot pose by the average pose of these best particles. This we saw had an effect on the accuracy of the estimates because there was even more emphasis put on the highly probable particles while at the same time the lower ones were disregarded. The reason we chose to only use a subset of the particle set to estimate the robot pose is that it improves the efficiency of the algorithm and have the robot pose less susceptible noise from particles with small weights. This is especially true when the number of particles is large and spread across the entire map. We need to tune the number of best particles we will use to compute the final estimate of the robot pose. If this number is too small, the robot pose is not very smooth when they are two or competing cluster of candidate and potentially make the robot stuck in a false location. If this number is too large, then the efficiency of the algorithm reduces and it takes longer for the robot to consolidate into the correct estimated pose on the map.


### Updating Particles
In our algorithm, we assign weights to the particles by comparing the scan data to the closest obstacle distance based on the occupancy field. The closer this value is to 0, the higher the probability of it being a match. We originally multiplied the values but we were not seeing very good results. After tweaking and testing, we observed that adding the probabilities from each scan made more sense. Starting with adding the square of the `p_z` to the `total_prob` sum for each particle, we experimented with raising it to different powers to determine how it effect its performance.

## Improvements

### Not Requiring Good Initial Estimate of Robot Pose
The current version of our particle filter requires having a good initial estimate of the robot pose for the particles to eventually consolidate into the true position of the robot in the map. This is because our model is overly confident. If we increase the number of particles and drop them in a wider range, the particles will not condense as multiple locations on the map can get similar scan configuration if the map is not very feature rich. If we reduce the number of particles and drop them in a condensed range, then if we don't have a good initial estimate

### Develop better metrics for parameter tuning
Another opportunity for improvement for this project is developing some metric to measure the accuracy of each successive guess. At the moment, we assume that our process of reweighing and resampling improves the pose estimate of the robot. However, we do not check if it is in fact closer to the true pose nor do we have a way to measure to what degree the new cloud has improved our estimate. We did a lot of tweaking and testing but if we calculated those metrics in some way we could determine how much of an effect a particular tweak on a parameter like scan model standard deviation has on the estimated pose. We can either look into ML method of dynamic parameter tuning or visualization such as coloring the particle by its weight.

### Handle Non Stationary Object
With additional time, we could look more into handling objects that are not stationary. Since we are using a robotic vacuum, one obvious application of robot localization is mapping a house to ensure that entire floor surfaces have been cleaned. While some things like walls, couches, staircases, doorways, etc. are stationary and therefore mostly likely accounted for on the map, many objects in households move around and not included in the map. It would be an interesting extension to explore settings and fine tune detection with movable objects.


## Takeaways
We are both very pleased that we chose to focus more on the particle filter algorithm as opposed to the nitty gritty of ROS. Choosing this route allowed us to focus on a particular aspect of the project that really interested us, the algorithm, as opposed to debugging ROS errors which we both have had experience with previously. The lessons to gain from this is we have compatible learning goals which makes it an enjoyable experience for both of us to be working on a team together and following our interests was an effective decision when choices were presented to us.

Additionally, we did a good job planing our execution of this project. Drawing from our experience working on the previous project, we structured our process very effectively from start to finish. To begin, we did research outside of the lectures in class to ensure that we both had a good conceptual understanding of the problem (robot localization) and the algorithm (particle filter). Once we felt comfortable to move on, we planned out the different components of the algorithm and then started building up `ParticleFilter` class step by step.
