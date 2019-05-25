# SELF DRIVING CAR NANODEGREE
# project6. Kidnapped Vehicle

## 1. Abstraction

The object of this project is to localize car's position

Before project (Kalman filter) is to detect object's using sensor fusion, and this project is detect my position exactly

by also using sensor fusion

Of coulse it is possible localize my position by using GPS, but its range accuracy is more than 1m

We need at least 10cm accuracy so we need more than just GPS

In this project, I will use GPS for initializing first car's position

after than use sensor fusion for detecting landmarks that already know its position

and finally find car's exact position by using particle filter



## 2. Related Study


#### 1) Motion models

① Assumption for bicycle model

② (x,y,theta) prediction


#### 2) Particle Filters

① About landmarks

② Role of weights

③ Resampling


#### 3) Implementation of a Particle Filter

① Algorithm steps : Initiation → Prediction → Update → Resampling

② About initiation : Meaning of number of particles, Initiation position

③ About prediction : Motion models

④ About update step : Data Association, Update weights, Transformations

## 3. Details

#### 1) Content Of This Repo

- ```src``` a directory with the project code
    - ```main.cpp``` : reads in data, calls a function to run the Particle filter, calls a function to calculate Error
    - ```ParticleFilter.cpp``` : describes particle filter functions
    - ```ParticleFilter.h``` : header file for .cpp
    - ```map.h``` : information about landmark position
    - ```json.hpp``` : for using json in main.cpp
    - ```helper_functions.h``` : functions used to in ParticleFilter.cpp

#### 2) Code

1. ParticleFilter.cpp

```c++
#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   1. Set the number of particles. 
   2. Initialize all particles to first position (based on estimates of x, y, theta and their uncertainties from GPS) and all weights to 1. 
   3. Add random Gaussian noise to each particle.
   **/
  
  if(is_initialized){
    return;
  }
  
  // Set the number of particles
  num_particles = 20;  
  
  // Set standard deviation of GPS data
  double std_dev_x = std[0];
  double std_dev_y = std[1];
  double std_dev_theta = std[2];
  
  // Set Gaussian distribution of GPS data
  // This is same as in class
  normal_distribution<double> dist_x(x, std_dev_x);
  normal_distribution<double> dist_y(y, std_dev_y);
  normal_distribution<double> dist_theta(theta, std_dev_theta);
  
  // Make particles distibuted by Gaussian with GPS data
  // 'gen' is std default randome engine defined in ParticleFilter class
  for(int i=0; i < num_particles; i++){
    Particle particle;
    particle.id = i;
    particle.x = dist_x(gen);
    particle.y = dist_y(gen);
    particle.theta = dist_theta(gen);
    particle.weight = 1.0;
    
    // Particles is a vector<Particle> defined in ParticleFilter class
    particles.push_back(particle);
    
    // All sets are done, change is_initialized to true
    is_initialized = true;
  }

}


void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   1. Predict next position of particle and add random Gaussian noise.
   **/

  // Set standard deviation of predict
  double std_dev_x = std_pos[0];
  double std_dev_y = std_pos[1];
  double std_dev_theta = std_pos[2];
  
  // Make standard deviation of predict, this will be added to predict result
  normal_distribution<double> dist_x(0, std_dev_x);
  normal_distribution<double> dist_y(0, std_dev_y);
  normal_distribution<double> dist_theta(0, std_dev_theta);
  
  // Predict new state by using velocity, yaw rate
  for(int i = 0; i < num_particles; i++){
    double theta = particles[i].theta;
    
    // this is for yaw_rate did not changed
    // that means car is driving straightly
    if( fabs(yaw_rate) < 0.000001 ){
      particles[i].x += velocity * delta_t * cos( theta );
      particles[i].y += velocity * delta_t * sin( theta );
    }
    // this is for car changed its direction
    else{
      particles[i].x += velocity / yaw_rate * ( sin( theta + yaw_rate * delta_t ) - sin( theta ) );
      particles[i].y += velocity / yaw_rate * ( cos( theta ) - cos( theta + yaw_rate * delta_t ) );
      particles[i].theta += yaw_rate * delta_t;
    
    // Finally we need to add noise to predicted results
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);
    }
  }
}


  
void ParticleFilter::dataAssociation (vector<LandmarkObs> predicted, vector<LandmarkObs>& observations) {
  // predicted : landmarks that is in range of radar/ridar sensor
  // observations : all sensor data transformed by particles  
  
  /**
   1. Find the predicted measurement that is closest to each observed measurement and assign the observed measurement to this particular landmark.
   2. This function will be used in updateWeights
   **/

  // dataAssociation is to matching all particles to nearest landmarks, and that landmarks should be in range of radar/ridar sensor
  // it will be used for updateWeights
  int number_landmarks = predicted.size();
  int number_sensor_data = observations.size();
  
  // Calculate distance between all particles and landmarks
  // Set particles.id to nearest landmarks
  for (int i = 0; i < number_sensor_data; i++){
    
    // set minimum distance to maximum double value
    // For each particles, update minimum distance from landmarks
    double min_distance = numeric_limits<double>::max();
    
    // Declare map_Id, that will be matching number for each particles to nearest landmark
    int map_id;
    
    for (int j = 0; j < number_landmarks; j++){
      
      double distance = sqrt(pow(observations[i].x - predicted[j].x,2) + pow(observations[i].y - predicted[j].y ,2));
      
      // Find nearest landmark and remember that map id
      if(distance < min_distance){
        min_distance = distance;
        map_id = predicted[j].id;
      }
      
    }
    // Set particles id
    observations[i].id = map_id;
  }    
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   1. Update the weights of each particle using a mult-variate Gaussian distribution. 
   **/
  
  double std_dev_range = std_landmark[0];
  double std_dev_bearing = std_landmark[1];
  
  // 1. Landmarks entering sensor range should be found first based on all points
  for (int i = 0; i < num_particles; i++){
    
    // This is setting for particles data
    double x = particles[i].x;
    double y = particles[i].y;
    double theta = particles[i].theta;
    
    // Landmarks in sensor range will be added to this vector
    vector<LandmarkObs> in_sensor_range_landmarks;
    
    for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++){
      
      // This is setting for landmarks data
      float landmark_x = map_landmarks.landmark_list[j].x_f;
      float landmark_y = map_landmarks.landmark_list[j].y_f;
      int landmark_id = map_landmarks.landmark_list[j].id_i;
      
      // Distance between particles and landmarks
      // This should be compared to sensor range
      double distance_x = x - landmark_x;
      double distance_y = y - landmark_y;
      double distance_land_between_particle = sqrt(pow(distance_x,2) + pow(distance_y,2));
      
      // If distance between particles and landmarks is in sensor range, add landmarks data to vector<LandmkarObs>
      if(distance_land_between_particle < sensor_range){
        
        in_sensor_range_landmarks.push_back(LandmarkObs{landmark_id, landmark_x, landmark_y});
        
      }     
    }
    
    // 2. Transform sensor data by particles data
    
    // This data will be added to vector<LandmarkObs>
    vector<LandmarkObs> transformed_observations;
    
    // Calculate transformed coordinate for each sensor data
    for (unsigned int j = 0; j < observations.size() ; j++){
     
      double transformed_x = cos(theta)*observations[j].x - sin(theta)*observations[j].y + x;
      double transformed_y = sin(theta)*observations[j].x + cos(theta)*observations[j].y + y;
      transformed_observations.push_back(LandmarkObs{ observations[j].id, transformed_x, transformed_y });      
      
    }
    
    dataAssociation(in_sensor_range_landmarks, transformed_observations);
    
    // Reset weight because we will multiply each weight, so it should be set 1 before calculating
    particles[i].weight = 1.0;
    
    for (unsigned int j = 0; j < transformed_observations.size(); j++){
      
      double transformed_sensor_x = transformed_observations[j].x;
      double transformed_sensor_y = transformed_observations[j].y;
      int landmark_sensor_matched_id = transformed_observations[j].id;
      
      double landmark_sensor_matched_x;
      double landmark_sensor_matched_y;
      
      int k = 0;
      int number_landmarks = in_sensor_range_landmarks.size();
      bool found = false;
      
      while( !found && k < number_landmarks) {
        
        if ( in_sensor_range_landmarks[k].id == landmark_sensor_matched_id) {
          
          found = true;
          landmark_sensor_matched_x = in_sensor_range_landmarks[k].x;
          landmark_sensor_matched_y = in_sensor_range_landmarks[k].y;
        }
        k++;
      }
      
      double dx = transformed_sensor_x - landmark_sensor_matched_x;
      double dy = transformed_sensor_y - landmark_sensor_matched_y;
      
      double weight = ( 1/(2*M_PI*std_dev_range*std_dev_bearing)) * exp( -( dx*dx/(2*std_dev_range*std_dev_range) + (dy*dy/(2*std_dev_bearing*std_dev_bearing)) ) );
      
      if ( weight == 0 ) {
        particles[i].weight *= 0.000001; 
      }
      else {
        particles[i].weight *= weight;
      }  
    }
  }
}

void ParticleFilter::resample() {
  /**
   1. Resample particles with replacement with probability proportional to their weight. 
   **/
  
  // Resample using wheel method taught by class
  
  // Calculate Max weight
  vector<double> weights;
  double max_weight = numeric_limits<double>::min();
  
  for ( int i = 0; i < num_particles; i++) {
    // set weights vector containing all weights of particles
    weights.push_back(particles[i].weight);
    // select Maximum weight
    if ( particles[i].weight > max_weight ) {
      max_weight = particles[i].weight; 
    }
  }
  
  // This will be used for generating beta, index
  uniform_real_distribution<double> dist_double(0.0, max_weight);
  uniform_int_distribution<int> dist_int(0, num_particles-1);
  
  // generating index
  int index = dist_int(gen);
  
  // declare beta
  double beta = 0.0;
  
  // Resample by wheel method
  // This is method taught at class
  vector<Particle> resampled_particles;
  for ( int i = 0; i < num_particles; i++) {
    beta += dist_double(gen) * 2.0;
    while ( beta > weights[index] ) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    resampled_particles.push_back(particles[index]);  
  }
  particles = resampled_particles;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
```


## 4. Results

<img src="./images/result.png" width="700">


## 5. Discussion

#### 1) About total flow of self-driving car

What do we need to make self driving car?

Firstly, we may need detect surroundings like we see when driving

For that, I learned computer vision, deep learning, sensor fusion at before projects

Secondly, we need to know where am I exactly, in the range of 10cm

This project is at exactly this point

This, particle filter is for localization of my exact position

Thirdly, we need to decide how to react like go straight, turn left, change lanes ...

Next project will cover this subjects






