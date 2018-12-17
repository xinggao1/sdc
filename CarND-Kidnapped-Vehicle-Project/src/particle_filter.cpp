/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  num_particles = 128;
  is_initialized = true;
  default_random_engine randomEngine;
  normal_distribution<double> disX(x, std[0]), disY(y, std[1]), disT(theta, std[2]);
  for(int i = 0; i < num_particles; ++i) {
    particles.emplace_back(i, disX(randomEngine), disY(randomEngine), disT(randomEngine), 1);
  }
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  default_random_engine randomEngine;
  normal_distribution<double> disX(0, std_pos[0]), disY(0, std_pos[1]), disT(0, std_pos[2]);
  for(int i = 0; i < num_particles; ++i) {
    Particle &p = particles[i];
    if(yaw_rate != 0) {
      p.x += velocity / yaw_rate * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta)) + disX(randomEngine);
      p.y += velocity / yaw_rate * (cos(p.theta) - cos(p.theta + yaw_rate * delta_t)) + disY(randomEngine);
    }else {
      p.x += velocity * delta_t * cos(p.theta) + disX(randomEngine);
      p.y += velocity * delta_t * sin(p.theta) + disY(randomEngine);
    }
    p.theta += yaw_rate * delta_t + disT(randomEngine);
  }
}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
  for(int i = 0; i < num_particles; ++i) {
    Particle &p = particles[i];
    p.sense_x.clear();
    p.sense_y.clear();
    p.associations.clear();
    for(auto &lm: observations) {
      double x_map = p.x + (cos(p.theta) * lm.x) - (sin(p.theta) * lm.y);
      double y_map = p.y + (sin(p.theta) * lm.x) + (cos(p.theta) * lm.y);
      p.sense_x.push_back(x_map);
      p.sense_y.push_back(y_map);
    }
    vector<int> nearby;
    for(auto &lm: map_landmarks.landmark_list) {
      if(dist2(p.x, p.y, lm.x_f, lm.y_f) <= sensor_range * sensor_range)
        nearby.push_back(lm.id_i);
    }
    p.associations.resize(p.sense_x.size());
    if(nearby.empty()) {
      printf("empty near by %f %f %f\n", p.x, p.y, p.theta);
      p.weight = 0;
      continue;
    }
    double weightNew = 1;
    for(size_t j = 0; j < p.associations.size(); ++j) {
      auto itBest = min_element(nearby.begin(), nearby.end(),
                                [&p, &map_landmarks, j](int n1, int n2) {
                                  return
                                  dist2(p.sense_x[j], p.sense_y[j], map_landmarks.landmark_list[n1 - 1].x_f,
                                        map_landmarks.landmark_list[n1 - 1].y_f)
                                  < dist2(p.sense_x[j], p.sense_y[j], map_landmarks.landmark_list[n2 - 1].x_f,
                                          map_landmarks.landmark_list[n2 - 1].y_f);
                                });

      p.associations[j] = *itBest;
      auto mu_x = map_landmarks.landmark_list[p.associations[j] - 1].x_f;
      auto mu_y = map_landmarks.landmark_list[p.associations[j] - 1].y_f;
      double gauss_norm = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);
      double exponent = ((p.sense_x[j] - mu_x) * (p.sense_x[j] - mu_x)) / (2 * std_landmark[0] * std_landmark[0]) +
                        ((p.sense_y[j] - mu_y) * (p.sense_y[j] - mu_y)) / (2 * std_landmark[1] * std_landmark[1]);
      weightNew *= gauss_norm * exp(-exponent);
    }
    p.weight = weightNew;
  }
}

void ParticleFilter::resample() {
  weights.clear();
  default_random_engine randomEngine;
  for(auto &p: particles)
    weights.push_back(p.weight);
  discrete_distribution<int> dd(weights.begin(), weights.end());
  vector<Particle> newParticles;
  for(int i = 0; i < num_particles; ++i) {
    newParticles.push_back(particles[dd(randomEngine)]);
    newParticles[i].id = i;
  }
  particles.swap(newParticles);
}

string ParticleFilter::getAssociations(Particle best) {
  const vector<int> &v = best.associations;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseX(Particle best) {
  const vector<double> &v = best.sense_x;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseY(Particle best) {
  const vector<double> &v = best.sense_y;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}
