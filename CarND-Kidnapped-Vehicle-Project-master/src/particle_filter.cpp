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
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	
	// Set the number of particles N
	num_particles = 100;
	std::default_random_engine gen;
	double std_x, std_y, std_theta; // Standard deviations for x, y, and theta

	// Set standard deviations for x, y, and theta
	std_x = std[0];
	std_y = std[1];
	std_theta = std[2];

	// This line creates a normal (Gaussian) distribution for x
	std::normal_distribution<double> dist_x(x, std_x);
	// Create normal distributions for y and theta
	std::normal_distribution<double> dist_y(y, std_y);
	std::normal_distribution<double> dist_theta(theta, std_theta);

	for (int i = 0; i < num_particles; ++i) {
		Particle sample;
		//Sample  and from these normal distrubtions like this: 
		//	 sample_x = dist_x(gen);
		//	 where "gen" is the random engine initialized earlier.
		sample.id = i;
		sample.x = dist_x(gen);
		sample.y = dist_y(gen);
		sample.theta = dist_theta(gen);
		sample.weight = 1.0;
		weights.push_back(sample.weight);
		particles.push_back(sample);
		// Print your samples to the terminal.
		//cout << "Sample " << i + 1 << " " << sample_x << " " << sample_y << " " << sample_theta << endl;
	}

	is_initialized = true;


}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	std::default_random_engine gen;
	double std_x, std_y, std_theta; // Standard deviations for x, y, and theta

	// Set standard deviations for x, y, and theta
	std_x = std_pos[0];
	std_y = std_pos[1];
	std_theta = std_pos[2];

	int num_particles = particles.size();
	double x, y, theta;
	for (int i = 0; i < num_particles; ++i) {
		//x = particles[i].x;
		//y = particles[i].y;
		//theta = particles[i].theta;
		if (fabs(yaw_rate)  < 0.0001) {
			x = particles[i].x + (velocity * delta_t) * cos(particles[i].theta);
			y = particles[i].y + (velocity * delta_t) * sin(particles[i].theta);
			theta = particles[i].theta;
		}
		else {
			x = particles[i].x + (velocity / yaw_rate) * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
			y = particles[i].y + (velocity / yaw_rate) * (-cos(particles[i].theta + yaw_rate * delta_t) + cos(particles[i].theta));
			theta = particles[i].theta + yaw_rate*delta_t;
		}
		// This line creates a normal (Gaussian) distribution for x, y and theta
		std::normal_distribution<double> dist_x(x, std_x);
		std::normal_distribution<double> dist_y(y, std_y);
		std::normal_distribution<double> dist_theta(theta, std_theta);

		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);
	}


}

/* Find the predicted measurement that is closest to each observed measurement and assign the
   observed measurement to this particular landmark, with exhausted search (may be replaced with KD-tree approaches)
*/
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// todo: find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// note: this method will not be called by the grading code. but you will probably find it useful to 
	//   implement this method and use it as a helper during the updateweights phase.
	int num_obs = observations.size();
	int num_preds = predicted.size();
	double dists = 0.0;
	for (int i = 0; i < num_obs; i++) {
		double minD = std::numeric_limits<float>::max(); 
		for (int j = 0; j < num_preds; j++) {
			dists = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y); //pow(observations[i].x - predicted[j].x, 2) + pow(observations[i].y - predicted[j].y, 2);
			if (dists < minD) {
				observations[i].id = predicted[j].id;
				minD = dists;
			}
		}
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	for (int p = 0; p < num_particles; p++) {
		double p_x = particles[p].x;
		double p_y = particles[p].y;
		double p_theta = particles[p].theta;
		// reinit weight
		particles[p].weight = 1.0;
		// create a vector to hold the map landmark locations predicted to be within sensor range of the particle
		vector<LandmarkObs> predictions;

		// for each map landmark...
		for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {

			// get id and x,y coordinates
			float lm_x = map_landmarks.landmark_list[j].x_f;
			float lm_y = map_landmarks.landmark_list[j].y_f;
			int lm_id = map_landmarks.landmark_list[j].id_i;

			// only consider landmarks within sensor range of the particle (rather than using the "dist" method considering a circular 
			// region around the particle, this considers a rectangular region but is computationally faster)
			double distance = dist(p_x, p_y, lm_x, lm_y);
			if (distance <= sensor_range) {
				predictions.push_back(LandmarkObs{ lm_id, lm_x, lm_y });// add prediction to vector
			}
		}

		vector<LandmarkObs> transformed_os;
		for (unsigned int j = 0; j < observations.size(); j++) {
			double t_x = cos(p_theta)*observations[j].x - sin(p_theta)*observations[j].y + p_x;
			double t_y = sin(p_theta)*observations[j].x + cos(p_theta)*observations[j].y + p_y;
			transformed_os.push_back(LandmarkObs{ observations[j].id, t_x, t_y });
		}
		// perform dataAssociation for the predictions and transformed observations on current particle
		dataAssociation(predictions, transformed_os);

		//compute the particle's weight
		for (const auto& obs_m : transformed_os) {
			Map::single_landmark_s landmark = map_landmarks.landmark_list.at(obs_m.id - 1);
			double x_term = pow(obs_m.x - landmark.x_f, 2) / (2 * pow(std_landmark[0], 2));
			double y_term = pow(obs_m.y - landmark.y_f, 2) / (2 * pow(std_landmark[1], 2));
			double w = exp(-(x_term + y_term)) / (2 * M_PI * std_landmark[0] * std_landmark[1]);
			particles[p].weight *= w;
		}
		weights.push_back(particles[p].weight);
	}
}

void ParticleFilter::resample() {

	// generate distribution according to weights
	std::random_device rd;
	std::mt19937 gen(rd());
	std::discrete_distribution<> dist(weights.begin(), weights.end());

	// create resampled particles
	vector<Particle> resampled_particles;
	resampled_particles.resize(num_particles);

	// resample the particles according to weights
	for (int i = 0; i<num_particles; i++) {
		int idx = dist(gen);
		resampled_particles[i] = particles[idx];
	}

	// assign the resampled_particles to the previous particles
	particles = resampled_particles;

	// clear the weight vector for the next round
	weights.clear();
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations = associations;
	particle.sense_x = sense_x;
	particle.sense_y = sense_y;

	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
