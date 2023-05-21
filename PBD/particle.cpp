#include "particle.h"

#include "utility.h"

Particle::Particle(glm::vec3 X, glm::vec3 V, float mass, float r, int group, glm::vec3 color, glm::vec3 goal)
{
	this->X = X;
	this->X_pred = glm::vec3(X.x, 0, X.z);
	this->Delta_x = glm::vec3(0., 0., 0.);
	this->Delta_x_ctr = 0;
	this->V = V;
	this->Accel = glm::vec3(0., 0., 0.);
	this->V_prev = glm::vec3(0., 0., 0.);
	this->V_pref = V_PREF_ACCEL;
	this->mass = mass;
	this->inv_mass = 1.0 / mass;
	this->group_id = group;
	this->goal = goal;
	this->final_goal = goal;
	this->r = r;

	this->cell_id = K_NOT_USED;
	this->cell_x = K_NOT_USED;
	this->cell_z = K_NOT_USED;
	this->color = color;
	this->path_ptr = 0;

	this->is_link = true;
}