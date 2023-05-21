#include "utility.h"
#include "constraint.h"
#include "simulation.h"
#include "particle.h"
#include "wall.h"

#pragma region BASE
Constraint::Constraint(Simulation* sim, int num_particles)
{
	this->sim = sim;
	this->num_particles = num_particles;
	this->delta_X = (glm::vec3*)malloc(num_particles * sizeof(glm::vec3));
	this->indicies = (int*)malloc(num_particles * (sizeof(int)));
	this->active = false;
	for (int i = 0; i < num_particles; i++)
	{
		delta_X[i] = glm::vec3(0., 0., 0.);
	}
}
Constraint::~Constraint()
{
	free(indicies);
	free(delta_X);
}
#pragma endregion

#pragma region Powerlaw
Powerlaw_Constraint::Powerlaw_Constraint(Simulation* sim, int i, int j) : Constraint(sim, 2) 
{
	this->i = i;
	this->j = j;
	this->indicies[0] = i;
	this->indicies[1] = j;
	// TODO if seg fault happens, it is because the particles are set up after
	// the constraints
	this->w_i_coef = sim->particles[i]->inv_mass;
	this->w_j_coef = sim->particles[j]->inv_mass;
	this->out = glm::vec3(0., 0., 0.);
	this->collision_margin =
		(sim->particles[i]->r + sim->particles[j]->r) * 1.05f;
	this->radius_init = (sim->particles[i]->r + sim->particles[j]->r);
	this->radius_sq_init = radius_init * radius_init;
	this->delta_t = sim->time_step;
	this->dv_i = 1.;  // 1./delta_t;
	this->dv_j = -1.; //-1./delta_t;
	this->max_acceleration = sim->time_step * MAX_ACCEL;
}
void Powerlaw_Constraint::project(Particle** particles)
{
	if (particles[i]->is_leader || particles[j]->is_leader)
		return;

	// we don't want to use the bad old values
	delta_X[0] = glm::vec3(0, 0, 0);
	delta_X[1] = glm::vec3(0, 0, 0);

	// 같은 그룹은 Long range 적용 x
	if (particles[i]->group_id == particles[j]->group_id)
	{
		return;
	}

	glm::vec3 x_i = particles[i]->X;
	glm::vec3 x_j = particles[j]->X;

	glm::vec3 x_pred_i = particles[i]->X_pred;
	glm::vec3 x_pred_j = particles[j]->X_pred;

	const float dist = distance(x_i, x_j);
	const float dist_pred = distance(x_pred_i, x_pred_j);
	float radius_sq = radius_sq_init;

	// 두 파티클이 접촉했을 때,
	if (dist < radius_init)
	{
		// 파고든 만큼의 거리
		radius_sq = (radius_init - dist) * (radius_init - dist);
	}

	// 두 파티클의 pred 위치 차이 / 시간
	float v_x = (x_pred_i.x - x_i.x) / delta_t - (x_pred_j.x - x_j.x) / delta_t;
	float v_z = (x_pred_i.z - x_i.z) / delta_t - (x_pred_j.z - x_j.z) / delta_t;

	// 두 파티클 사이의 현재 위치 차이
	float x0 = x_i.x - x_j.x;
	float z0 = x_i.z - x_j.z;

	float v_sq = v_x * v_x + v_z * v_z;

	float x0_sq = x0 * x0;
	float z0_sq = z0 * z0;
	float x_sq = x0_sq + z0_sq;  // x_i의 magnitude_sq

	// a
	float a = v_sq;
	//float a = (dist_pred * dist_pred) / (delta_t * delta_t);

	// b (x0, y0) * (v_x, v_y)
	float b = -v_x * x0 - v_z * z0;
	//float b = -dot((x_i - x_j), (x_pred_i - x_pred_j)) / delta_t;

	// c
	//float c_ = x_sq - radius_sq;
	float c = dist * dist - radius_sq;

	// d
	float d_sq = (b * b) - (a * c);
	float d = sqrtf(d_sq);

	if (d_sq > 0 && (a < -_EPSILON || a > _EPSILON))
	{
		// time
		float tao = (b - d) / a;

		if (tao > 0 && tao < tao0)
		{
			float tao_alt = (b + d) / a;

			// pick the min solution that is > 0 (작은 값 선택 -> 더 가까운 지점에서 충돌한다)
			tao = (tao_alt < tao) && (tao_alt > 0) ? tao_alt : tao;
			//printf("%f\n", tao);

			float tao_hat = delta_t * floorf(tao / delta_t);  // clamp tao
			float tao_dia = tao_hat + delta_t;

			float x_hat_i = x_i.x + tao_hat * particles[i]->V.x;
			float z_hat_i = x_i.z + tao_hat * particles[i]->V.z;
			float x_hat_j = x_j.x + tao_hat * particles[j]->V.x;
			float z_hat_j = x_j.z + tao_hat * particles[j]->V.z;
			glm::vec3 hat_i = glm::vec3(x_hat_i, 0, z_hat_i);
			glm::vec3 hat_j = glm::vec3(x_hat_j, 0, z_hat_j);

			float x_dia_i = x_i.x + tao_dia * particles[i]->V.x;
			float z_dia_i = x_i.z + tao_dia * particles[i]->V.z;
			float x_dia_j = x_j.x + tao_dia * particles[j]->V.x;
			float z_dia_j = x_j.z + tao_dia * particles[j]->V.z;
			glm::vec3 dia_i = glm::vec3(x_dia_i, 0, z_dia_i);
			glm::vec3 dia_j = glm::vec3(x_dia_j, 0, z_dia_j);

			float constraint_value = 0;
			constraint_value = (distance(dia_i, dia_j) - collision_margin);

			if (constraint_value < 0)
			{
				float normal_x = (x_dia_i - x_dia_j) / (distance(dia_i, dia_j));
				float normal_z = (z_dia_i - z_dia_j) / (distance(dia_i, dia_j));
				glm::vec2 nor = glm::vec2(normal_x, normal_z);

				float gradient_i_x = normal_x;
				float gradient_i_z = normal_z;
				float gradient_j_x = -gradient_i_x;
				float gradient_j_z = -gradient_i_z;
				glm::vec3 gradient_i = glm::vec3(gradient_i_x, 0, gradient_i_z);
				glm::vec3 gradient_j = glm::vec3(gradient_j_x, 0, gradient_j_z);

				float stiff = k * exp(-(tao_hat * tao_hat) / tao0);

				float s = constraint_value /
					(particles[i]->inv_mass * (double)(norm(gradient_i) * norm(gradient_i)) +
						particles[j]->inv_mass * (double)(norm(gradient_j) * norm(gradient_j)));

				delta_X[0] = gradient_i * (-s) * w_i_coef * stiff;
				delta_X[1] = gradient_j * (-s) * w_j_coef * stiff;

				/*clamp(delta_X[0], max_acceleration);
				clamp(delta_X[1], max_acceleration);*/

				active = true;

				if (AVOIDANCE)
				{
					glm::vec3 modified_i = dia_i + delta_X[0];  // long range collision 해소된 위치
					glm::vec3 modified_j = dia_j + delta_X[1];  // margin (5% 때문에 2보다 좀 더 큰 값을 가짐)

					glm::vec3 modified_nor = (modified_i - modified_j) / norm(modified_i - modified_j);

					glm::vec3 d = (modified_i - hat_i) - (modified_j - hat_j);
					glm::vec3 dn = modified_nor * dot(d, modified_nor);
					glm::vec3 dt = d - dn;

					delta_X[0] += dt * particles[i]->V_pref * delta_t;
					delta_X[1] -= dt * particles[j]->V_pref * delta_t;

					clamp(delta_X[0], max_acceleration);
					clamp(delta_X[1], max_acceleration);

					active = true;
				}
			}
		}
	}
}
const float Powerlaw_Constraint::k = 1.5; // stiffness
const float Powerlaw_Constraint::tao0 = 20.00;
const float Powerlaw_Constraint::maxValue = 0.2; // delta_t * pref_speed
#pragma endregion

#pragma region Mesh
Mesh_Constraint::Mesh_Constraint(Simulation* sim, int i) : Constraint(sim, 1)
{
	this->i = i;
	this->w_i_coef =
		-sim->particles[i]->inv_mass /
		(sim->particles[i]->inv_mass + sim->particles[i]->inv_mass);
	this->indicies[0] = i;
	this->dist = 2.2f;
	this->contact_normal = glm::vec3(0.0, 0.0, 0.0);
	this->delta_t = sim->time_step;
	this->accel_limit = MAX_ACCEL * delta_t * delta_t;
}
void Mesh_Constraint::project(Particle** particles)
{
	if (!IS_FORMATION)
		return;

	delta_X[0] = VEC_ZERO;

	Particle* particle = particles[i];

	/*if (particle->is_leader)
		return;*/

	/*if (!particle->is_link)
	{
		return;
	}*/

	float f = distance(particle->X, particle->offset);
	float k = 1.0f;

	if (f > dist)
	{
		contact_normal = (particle->X - particle->offset);
		delta_X[0] += contact_normal * w_i_coef * k;
		clamp(delta_X[0], accel_limit);

		active = true;
	}
}
#pragma endregion
