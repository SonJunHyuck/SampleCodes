#include "utility.h"
#include "simulation.h"
#include "constraint.h"
#include "particle.h"
#include "path_planner.h"
#include "grid.h"
#include "wall.h"
#include "group.h"
#include "station.h"

Simulation::Simulation(int num_particles, int num_constraints, float time_step, char* out_path)
{
	this->num_particles = num_particles;
	this->num_particles_half = num_particles * 0.5f;
	this->time_step = time_step;
	this->particles = (Particle**)malloc(sizeof(void*) * num_particles);
	this->num_constraints = 0;
	this->constraints = NULL;
	this->collision_map =
		std::unordered_map<unsigned long long, Constraint*>();
	this->collision_upper_trig_arr = NULL;
	this->powerlaw_upper_trig_arr = NULL;
	this->stability_upper_trig_arr = NULL;
	this->distance_trig_arr = NULL;
	this->grid = new Grid(num_particles, CELL_SIZE,
		glm::vec3(GRID_MIN_X, 0, GRID_MIN_Z),
		glm::vec3(GRID_MAX_X, 0, GRID_MAX_Z));
	this->stability_grid =
		new Grid(num_particles, CELL_SIZE, //2.66, // 5.66, //7.66,
			glm::vec3(GRID_MIN_X, 0, GRID_MIN_Z),
			glm::vec3(GRID_MAX_X, 0, GRID_MAX_Z));

	this->planner = new PathPlanner(num_particles, this->particles);
	this->step_no = 1;
	this->friction_constraint_stiffness = 0.22f;
	this->num_walls = 0;
	this->walls = NULL;
	this->num_groups = IS_SINGLE ? 1 : GROUP_COUNT;
	this->groups = (Group**)malloc(sizeof(void*) * num_groups);

	this->stations = std::vector<Station>();

	this->start = GetSeconds();
}

Simulation::~Simulation() {
	for (int i = 0; i < num_particles; i++) {
		delete particles[i];
	}
	for (int i = 0; i < num_constraints; i++) {
		delete constraints[i];
	}

	if (walls != NULL) {
		for (int i = 0; i < num_walls; i++) {
			delete walls[i];
		}
	}

	int trig_len = 1 + (num_particles * (num_particles + 1) / 2);
	for (int i = 0; i < num_particles; i++) {
		for (int j = 0; j < num_particles; j++) {
			if (i < j) {
				int r = i;
				int c = j;
				int t_idx = (num_particles * r) + c - (r * (r + 1) * 0.5);
				if (collision_upper_trig_arr != NULL) {
					delete collision_upper_trig_arr[t_idx];
				}
				if (powerlaw_upper_trig_arr != NULL) {
					delete powerlaw_upper_trig_arr[t_idx];
				}
				if (stability_upper_trig_arr != NULL) {
					delete stability_upper_trig_arr[t_idx];
				}
			}
		}
	}

	free(constraints);
	free(particles);
	free(collision_upper_trig_arr);
	free(powerlaw_upper_trig_arr);
	delete planner;
	delete grid;
}

void Simulation::calc_constraint_stiffness(int n) {
	// 1.-(1.-0.25)**(4./6)
	friction_constraint_stiffness =
		1.0f - powf(1.0f - friction_constraint_stiffness, (1.0f / n));
}
 
void Simulation::update_predicted_position()
{
	// predict + correction = next pos
	for (int i = 0; i < num_particles; i++)
	{
		if (particles[i]->Delta_x_ctr > 0)
		{
			particles[i]->X_pred.x +=
				(ALPHA * particles[i]->Delta_x.x / particles[i]->Delta_x_ctr);
			particles[i]->X_pred.z +=
				(ALPHA * particles[i]->Delta_x.z / particles[i]->Delta_x_ctr);

			// clamp
			if (false)
			{
				float maxValue = 0.069;
				float length_d_i = distance(particles[i]->X_pred, particles[i]->X);
				
				if (length_d_i > maxValue)
				{
					float mult = (maxValue / length_d_i);
					particles[i]->X_pred.x =
						particles[i]->X.x +
						(particles[i]->X_pred.x - particles[i]->X.x) * mult;
					particles[i]->X_pred.z =
						particles[i]->X.z +
						(particles[i]->X_pred.z - particles[i]->X.z) * mult;
				}
			}

			particles[i]->Delta_x = VEC_ZERO;
			particles[i]->Delta_x_ctr = 0;
		}
	}
}

void Simulation::stabilization()
{
	stability_grid->update_stability(particles);

	for (int i = 0; i < 1; i++) 
	{
		for (int i = 0; i < num_particles; i++) 
		{
			particles[i]->Delta_x = VEC_ZERO;
			particles[i]->Delta_x_ctr = 0;
		}

		// friction constraints
		for (int i = 0; i < num_particles; i++) {
			// iterate over adjacent cells
			for (int x = -1; x <= 1; x++) {
				int cur_x = particles[i]->cell_x + x;
				if (cur_x >= 0 && cur_x < stability_grid->num_cols) {
					for (int z = -1; z <= 1; z++) {
						int cur_z = particles[i]->cell_z + z;
						if (cur_z >= 0 && cur_z < stability_grid->num_rows) {
							int cell_id =
								particles[i]->cell_id + x + (z * stability_grid->num_rows);
							if (stability_grid->grid_counters[cell_id] > 0) {
								for (int idx = 0;
									idx < stability_grid->grid_counters[cell_id]; idx++) {
									int j = stability_grid->grid_cells[cell_id][idx];
									if (i < j) // so only do collision once
									{
										int t_idx = (num_particles * i) + j - (i * (i + 1) * 0.5);
										stability_upper_trig_arr[t_idx]->project(particles);
									}
								}
							}
						}
					}
				}
			}
		}

		// traverse friction constraints to accumalte deltas
		for (int i = 0; i < num_particles; i++) {
			// iterate over adjacent cells
			for (int x = -1; x <= 1; x++) {
				int cur_x = particles[i]->cell_x + x;
				if (cur_x >= 0 && cur_x < stability_grid->num_cols) {
					for (int z = -1; z <= 1; z++) {
						int cur_z = particles[i]->cell_z + z;
						if (cur_z >= 0 && cur_z < stability_grid->num_rows) {
							int cell_id =
								particles[i]->cell_id + x + (z * stability_grid->num_rows);
							if (stability_grid->grid_counters[cell_id] > 0) {
								for (int idx = 0;
									idx < stability_grid->grid_counters[cell_id]; idx++) {
									int j = stability_grid->grid_cells[cell_id][idx];
									if (i < j) // so only do collision once
									{
										int t_idx = (num_particles * i) + j - (i * (i + 1) * 0.5);
										if (stability_upper_trig_arr[t_idx]->active) {
											for (int ctr = 0;
												ctr <
												stability_upper_trig_arr[t_idx]->num_particles;
												ctr++) {
												int p_idx =
													stability_upper_trig_arr[t_idx]->indicies[ctr];
												particles[p_idx]->Delta_x.x +=
													stability_upper_trig_arr[t_idx]->delta_X[ctr].x;
												particles[p_idx]->Delta_x.z +=
													stability_upper_trig_arr[t_idx]->delta_X[ctr].z;
												particles[p_idx]->Delta_x_ctr++;
											}
											stability_upper_trig_arr[t_idx]->active = false;
										}
									}
								}
							}
						}
					}
				}
			}
		}

		for (int i = 0; i < num_particles; i++) {
			if (particles[i]->Delta_x_ctr > 0) {
				float dx =
					ALPHA * particles[i]->Delta_x.x / particles[i]->Delta_x_ctr;
				float dz =
					ALPHA * particles[i]->Delta_x.z / particles[i]->Delta_x_ctr;
				particles[i]->X_pred.x += dx;
				particles[i]->X_pred.z += dz;
				particles[i]->X.x += dx;
				particles[i]->X.z += dz;
			}
		}
	}
}

void Simulation::project_longrange_constraints()
{
	// init to 0
	for (int i = 0; i < num_particles; i++)
	{
		particles[i]->Delta_x = VEC_ZERO;
		particles[i]->Delta_x_ctr = 0;
	}

	// grid 기반으로 자신이 존재하는 cell과 인접한 cell에 있는 파티클들에 대한 constraint
	for (int i = 0; i < num_particles; i++)
	{
		// iterate over adjacent cells (인접 cell들 확인)
		for (int x = -2; x <= 2; x++)
		{
			int cur_x = particles[i]->cell_x + x;
			if (cur_x >= 0 && cur_x < grid->num_cols)
			{
				for (int z = -2; z <= 2; z++)
				{
					int cur_z = particles[i]->cell_z + z;
					if (cur_z >= 0 && cur_z < grid->num_rows)
					{
						int cell_id = particles[i]->cell_id + x + (z * grid->num_rows);
						// 해당 cell에 particle이 존재하고 있다면
						if (grid->grid_counters[cell_id] > 0)
						{
							// 해당 cell에 존재하는 파티클 수 만큼
							for (int idx = 0; idx < grid->grid_counters[cell_id]; idx++)
							{
								int j = grid->grid_cells[cell_id][idx];
								if (i < j) // so only do collision once
								{
									int t_idx = (num_particles * i) + j - (i * (i + 1) * 0.5);
									powerlaw_upper_trig_arr[t_idx]->project(particles);
								}
							}
						}
					}
				}
			}
		}
	}

	// traverse friction constraints to accumalte deltas
	for (int i = 0; i < num_particles; i++) {
		// iterate over adjacent cells
		for (int x = -2; x <= 2; x++)
		{
			int cur_x = particles[i]->cell_x + x;
			if (cur_x >= 0 && cur_x < grid->num_cols)
			{
				for (int z = -2; z <= 2; z++)
				{
					int cur_z = particles[i]->cell_z + z;
					if (cur_z >= 0 && cur_z < grid->num_rows)
					{
						int cell_id = particles[i]->cell_id + x + (z * grid->num_rows);
						if (grid->grid_counters[cell_id] > 0)
						{
							for (int idx = 0; idx < grid->grid_counters[cell_id]; idx++)
							{
								int j = grid->grid_cells[cell_id][idx];
								if (i < j) // so only do collision once
								{
									int t_idx = (num_particles * i) + j - (i * (i + 1) * 0.5);
									if (powerlaw_upper_trig_arr[t_idx]->active)
									{
										for (int ctr = 0;
											ctr < powerlaw_upper_trig_arr[t_idx]->num_particles;
											ctr++)
										{
											int p_idx =
												powerlaw_upper_trig_arr[t_idx]->indicies[ctr];
											particles[p_idx]->Delta_x.x +=
												powerlaw_upper_trig_arr[t_idx]->delta_X[ctr].x;
											particles[p_idx]->Delta_x.z +=
												powerlaw_upper_trig_arr[t_idx]->delta_X[ctr].z;
											particles[p_idx]->Delta_x_ctr++;
										}
										powerlaw_upper_trig_arr[t_idx]->active = false;
									}
								}
							}
						}
					}
				}
			}
		}
	}

	// 5. predict(Particle) <- correction(Constraint) 
	update_predicted_position();
}

void Simulation::project_constraints()
{
	// init to 0
	for (int i = 0; i < num_particles; i++) {
		particles[i]->Delta_x.x = 0.0;
		particles[i]->Delta_x.z = 0.0;
		particles[i]->Delta_x_ctr = 0;
	}

	// friction constraints (projection)
	for (int i = 0; i < num_particles; i++) {
		// iterate over adjacent cells
		for (int x = -1; x <= 1; x++) {
			int cur_x = particles[i]->cell_x + x;
			if (cur_x >= 0 && cur_x < grid->num_cols) {
				for (int z = -1; z <= 1; z++) {
					int cur_z = particles[i]->cell_z + z;
					if (cur_z >= 0 && cur_z < grid->num_rows) {
						int cell_id = particles[i]->cell_id + x + (z * grid->num_rows);
						if (grid->grid_counters[cell_id] > 0) {
							for (int idx = 0; idx < grid->grid_counters[cell_id]; idx++) {
								int j = grid->grid_cells[cell_id][idx];
								if (i < j) // so only do collision once
								{
									// collision_map[i * num_particles + j]->project(particles);
									int t_idx = (num_particles * i) + j - (i * (i + 1) * 0.5);
									collision_upper_trig_arr[t_idx]->project(particles);
									// powerlaw_upper_trig_arr[t_idx]->project(particles);
									// stability_upper_trig_arr[t_idx]->project(particles);
								}
							}
						}
					}
				}
			}
		}
	}

	// traverse friction constraints to accumalte deltas
	for (int i = 0; i < num_particles; i++)
	{
		// iterate over adjacent cells
		for (int x = -1; x <= 1; x++)
		{
			int cur_x = particles[i]->cell_x + x;
			if (cur_x >= 0 && cur_x < grid->num_cols)
			{
				for (int z = -1; z <= 1; z++)
				{
					int cur_z = particles[i]->cell_z + z;
					if (cur_z >= 0 && cur_z < grid->num_rows)
					{
						int cell_id = particles[i]->cell_id + x + (z * grid->num_rows);
						if (grid->grid_counters[cell_id] > 0)
						{
							for (int idx = 0; idx < grid->grid_counters[cell_id]; idx++)
							{
								int j = grid->grid_cells[cell_id][idx];
								if (i < j) // so only do collision once
								{
									int t_idx = (num_particles * i) + j - (i * (i + 1) * 0.5);
									if (collision_upper_trig_arr[t_idx]->active)
									{
										for (int ctr = 0;
											ctr < collision_upper_trig_arr[t_idx]->num_particles;
											ctr++)
										{
											int p_idx =
												collision_upper_trig_arr[t_idx]->indicies[ctr];
											particles[p_idx]->Delta_x.x +=
												collision_upper_trig_arr[t_idx]->delta_X[ctr].x;
											particles[p_idx]->Delta_x.z +=
												collision_upper_trig_arr[t_idx]->delta_X[ctr].z;
											particles[p_idx]->Delta_x_ctr++;
										}
										collision_upper_trig_arr[t_idx]->active = false;
									}

									/*
									if(powerlaw_upper_trig_arr[t_idx]->active)
									{
										for(int ctr=0;
											ctr<powerlaw_upper_trig_arr[t_idx]->num_particles
											;ctr++)
										{
											int
									p_idx=powerlaw_upper_trig_arr[t_idx]->indicies[ctr];
											particles[p_idx]->Delta_x.x +=
									powerlaw_upper_trig_arr[t_idx]->delta_X[ctr].x;
											particles[p_idx]->Delta_x.y +=
									powerlaw_upper_trig_arr[t_idx]->delta_X[ctr].y;
											particles[p_idx]->Delta_x_ctr++;
										}
										powerlaw_upper_trig_arr[t_idx]->active=false;
									}
									*/
								}
							}
						}
					}
				}
			}
		}
	}
	update_predicted_position();

	// wall + ground + mesh constraints
	for (int i = 0; i < num_constraints; i++) {
		constraints[i]->project(particles);
		if (constraints[i]->active) {
			for (int j = 0; j < constraints[i]->num_particles; j++) {
				int idx = constraints[i]->indicies[j];
				particles[idx]->Delta_x.x += constraints[i]->delta_X[j].x;
				particles[idx]->Delta_x.z += constraints[i]->delta_X[j].z;
				particles[idx]->Delta_x_ctr++;
			}
			constraints[i]->active = false;
		}
	}
	update_predicted_position();
}

void Simulation::do_time_step()
{
	for (int i = 0; i < num_groups; i++)
	{
		groups[i]->update_path();
	}

	// 1. (init Pos, blending velocity) -> 4.1
	for (int i = 0; i < num_particles; i++)
	{
		particles[i]->V_prev = particles[i]->V;

		planner->calc_velocity(i);  // V.length = V_pref (calc velocity from planner)
		particles[i]->V *= 0.9999f;
		particles[i]->V = KSI * planner->velocity_buffer[i] + (1 - KSI) * particles[i]->V;
		particles[i]->X_pred += time_step * particles[i]->V;
	}

	// 2. searching neighboring
	grid->update(particles);

	for (int i = 0; i < num_particles; i++)
	{
		Particle* p = particles[i];

		// 위험지역에서 대열 해제
		if (grid->grid_safty[p->cell_id] == MIXED_AREA)
			p->is_link = false;
		else
			p->is_link = true;
	}

	// 3. long_range constraint (4.4, 4.5)
	project_longrange_constraints();

	// 4. Short range Destination
	for (int i = 1; i < (ITER_COUNT + 1); i++)
	{
		calc_constraint_stiffness(i);
		project_constraints();  // 4.2 (short)
	}

	// 5. Update Velocity
	for (int i = 0; i < num_particles; i++)
	{
		particles[i]->dummy_X = particles[i]->X;
		float dx = particles[i]->X_pred.x - particles[i]->X.x;
		float dz = particles[i]->X_pred.z - particles[i]->X.z;

		particles[i]->V.x = dx / time_step;  // 속도 조합 단계에서 time_step 곱해주기 때문에 지금 나눠줌
		particles[i]->V.z = dz / time_step;

		if (FTL)
		{
			// + s_damping * (-d_ / time_step)
			// d_는 다음 Particle의 conrrection
			glm::vec3 ftl = glm::vec3(0, 0, 0);

			glm::vec3 d_next = groups[particles[i]->group_id]->leader->Delta_x;
			float s_damping = 0.8f;

			ftl = (groups[particles[i]->group_id]->leader->Delta_x * s_damping) / time_step;

			particles[i]->V += ftl;
		}

		if (STEERING)
		{
			// V에는 이미 time_step이 / 연산 돼있기 때문에 값이 충분히 큼 -> desired_V도 충분히 크게 만들어주기 위해
			float v_length = norm(particles[i]->V);
			glm::vec3 desired_V = (particles[i]->goal - particles[i]->X) / time_step;
			glm::vec3 steering = (desired_V - particles[i]->V);
			clamp(steering, MAX_ACCEL * time_step);

			particles[i]->V += steering;

			clamp(particles[i]->V, v_length);
		}

		if (COHESION)
		{
			// viscosity 4.3
			float h = 7.0f; //15.0f;  // 구심력이 작용하는 거리
			float c = 217.0f;// *3.0f;  // 구심력의 크기
			float weight = 0;
			glm::vec3 cohesion = glm::vec3(0, 0, 0);

			for (int j = 0; j < num_particles; j++)
			{
				if (particles[i]->group_id == particles[j]->group_id)
				{
					float r = distance(particles[i]->X, particles[j]->X);

					if (r <= h && r >= 0)
					{
						// 두 파티클이 가까울 수록 weight up
						weight = (c * pow((h * h - r * r), 3)) / (double)(64.0f * _M_PI * pow(h, 9));

						cohesion += (particles[i]->V - particles[j]->V) * weight;
					}
				}
			}

			// 인력 (원래는 더하기로 나와있지만, 빼는것이 인력이라고 판단)
			particles[i]->V -= cohesion;
		}

		particles[i]->X = particles[i]->X_pred;
	}

	step_no++;
}
