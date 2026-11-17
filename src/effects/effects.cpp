#include "effects.hpp"

using namespace cgp;

// Update a transition_structure before starting a new transition
void effect_transition_start(effect_transition_structure& effect_transition, character_structure& character, std::string const& destination_anim)
{
	effect_transition.source_anim = character.current_animation_name;
	effect_transition.timer_source_anim = character.timer;

	effect_transition.destination_anim = destination_anim;
	effect_transition.timer_destination_anim.event_period = character.animated_model.animation.at(destination_anim).time_max;
	effect_transition.timer_destination_anim.t_periodic=0;
	effect_transition.timer_destination_anim.start();

	effect_transition.timer_completion.t=0;
	effect_transition.timer_completion.scale = 1.0f/effect_transition.transition_time; // force the timer to go from 0 to 1 during the transition_time
	effect_transition.timer_completion.start();
	effect_transition.active = true;

	character.current_animation_name = destination_anim;
}

// Check if a transition need to be de-activated if it is fully completed
void effect_transition_stop_if_completed(effect_transition_structure& transition, character_structure& character) {
	if(transition.timer_completion.t >= 1.0f) {
		character.set_current_animation(transition.destination_anim);
		character.timer = transition.timer_destination_anim;
		transition.active = false;
	}
}
void effect_transition_compute(effect_transition_structure& effect_transition, character_structure& character)
{

	effect_transition_structure& transition = effect_transition;
	animated_model_structure& model = character.animated_model;

	// Compute the skeleton from the source animation
	model.set_skeleton_from_animation(transition.source_anim, transition.timer_source_anim.t_periodic);
	numarray<mat4> joint_source_anim_local = model.skeleton.joint_matrix_local;
	numarray<mat4> joint_source_anim_global = model.skeleton.joint_matrix_global;

	// Compute the skeleton from the destination animation
	model.set_skeleton_from_animation(transition.destination_anim, transition.timer_destination_anim.t_periodic);
	numarray<mat4> joint_destination_anim_local = model.skeleton.joint_matrix_local;
	numarray<mat4> joint_destination_anim_global = model.skeleton.joint_matrix_global;

	float alpha_completion = transition.timer_completion.t;

	float const alpha = clamp(alpha_completion, 0.0f, 1.0f); // just to be sure
	int const N = model.skeleton.size();

	for (int k = 0; k < N; k++)
	{
		affine_rt const a0 = affine_rt::from_matrix(joint_source_anim_local[k]);
		affine_rt const a1 = affine_rt::from_matrix(joint_destination_anim_local[k]);

		vec3 const t = (1.0f - alpha) * a0.translation + alpha * a1.translation;
		rotation_transform const r = rotation_transform::slerp(a0.rotation, a1.rotation, alpha);

		model.skeleton.joint_matrix_local[k] = affine_rt(r, t).matrix();
	}

	model.skeleton.update_joint_matrix_local_to_global();

	transition.timer_source_anim.update();
	transition.timer_destination_anim.update();
	transition.timer_completion.update();
	character.timer = transition.timer_destination_anim;
}

// This function implements the change of animation when we press or release the UP key during a walk effect
void effect_walking_keyboard_event(effect_transition_structure& effect_transition, character_structure& character, cgp::input_devices const& inputs, effect_walking_structure const& effect_walking)
{
	// If we just press the key up (or W), start the Walk cycle animation
	if(inputs.keyboard.last_action.is_pressed(GLFW_KEY_UP) || inputs.keyboard.last_action.is_pressed(GLFW_KEY_W)) {
		effect_transition_start(effect_transition, character, effect_walking.walk_anim_name);
	}
	// If we just release the key up (or W), start the Idle cycle animation
	if(inputs.keyboard.last_action.is_released(GLFW_KEY_UP) || inputs.keyboard.last_action.is_released(GLFW_KEY_W)) {
		effect_transition_start(effect_transition, character, effect_walking.idle_anim_name);
	}

}

// This function implements the change of position of the character when a directional key is pressed
// This function is called at every frame when the walk effect is active
void effect_walking(effect_walking_structure& effect_walking, character_structure& character, cgp::input_devices const& inputs, effect_transition_structure const& effect_transition)
{
	float const dt = inputs.time_interval;
	float const speed = 4.0f;
	float const angular_speed = 3.0f;

	// Left and right orientation inputs
	if (inputs.keyboard.is_pressed(GLFW_KEY_LEFT) || inputs.keyboard.is_pressed(GLFW_KEY_A))
		effect_walking.root_angle += angular_speed * dt;
	if (inputs.keyboard.is_pressed(GLFW_KEY_RIGHT) || inputs.keyboard.is_pressed(GLFW_KEY_D))
		effect_walking.root_angle -= angular_speed * dt;

	// Get orientation transformation
	// {0, 1, 0} because floor is the xz plane so the character can only rotate on the y axis
	rotation_transform const R = rotation_axis_angle({0, 1, 0}, effect_walking.root_angle);

	// Move forward input
	if (inputs.keyboard.is_pressed(GLFW_KEY_UP) || inputs.keyboard.is_pressed(GLFW_KEY_W))
		effect_walking.root_position += speed * dt * (R * vec3(0, 0, 1));

	effect_place_root(character.animated_model.skeleton, effect_walking.root_position, effect_walking.root_angle);

	(void)effect_transition; // I do not know why but we did not use it...
}

void effect_place_root(skeleton_structure& skeleton, vec3 const& position, float angle)
{
	mat4& M = skeleton.joint_matrix_local[0];
	rotation_transform const R = rotation_axis_angle({0, 1, 0}, angle);

	// Remove the horizontal drift of the animation: only the agent position moves the character
	vec3 t = M.get_block_translation();
	t.x = 0.0f;
	// t.y: no need to change it for now
	t.z = 0.0f;

	// Move the (reference) character
	M.set_block_linear(R.matrix() * M.get_block_linear());
	M.set_block_translation(R * t + position);
	skeleton.update_joint_matrix_local_to_global();
}


// Rotate the joint of the head to look toward the given objective position (if possible)
void effect_rotate_head_toward_objective_position(skeleton_structure& skeleton, int joint_head_index, vec3 const& objective_position)
{
	mat4 const& G = skeleton.joint_matrix_global[joint_head_index];
	vec3 const to_target = objective_position - G.get_block_translation(); // vector direction to the target
	float const length = norm(to_target);
	if (length < 1e-5f)
		return;

	// Direction towards the target
	// Get inverse with transpose: from global space to head space
	vec3 const d = transpose(G.get_block_linear()) * (to_target / length);

	float yaw = std::atan2(d.x, d.z);
	float pitch = std::asin(clamp(d.y, -1.0f, 1.0f));
	// Define plausible head orientation limits
	yaw = clamp(yaw, -1.2f, 1.2f);
	pitch = clamp(pitch, -0.6f, 0.6f);

	// Combine rotations (two degrees of freedom)
	mat3 const R = (rotation_axis_angle({0, 1, 0}, yaw) * rotation_axis_angle({1, 0, 0}, -pitch)).matrix();

	// Apply rotation
	mat4& L = skeleton.joint_matrix_local[joint_head_index];
	L.set_block_linear(L.get_block_linear() * R);
	skeleton.update_joint_matrix_local_to_global();
}

void effect_dizzy_stars_initialize(effect_dizzy_stars_structure& effect, std::string const& directory)
{
	auto elements = mesh_load_file_obj_advanced(directory, "star.obj");
	effect.star = mesh_obj_advanced_loader::convert_to_mesh_drawable(elements);
}

void effect_dizzy_stars_draw(effect_dizzy_stars_structure& effect, skeleton_structure const& skeleton,
							 int joint_head_index, float time, float appear,
							 environment_generic_structure const& environment)
{
	vec3 const head = skeleton.joint_matrix_global[joint_head_index].get_block_translation();

	for (int k = 0; k < effect.star_count; k++) {
		// Stars are evenly spread on a circle that rotates around the head
		float const angle = effect.orbit_speed * time + 2.0f * Pi * k / effect.star_count;
		vec3 const position = head + vec3(effect.radius * std::cos(angle),
									effect.height + 0.03f * std::sin(3.0f * time + k),
									effect.radius * std::sin(angle));

		// Draw all the stars
		for (auto& part : effect.star) {
			part.model.translation = position;
			part.model.scaling = effect.star_scale * appear;
			part.model.rotation = rotation_axis_angle({0, 1, 0}, effect.spin_speed * time + k);
			draw(part, environment);
		}
	}
}
