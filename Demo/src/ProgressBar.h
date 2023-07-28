#pragma once
#include <chrono>
#include <vector>
#include <string>

const int COMPLETION_PROJECTION_HISTORY_SIZE = 3;

enum ProgressBarStateStatus { IN_USE, INITIALIZING, UNUSED };
struct ProgressBarState {
	ProgressBarStateStatus use_status;

	std::chrono::system_clock::time_point start_time;
	std::chrono::system_clock::time_point last_render_time;
	std::chrono::system_clock::time_point prev_percent_history[COMPLETION_PROJECTION_HISTORY_SIZE];

	int current_percent_int;
	int animation_state;
};

//ProgressBarState renderProgressBar(ProgressBarState prev_state, int bar_width, float current_percent, bool done)

ProgressBarState render_progress_bar(double percent, int width, bool done, ProgressBarState prev_state = ProgressBarState{ UNUSED }, float force_update_time = std::numeric_limits<float>::infinity(), bool show_animation = false);