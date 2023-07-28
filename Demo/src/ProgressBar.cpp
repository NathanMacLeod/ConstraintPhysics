#include "ProgressBar.h"

ProgressBarState render_progress_bar(double percent, int width, bool done, ProgressBarState prev_state, float force_update_time, bool show_animation) {
	static std::vector<std::string> animation_c = { "|@-----<", ">-@----<", ">--@---<", ">---@--<", ">----@-<", ">-----@|", ">----@-<", ">---@--<", ">--@---<", ">-@----<" };
	ProgressBarState out = prev_state;
	out.use_status = IN_USE;
	auto now = std::chrono::system_clock::now();

	int current_percent_int = done ? 100 : 100 * percent;
	out.current_percent_int = current_percent_int;
	float time_since_last_render = prev_state.use_status != IN_USE ? -1 : std::chrono::duration<float>(now - prev_state.last_render_time).count();
	//should rerender progress bar?
	if (prev_state.use_status != IN_USE || time_since_last_render > force_update_time || (force_update_time == std::numeric_limits<float>::infinity() && current_percent_int != prev_state.current_percent_int) || done) {
		out.last_render_time = now;
		//clear any previous characters
		printf("%200s\r", "");

		//render loading animation
		if (prev_state.use_status != UNUSED && !done && show_animation) {
			int animation_state = prev_state.use_status == INITIALIZING ? 0 : (prev_state.animation_state + 1) % animation_c.size();
			out.animation_state = animation_state;
			printf("  %s ", animation_c[animation_state].c_str());
		}

		//render progress bar
		printf("[");
		int prog = done ? width : percent * width;
		for (int i = 0; i < width; i++) {
			if (i <= prog) {
				printf("#");
			}
			else {
				printf("-");
			}
		}
		printf("] %d%%", current_percent_int);

		//handle time projection & total time taken stuff
		if (prev_state.use_status != UNUSED) {

			//calculate and render projected time_remaining / total time
			if (!done && prev_state.use_status == IN_USE) {
				//compare to oldest data point in comparison history
				int comparison_indx = std::min<int>(prev_state.current_percent_int, COMPLETION_PROJECTION_HISTORY_SIZE - 1);

				float time_elapsed = std::chrono::duration<float>(now - prev_state.prev_percent_history[comparison_indx]).count();
				float percents_computed_since = (100 * percent - prev_state.current_percent_int + comparison_indx) / 100.0;

				float time_remaining = (1.0 - percent) * time_elapsed / percents_computed_since; //time per percent * percent remaining

				int remaining_minutes = time_remaining / 60;
				int remaining_seconds = time_remaining - remaining_minutes * 60;

				printf("         Time Remaining: %02d:%02d\r", remaining_minutes, remaining_seconds);
			}
			else if (done && prev_state.use_status == IN_USE) {
				float total_time = std::chrono::duration<float>(now - prev_state.start_time).count();

				int total_minutes = total_time / 60;
				int total_seconds = total_time - total_minutes * 60;

				printf("         Total Time: %02d:%02d\n", total_minutes, total_seconds);
			}
		}

		if (prev_state.use_status != IN_USE && !done) {
			printf("\r");
		}
		else if (prev_state.use_status != IN_USE && done) {
			printf("\n");
		}
	}

	//maintain history
	if (prev_state.use_status == INITIALIZING) {
		out.start_time = now;
	}

	bool new_percent_reached = prev_state.use_status == INITIALIZING || current_percent_int != prev_state.current_percent_int;
	if (new_percent_reached) {
		//drop oldest history point, add new one
		for (int i = 1; i < COMPLETION_PROJECTION_HISTORY_SIZE; i++) {
			out.prev_percent_history[i] = prev_state.prev_percent_history[i - 1];
		}
		out.prev_percent_history[0] = now;
	}

	return out;
}