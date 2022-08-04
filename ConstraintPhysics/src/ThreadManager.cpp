#include "ThreadManager.h"
#include <thread>
#include <atomic>

namespace phyz {

	template <typename T>
	void do_all_thread_func(const std::vector<T>& in_vector, void* params, std::function<void(T job, void* params)> action, std::atomic_int* next_job_index, std::atomic_int* num_jobs_done) {
		int my_job_index;
		while ((my_job_index = std::atomic_fetch_add(next_job_index, 1)) < in_vector.size()) {
			action(in_vector[my_job_index], params);
			std::atomic_fetch_add(num_jobs_done);
		}
	}

	template <typename T>
	void do_all(int n_threads, const std::vector<T>& in_vector, void* params, std::function<void(T job, void* params)> action) {
		std::atomic_int next_job_index = 0;
		std::atomic_int num_jobs_done = 0;

		std::vector<std::thread> threads(n_threads);
		for (int i = 0; i < n_threads; i++) {
			threads[i] = std::thread(do_all_thread_func, in_vector, params, action, &next_job_index, &num_jobs_done);
		}
		for (int i = 0; i < n_threads; i++) {
			threads[i].join();
		}
	}

}