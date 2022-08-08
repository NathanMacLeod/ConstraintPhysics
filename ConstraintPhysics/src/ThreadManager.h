#pragma once
#include <vector>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <chrono>

namespace phyz {
	class ThreadManager {
	public:
		
		~ThreadManager() {
			if (!terminated) {
				terminate_threads();
			}
		}

		void init(int thread_pool_size) {
			terminated = false;
			threads.reserve(thread_pool_size);
			for (int i = 0; i < thread_pool_size; i++) {
				threads.push_back(std::thread(worker_thread_func, &terminated, &jobs, &sleep_cv, &sleep_lock, &job_lock));
			}
		}

		void terminate_threads() {
			terminated = true;
			sleep_cv.notify_all();
			for (std::thread& t : threads) {
				t.join();
			}
			threads.clear();
		}
		
		template <typename T>
		void do_all(int n_threads, const std::vector<T>& in_vector, std::function<void(T job)> action) {
			if (in_vector.size() == 0) {
				return;
			}

			std::mutex done_lock;
			std::condition_variable done_cv;

			std::atomic_int next_index = 0;
			std::atomic_int num_indices_done = 0;
			std::atomic_int num_jobs_exited = 0;

			int n_jobs = std::min<double>(n_threads, in_vector.size());
			job_lock.lock();
			for (int i = 0; i < n_jobs; i++) {
				jobs.push_back([&]() {

					int my_index = -1;
					while ((my_index = std::atomic_fetch_add(&next_index, 1)) < in_vector.size()) {
						action(in_vector[my_index]);
						std::atomic_fetch_add(&num_indices_done, 1);
					}

					bool last_exiting = std::atomic_fetch_add(&num_jobs_exited, 1) + 1 == n_jobs;
					if (last_exiting) {
						done_cv.notify_one();
					}

				});
			}
			job_lock.unlock();
			sleep_cv.notify_all();

			std::unique_lock<std::mutex> lk(done_lock);
			done_cv.wait(lk, [&]() -> bool { return num_jobs_exited.load() >= n_jobs; });
			//while (!(num_jobs_exited.load() >= n_jobs));

		}
	private:

		static void wait(std::mutex* lock, std::condition_variable* cv, std::function<bool()> done) {
			std::unique_lock<std::mutex> lk(*lock);
			cv->wait(lk, done);
		}

		static void worker_thread_func(bool* terminated, std::vector<std::function<void()>>* jobs, std::condition_variable* sleep_cv, std::mutex* sleep_lock, std::mutex* job_lock) {
			while (!(*terminated)) {

				std::unique_lock<std::mutex> lk(*sleep_lock);
				sleep_cv->wait(lk, [&]() -> bool { return *terminated || jobs->size() > 0; });
				//while (!(*terminated || jobs->size() > 0));

				if (*terminated) {
					return;
				}

				job_lock->lock();
				if (jobs->size() > 0) {
					std::function<void()> f = jobs->back();
					jobs->pop_back();

					job_lock->unlock();
					f();	
				}
				else {
					job_lock->unlock();
				}

			}
		}

		bool terminated = false;
		std::vector<std::thread> threads;
		std::vector<std::function<void()>> jobs;
		std::mutex job_lock;
		std::mutex sleep_lock;
		std::condition_variable sleep_cv;
	};
}