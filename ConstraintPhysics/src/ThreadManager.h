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
		
		template <typename T, typename Func>
		void do_all(int n_threads, const std::vector<T>& in_vector, Func action) {
			if (in_vector.size() == 0) {
				return;
			}

			std::mutex done_lock;
			std::condition_variable done_cv;

			std::atomic_int next_index = 0;
			std::atomic_int num_indices_done = 0;
			std::atomic_int num_jobs_exited = 0;
			bool all_done = false;

			std::unique_lock<std::mutex> lk(done_lock);
			int n_jobs = std::min<double>(n_threads, in_vector.size());
			job_lock.lock();
			for (int i = 0; i < n_jobs; i++) {
				jobs.push_back([&, n_jobs]() {

					int my_index = -1;
					while ((my_index = std::atomic_fetch_add(&next_index, 1)) < in_vector.size()) {
						action(in_vector[my_index]);
						std::atomic_fetch_add(&num_indices_done, 1);
					}

					bool last_exiting = std::atomic_fetch_add(&num_jobs_exited, 1) + 1 == n_jobs;
					if (last_exiting) {
						done_lock.lock();
						all_done = true;
						done_lock.unlock();
						done_cv.notify_one();
						
					}

				});
			}
			job_lock.unlock();
			sleep_cv.notify_all();

			done_cv.notify_all();

			
			wait(&lk, &done_cv, [&]() -> bool { return all_done; });

		}
	private:

		static void wait(std::unique_lock<std::mutex>* lk, std::condition_variable* cv, std::function<bool()> done) {
			while (!done()) {
				cv->wait(*lk, done);
			}
		}

		static void worker_thread_func(bool* terminated, std::vector<std::function<void()>>* jobs, std::condition_variable* sleep_cv, std::mutex* sleep_lock, std::mutex* job_lock) {
			while (!(*terminated)) {

				std::unique_lock<std::mutex> lk(*sleep_lock);
				wait(&lk, sleep_cv, [&]() -> bool { return *terminated || jobs->size() > 0; });
				lk.unlock();
				
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