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

		class JobStatus {
		public:
			bool isDone() { return is_done; }
			void waitUntilDone() {
				if (is_done) {
					return;
				}
				std::unique_lock<std::mutex> lk(wait_lock);
				wait_cv.wait(lk, [&]() -> bool { return is_done; });
			}
			friend class ThreadManager;
		private:
			void markDone() {
				wait_lock.lock();
				is_done = true;
				wait_lock.unlock();
				wait_cv.notify_all();
			}

			void init(uint32_t n_threads, uint32_t n_tasks) {
				is_done = false;
				next_task_index = 0;
				task_completed_count = 0;
			}
			
			std::atomic_bool is_done = false;
			std::mutex wait_lock;
			std::condition_variable wait_cv;

			std::atomic_int next_task_index;
			std::atomic_int task_completed_count;
		};

		~ThreadManager() {
			if (!terminated) {
				terminate_threads();
			}
		}

		void init(int thread_pool_size) {
			terminated = false;
			threads.reserve(thread_pool_size);
			for (int i = 0; i < thread_pool_size; i++) {
				threads.push_back(std::thread(worker_thread_func, &terminated, &jobs, &await_jobs_cv, &job_lock));
			}
		}

		void terminate_threads() {
			job_lock.lock();
			terminated = true;
			job_lock.unlock();

			await_jobs_cv.notify_all();
			for (std::thread& t : threads) {
				t.join();
			}
			threads.clear();
		}

		template<typename Func>
		void submit(const Func& action, JobStatus* status) {
			status->init(1, 1);

			job_lock.lock();
			jobs.push_back([action, status]() {
				action();
				status->markDone();
			});
			job_lock.unlock();

			await_jobs_cv.notify_one();
		}

		template <typename T, typename Func>
		void submit_do_all(uint32_t n_threads, const std::vector<T>* in_vector, const Func& action, JobStatus* status) {
			if (in_vector->size() == 0) {
				return;
			}

			uint32_t n_tasks = static_cast<uint32_t>(in_vector->size());
			status->init(n_threads, n_tasks);
			uint32_t jobs_for_task_set = std::min<uint32_t>(n_threads, n_tasks);
			
			job_lock.lock();
			for (uint32_t i = 0; i < jobs_for_task_set; i++) {
				jobs.push_back([status, in_vector, action]() {

					int my_index = -1;
					while ((my_index = std::atomic_fetch_add(&status->next_task_index, 1)) < in_vector->size()) {
						action(in_vector->at(my_index));

						int completed_count = 1 + std::atomic_fetch_add(&status->task_completed_count, 1);// +1 to get value after adding
						if (completed_count == in_vector->size()) {
							status->markDone();
						}
					}

				});
			}
			job_lock.unlock();

			await_jobs_cv.notify_all();
		}

		// common pattern
		template <typename T, typename Func>
		inline void await_do_all(uint32_t n_threads, const std::vector<T>* in_vector, const Func& action) {
			JobStatus s;
			submit_do_all(n_threads, in_vector, action, &s);
			s.waitUntilDone();
		}

	private:

		static void worker_thread_func(bool* terminated, std::vector<std::function<void()>>* jobs, std::condition_variable* await_jobs_cv, std::mutex* job_lock) {
			while (!(*terminated)) {

				std::unique_lock<std::mutex> lk(*job_lock);
				await_jobs_cv->wait(lk, [&]() -> bool { return *terminated || jobs->size() > 0; });

				if (*terminated) {
					return;
				}

				if (jobs->size() > 0) {
					std::function<void()> f = jobs->back();
					jobs->pop_back();
					lk.unlock();

					f();
				}
			}
		}

		bool terminated = false;
		std::vector<std::thread> threads;
		std::vector<std::function<void()>> jobs;
		std::mutex job_lock;
		std::condition_variable await_jobs_cv;

		int total_job_count;
		std::mutex done_lock;
		std::condition_variable done_cv;
		std::atomic_int num_jobs_exited;
		bool all_done;
	};
}