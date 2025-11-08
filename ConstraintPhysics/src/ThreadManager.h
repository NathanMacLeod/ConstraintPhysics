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
				std::unique_lock<std::mutex> lk(wait_lock);
				if (is_done) {
					return;
				}
				wait(&lk, &wait_cv, [&]() -> bool { return is_done; });
			}
			void markDone() {
				wait_lock.lock();
				is_done = true;
				wait_lock.unlock();
				wait_cv.notify_all();
			}
		private:
			std::atomic_bool is_done = false;
			std::mutex wait_lock;
			std::condition_variable wait_cv;
		};

	private:
		struct TastSetManagement {
			TastSetManagement(uint32_t n_threads, uint32_t num_tasks, JobStatus* status_update)
				: next_index(0), jobs_for_task_set(std::min<int>(n_threads, num_tasks)), status_update(status_update), index_completed_count(0)
			{}

			TastSetManagement(const TastSetManagement& t)
				: next_index(t.next_index.load()), jobs_for_task_set(t.jobs_for_task_set), status_update(t.status_update), index_completed_count(t.index_completed_count.load())
			{}

			std::atomic_int next_index;
			std::atomic_int index_completed_count;
			uint32_t jobs_for_task_set;
			JobStatus* status_update;
		};
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
				threads.push_back(std::thread(worker_thread_func, &terminated, &jobs, &start_working, &sleep_cv, &sleep_lock, &job_lock));
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

		template<typename Func>
		void enqueue_task(const Func& action, JobStatus* status_update = nullptr) {
			start_working = false;
			task_sets.push_back(TastSetManagement(1, 1, status_update));
			int task_index = task_sets.size() - 1;

			job_lock.lock();
			jobs.push_back([&, task_index, action]() {

				action();
				if (task_sets[task_index].status_update != nullptr) {
					task_sets[task_index].status_update->markDone();
				}

				bool last_exiting = std::atomic_fetch_add(&num_jobs_exited, 1) + 1 == total_job_count;
				if (last_exiting) {
					done_lock.lock();
					all_done = true;
					done_lock.unlock();
					done_cv.notify_one();
				}

				});
			job_lock.unlock();
		}

		template <typename T, typename Func>
		void enqueue_do_all_tasks(uint32_t n_threads, const std::vector<T>* in_vector, const Func& action, JobStatus* status_update = nullptr) {
			start_working = false;
			if (in_vector->size() == 0) {
				return;
			}

			task_sets.push_back(TastSetManagement(n_threads, static_cast<uint32_t>(in_vector->size()), status_update));
			int task_index = static_cast<int>(task_sets.size()) - 1;

			job_lock.lock();
			for (uint32_t i = 0; i < task_sets[task_index].jobs_for_task_set; i++) {
				jobs.push_back([&, task_index, in_vector, action]() {

					int my_index = -1;
					while ((my_index = std::atomic_fetch_add(&task_sets[task_index].next_index, 1)) < in_vector->size()) {
						action(in_vector->at(my_index), my_index);


						int completed_count = 1 + std::atomic_fetch_add(&task_sets[task_index].index_completed_count, 1);// +1 to get value after adding
						if (task_sets[task_index].status_update != nullptr && completed_count == in_vector->size()) {
							task_sets[task_index].status_update->markDone();
						}
					}

					bool last_exiting = std::atomic_fetch_add(&num_jobs_exited, 1) + 1 == total_job_count;
					if (last_exiting) {
						done_lock.lock();
						all_done = true;
						done_lock.unlock();
						done_cv.notify_one();

					}

				});
			}
			job_lock.unlock();
		}

		void execute_jobs() {
			start_working = false;
			all_done = false;
			total_job_count = 0;
			num_jobs_exited = 0;

			for (const TastSetManagement& t : task_sets) total_job_count += t.jobs_for_task_set;
			if (total_job_count == 0) return;

			std::unique_lock<std::mutex> lk(done_lock);
			start_working = true;
			sleep_cv.notify_all();
			done_cv.notify_all();
			wait(&lk, &done_cv, [&]() -> bool { return all_done; });

			start_working = false;
			task_sets.clear();
		}

		template <typename T, typename Func>
		void do_all(uint32_t n_threads, const std::vector<T>& in_vector, const Func& action) {
			if (in_vector.size() == 0) {
				return;
			}

			start_working = false;
			std::mutex done_lock;
			std::condition_variable done_cv;

			std::atomic_int next_index = 0;
			std::atomic_int num_jobs_exited = 0;
			bool all_done = false;

			std::unique_lock<std::mutex> lk(done_lock);
			uint32_t n_jobs = std::min<uint32_t>(n_threads, static_cast<uint32_t>(in_vector.size()));
			job_lock.lock();
			for (uint32_t i = 0; i < n_jobs; i++) {
				jobs.push_back([&, n_jobs]() {

					int my_index = -1;
					while ((my_index = std::atomic_fetch_add(&next_index, 1)) < in_vector.size()) {
						action(in_vector[my_index]);
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

			start_working = true;
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

		static void worker_thread_func(bool* terminated, std::vector<std::function<void()>>* jobs, std::atomic_bool* start_working, std::condition_variable* sleep_cv, std::mutex* sleep_lock, std::mutex* job_lock) {
			while (!(*terminated)) {

				std::unique_lock<std::mutex> lk(*sleep_lock);
				wait(&lk, sleep_cv, [&]() -> bool { return *terminated || (*start_working && jobs->size() > 0); });
				lk.unlock();

				if (*terminated) {
					return;
				}

				job_lock->lock();
				if (*start_working && jobs->size() > 0) {
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
		std::vector<TastSetManagement> task_sets;
		std::mutex job_lock;
		std::mutex sleep_lock;
		std::condition_variable sleep_cv;

		std::atomic_bool start_working = false;
		int total_job_count;
		std::mutex done_lock;
		std::condition_variable done_cv;
		std::atomic_int num_jobs_exited;
		bool all_done;
	};
}