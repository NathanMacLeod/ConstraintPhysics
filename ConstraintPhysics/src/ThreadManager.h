#pragma once
#include <vector>
#include <functional>

namespace phyz {

	
	namespace ThreadManager {

		template <typename T>
		void do_all(int n_threads, std::vector<T> in_vector, void* params, std::function<void(T current, void* params)> action);

	}

}