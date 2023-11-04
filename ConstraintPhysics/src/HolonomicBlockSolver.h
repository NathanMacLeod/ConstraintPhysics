#pragma once
#include "ConstraintSolver.h"

namespace phyz {

	class HolonomicSystem {
	public:

		class HolonomicSystem(std::vector<Constraint*> constraints);
		void computeInverse(double cfm);


	private:
		std::vector<Constraint*> constraints;

		double* buffer;
		double buffer_capacity;
		double* diagonal_elem_buffer;
		double diagonal_elem_buffer_capacity;

		std::vector<int> block_location_table;
		const int BLOCK_EMPTY = -1;
		int getBlockBufferLocation(int block_row, int block_column);
	};

};