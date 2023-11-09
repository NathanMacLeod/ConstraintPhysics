#pragma once
#include "ConstraintSolver.h"

namespace phyz {

	class HolonomicSystem {
	public:

		class HolonomicSystem(std::vector<Constraint*> constraints);
		void computeInverse(double cfm);
		void computeAndApplyImpulses(bool use_psuedo_velocities);

	private:
		std::vector<Constraint*> constraints;
		int system_degree;

		double* buffer;
		int buffer_capacity;
		double* diagonal_elem_buffer;
		int diagonal_elem_buffer_capacity;

		std::vector<int> block_location_table;
		std::vector<int> vector_location_lookup;
		const int BLOCK_EMPTY = -1;
		int getBlockBufferLocation(int block_row, int block_column);
		int getBlockDiagonalElemBufferLocation(int block_row, int block_column);
		int getVectorPos(int constraint_row);

	};

};