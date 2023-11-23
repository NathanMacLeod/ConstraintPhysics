#pragma once
#include <vector>
#include <string>

namespace phyz {
	class Constraint;

	class HolonomicSystem {
	public:
		HolonomicSystem() : buffer(nullptr), diagonal_elem_buffer(nullptr) {}
		HolonomicSystem(std::vector<Constraint*> constraints);
		HolonomicSystem(HolonomicSystem&& h);
		~HolonomicSystem();
		void operator=(HolonomicSystem&& h);
		void computeInverse(double cfm);
		void computeAndApplyImpulses(bool use_psuedo_velocities);
		inline int getDegree() { return system_degree; }
		inline int getNumConstraints() { return constraints.size(); }

		void debugPrintBuffer(std::string message="BUFFER CONTENT");

	private:
		std::vector<Constraint*> constraints;
		int system_degree;

		double* buffer;
		int buffer_capacity;
		double* diagonal_elem_buffer;
		int diagonal_elem_buffer_capacity;

		std::vector<int> block_location_table;
		std::vector<int> vector_location_lookup;
		const static int BLOCK_EMPTY = -1;
		int getFirstNonEmptyBlockBelowDiagonal(int column);
		int getBlockBufferLocation(int block_row, int block_column);
		int getBlockDiagonalElemBufferLocation(int block_row, int block_column);
		int getVectorPos(int constraint_row);

		std::vector<double> debug_buffer;
		std::vector<double> debug_diagonal_elem_buffer;
		std::vector<double> debug_inverse;
	};

};