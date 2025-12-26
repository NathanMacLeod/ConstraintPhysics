#pragma once
#include <vector>
#include <string>
#include <cinttypes>

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
		inline uint32_t getDegree() { return system_degree; }
		inline uint32_t getNumConstraints() { return static_cast<uint32_t>(constraints.size()); }

		void debugPrintBuffer(std::string message="BUFFER CONTENT", bool print_val=true);


		double* buffer;
	private:
		std::vector<Constraint*> constraints;
		int system_degree;
		
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

#ifndef NDEBUG
		bool USE_GAUSS_ELIM_FOR_INVERSE = false;
		std::vector<double> debug_buffer;
		std::vector<double> debug_diagonal_elem_buffer;
		std::vector<double> debug_inverse;
#endif
	};

};