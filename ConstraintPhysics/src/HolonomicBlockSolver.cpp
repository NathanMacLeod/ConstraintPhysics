#include "HolonomicBlockSolver.h"

namespace phyz {

	HolonomicSystem::HolonomicSystem(std::vector<Constraint*> constraints) 
		: constraints(constraints), buffer_capacity(0)
	{
		//todo: compute the proper ordering to have a sparse system, and exploit that.

		//compute the required size of the buffer and fill in the block location table
		int matrix_degree = 0;

		for (int col = 0; col < constraints.size(); col++) {

			matrix_degree += constraints[col]->getDegree();

			for (int row = col; row < constraints.size(); row++) {
				//in final version we would check if this block actually needs to exist

				block_location_table.push_back(buffer_capacity);
				buffer_capacity += constraints[row]->getDegree() * constraints[col]->getDegree();
			}
		}

		int MAX_CONSTRAINT_DEGREE = 6;
		diagonal_elem_buffer_capacity = MAX_CONSTRAINT_DEGREE * matrix_degree;

		buffer = (double*)calloc(buffer_capacity, sizeof(double));
		diagonal_elem_buffer = (double*)calloc(diagonal_elem_buffer_capacity, sizeof(double));

	}

	template<int c1_degree, int c2_degree>
	static void matMult(double* target, const mthz::NMat<c1_degree, 6>& c1_mat, const mthz::NMat<6, c2_degree>& c2_mat) {
		for (int i = 0; i < c1_degree * c2_degree; i++) {
			int row = i / c2_degree;
			int col = i % c2_degree;

			for (int j = 0; j < 6; j++) {
				target[i] += c1_mat.v[row][j] * c2_mat.v[j][col];
			}
		}
	}

	template<int c1_degree, int c2_degree>
	static void computeBlockInitialValueDegreed(double* target, DegreedConstraint<c1_degree>* c1, DegreedConstraint<c2_degree>* c2) {
		mthz::NMat<6, c2_degree>* a_c2interaction = nullptr;
		mthz::NMat<6, c2_degree>* b_c2interaction = nullptr;
		if      (c1->a == c2->a) a_c2interaction = &c2->impulse_to_a_velocity;
		else if (c1->a == c2->b) a_c2interaction = &c2->impulse_to_b_velocity;
		if      (c1->b == c2->a) b_c2interaction = &c2->impulse_to_a_velocity;
		else if (c1->b == c2->b) b_c2interaction = &c2->impulse_to_b_velocity;
		

		if (a_c2interaction != nullptr) matMult(target, *c1->a_jacobian, *a_c2interaction);
		if (b_c2interaction != nullptr) matMult(target, *c1->b_jacobian, *b_c2interaction);
	}

	static void computeBlockInitialValue(double* target, Constraint* c1, Constraint* c2);

	void HolonomicSystem::computeInverse(double cfm) {
		for (int col = 0; col < constraints.size(); col++) {

			for (int row = col; row < constraints.size(); row++) {
				Constraint* c1 = constraints[row];
				Constraint* c2 = constraints[col];

				double* block_pos = buffer + getBlockBufferLocation(row, col);
				computeBlockInitialValue(block_pos, c1, c2);
			}
		}
	}

	int HolonomicSystem::getBlockBufferLocation(int block_row, int block_column) {
		int n = constraints.size();

		assert(block_row >= 0 && block_row <= n);
		assert(block_column < block_row);

		int table_index = block_row + ((2 * n + 1) * block_column - block_column * block_column) / 2;

		assert(table_index >= 0 && table_index < buffer_capacity);

		return block_location_table[table_index];
	}

	static void computeBlockInitialValue(double* block_pos, Constraint* c1, Constraint* c2) {
		//lol
		switch (c1->getDegree()) {
		case 1:
		{
			DegreedConstraint<1>* d1 = (DegreedConstraint<1>*)c1;
			switch (c2->getDegree()) {
			case 1: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<1>*)c2); break;
			case 2: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<2>*)c2); break;
			case 3: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<3>*)c2); break;
			case 4: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<4>*)c2); break;
			case 5: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<5>*)c2); break;
			case 6: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<6>*)c2); break;
			}
			break;
		}
		case 2:
		{
			DegreedConstraint<2>* d1 = (DegreedConstraint<2>*)c1;
			switch (c2->getDegree()) {
			case 1: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<1>*)c2); break;
			case 2: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<2>*)c2); break;
			case 3: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<3>*)c2); break;
			case 4: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<4>*)c2); break;
			case 5: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<5>*)c2); break;
			case 6: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<6>*)c2); break;
			}
			break;
		}
		case 3:
		{
			DegreedConstraint<3>* d1 = (DegreedConstraint<3>*)c1;
			switch (c2->getDegree()) {
			case 1: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<1>*)c2); break;
			case 2: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<2>*)c2); break;
			case 3: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<3>*)c2); break;
			case 4: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<4>*)c2); break;
			case 5: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<5>*)c2); break;
			case 6: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<6>*)c2); break;
			}
			break;
		}
		case 4:
		{
			DegreedConstraint<4>* d1 = (DegreedConstraint<4>*)c1;
			switch (c2->getDegree()) {
			case 1: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<1>*)c2); break;
			case 2: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<2>*)c2); break;
			case 3: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<3>*)c2); break;
			case 4: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<4>*)c2); break;
			case 5: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<5>*)c2); break;
			case 6: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<6>*)c2); break;
			}
			break;
		}
		case 5:
		{
			DegreedConstraint<5>* d1 = (DegreedConstraint<5>*)c1;
			switch (c2->getDegree()) {
			case 1: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<1>*)c2); break;
			case 2: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<2>*)c2); break;
			case 3: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<3>*)c2); break;
			case 4: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<4>*)c2); break;
			case 5: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<5>*)c2); break;
			case 6: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<6>*)c2); break;
			}
			break;
		}
		case 6:
		{
			DegreedConstraint<6>* d1 = (DegreedConstraint<6>*)c1;
			switch (c2->getDegree()) {
			case 1: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<1>*)c2); break;
			case 2: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<2>*)c2); break;
			case 3: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<3>*)c2); break;
			case 4: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<4>*)c2); break;
			case 5: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<5>*)c2); break;
			case 6: computeBlockInitialValue(block_pos, d1, (DegreedConstraint<6>*)c2); break;
			}
			break;
		}
		}
	}
};