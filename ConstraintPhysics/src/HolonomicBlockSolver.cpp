#include "HolonomicBlockSolver.h"

namespace phyz {

	HolonomicSystem::HolonomicSystem(std::vector<Constraint*> constraints) 
		: constraints(constraints), system_degree(0), buffer_capacity(0)
	{
		//todo: compute the proper ordering to have a sparse system, and exploit that.

		//compute the required size of the buffer and fill in the block location table
		for (int col = 0; col < constraints.size(); col++) {

			vector_location_lookup.push_back(system_degree);
			system_degree += constraints[col]->getDegree();

			for (int row = col; row < constraints.size(); row++) {
				//in final version we would check if this block actually needs to exist

				block_location_table.push_back(buffer_capacity);
				buffer_capacity += constraints[row]->getDegree() * constraints[col]->getDegree();
			}
		}

		int MAX_CONSTRAINT_DEGREE = 6;
		diagonal_elem_buffer_capacity = MAX_CONSTRAINT_DEGREE * system_degree;

		buffer = (double*)calloc(buffer_capacity, sizeof(double));
		diagonal_elem_buffer = (double*)calloc(diagonal_elem_buffer_capacity, sizeof(double));

	}

	template<int n>
	static void writeTargetDelta(DegreedConstraint<n>* constraint, std::vector<double>* target, int write_index, bool use_psuedo_velocities);

	static void multMatWithVec(int block_width, int block_height, std::vector<double>* target, int write_index, std::vector<double>& source_vector, int source_index, double* matrix_source);
	static void multMatTransposedWithVec(int block_width, int block_height, std::vector<double>* target, int write_index, std::vector<double>& source_vector, int source_index, double* matrix_source);

	void HolonomicSystem::computeAndApplyImpulses(bool use_psuedo_velocities) {
		//solving system d = A^-1(t - c)
		//d: delta in impulses needed to satisfy all constraints
		//A^1: inverse of the impulse to value matrix of this holonomic system
		//t: target value of all constraints
		//c: current value of all constraints

		std::vector<double> delta(system_degree); //initialize as t - c, then transform into d via multiplication by A^-1
		
		for (int i = 0; i < constraints.size(); i++) {
			int pos = getVectorPos(i);
			switch (constraints[i]->getDegree()) {
			case 1: writeTargetDelta<1>((DegreedConstraint<1>*)constraints[i], &delta, pos, use_psuedo_velocities); break;
			case 2: writeTargetDelta<2>((DegreedConstraint<2>*)constraints[i], &delta, pos, use_psuedo_velocities); break;
			case 3: writeTargetDelta<3>((DegreedConstraint<3>*)constraints[i], &delta, pos, use_psuedo_velocities); break;
			case 4: writeTargetDelta<4>((DegreedConstraint<4>*)constraints[i], &delta, pos, use_psuedo_velocities); break;
			case 5: writeTargetDelta<5>((DegreedConstraint<5>*)constraints[i], &delta, pos, use_psuedo_velocities); break;
			case 6: writeTargetDelta<6>((DegreedConstraint<6>*)constraints[i], &delta, pos, use_psuedo_velocities); break;
			}
		}

		//A^-1 = (L^-t)(D^-1)(L^-1) 
		
		//Multiply by L inverse (inversion is as simple as making non diagonal blocks negative)
		std::vector<double> multByLEffect(system_degree);
		for (int col = 0; col < constraints.size(); col++) {
			multByLEffect = std::vector<double>(system_degree, 0); //set to zero

			for (int row = col + 1; row < constraints.size(); row++) {
				int block_width = constraints[col]->getDegree();
				int block_height = constraints[row]->getDegree();
				double* block_pos = buffer + getBlockBufferLocation(row, col);
				int source_index = getVectorPos(col);
				int write_index = getVectorPos(row);

				multMatWithVec(block_width, block_height, &multByLEffect, write_index, delta, source_index, block_pos);
			}

			for (int i = 0; i < system_degree; i++) {
				delta[i] -= multByLEffect[i];
			}
		}

		//Multiply by D inverse
		for (int col = 0; col < constraints.size(); col++) {
			int block_degree = constraints[col]->getDegree();
			double* block_pos = buffer + getBlockBufferLocation(col, col);
			int vec_index = getVectorPos(col);
			multMatWithVec(block_degree, block_degree, &delta, vec_index, delta, vec_index, block_pos);
		}
	}


	int HolonomicSystem::getVectorPos(int constraint_row) {
		assert(constraint_row > 0 && constraint_row < constraints.size());
		return vector_location_lookup[constraint_row];
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
		

		if (a_c2interaction != nullptr) matMult(target, c1->a_jacobian, *a_c2interaction);
		if (b_c2interaction != nullptr) matMult(target, c1->b_jacobian, *b_c2interaction);
	}

	static void computeBlockInitialValue(double* target, Constraint* c1, Constraint* c2);

	//note this method computes the dot of a row on the left matrix and a row on the right matrix,
	//NOT a row on the left and a col on the right like you do in normal matrix multiplication.
	//This is because the instances of matrix multiplication where we use this are:
	//Multiplication with a symmetric matrix on the right, so dot with row or col is equivalent
	//Multiplication where we store the right matrix transposed in memory, so we are basically transposing it back with this
	template<int n>
	double dotProd(double* left_mat_row_start, double* right_mat_row_start) {
		double out = 0;
		for (int i = 0; i < n; i++) {
			out += left_mat_row_start[i] * right_mat_row_start[i];
		}
		return out;
	}

	static void calculateLowerTriangleBlock(int block_width, int block_height, double* target, double* lower_triangle_source, double* inverse_diagonal_block);

	template<int edge_width>
	static void computeLowerDiagonalBlock(int block_width, int block_height, double* target, double* ldtb_source, double* ldtb_buffer_source);

	void HolonomicSystem::computeInverse(double cfm) {
		//set initial values
		for (int col = 0; col < constraints.size(); col++) {

			for (int row = col; row < constraints.size(); row++) {
				Constraint* c1 = constraints[row];
				Constraint* c2 = constraints[col];

				double* block_pos = buffer + getBlockBufferLocation(row, col);
				computeBlockInitialValue(block_pos, c1, c2);
			}
		}

		//compute LDL
		for (int col = 0; col < constraints.size(); col++) {

			//computing inverse of diagonal block
			double* diagonal_block = buffer + getBlockBufferLocation(col, col);
			switch (constraints[col]->getDegree()) {
			case 1: mthz::rowMajorOrderInverse<1>(diagonal_block, diagonal_block); break;
			case 2: mthz::rowMajorOrderInverse<2>(diagonal_block, diagonal_block); break;
			case 3: mthz::rowMajorOrderInverse<3>(diagonal_block, diagonal_block); break;
			case 4: mthz::rowMajorOrderInverse<4>(diagonal_block, diagonal_block); break;
			case 5: mthz::rowMajorOrderInverse<5>(diagonal_block, diagonal_block); break;
			case 6: mthz::rowMajorOrderInverse<6>(diagonal_block, diagonal_block); break;
			}

			if (col + 1 == constraints.size()) continue; //no more blocks below or to the right of this.

			//computing lower triagular element for this block
			for (int row = col + 1; row < constraints.size(); row++) { //iterate per block in the column
				double* diagonal_elem_target = diagonal_elem_buffer + getBlockDiagonalElemBufferLocation(row, col); //we still need the old value in the lower block, so we are writing to a temporary buffer
				int block_buffer_pos = getBlockBufferLocation(row, col);
				int block_height = constraints[row]->getDegree();
				int block_width = constraints[col]->getDegree();

				calculateLowerTriangleBlock(block_width, block_height, diagonal_elem_target, diagonal_elem_buffer + block_buffer_pos, diagonal_block);
			}

			//calculate subtraction in the lower diagonal block
			for (int ld_col = col + 1; ld_col < constraints.size(); ld_col++) {
				for (int ld_row = ld_col; ld_row < constraints.size(); ld_row++) {

					double* ldtb_source = buffer + getBlockBufferLocation(ld_row, col);
					double* ldtb_buffer_source = diagonal_elem_buffer + getBlockDiagonalElemBufferLocation(ld_col, col);
					double* target = buffer + getBlockBufferLocation(ld_row, ld_col);

					switch (constraints[col]->getDegree()) {
					case 1: computeLowerDiagonalBlock<1>(constraints[ld_col]->getDegree(), constraints[ld_row]->getDegree(), target, ldtb_source, ldtb_buffer_source); break;
					case 2: computeLowerDiagonalBlock<2>(constraints[ld_col]->getDegree(), constraints[ld_row]->getDegree(), target, ldtb_source, ldtb_buffer_source); break;
					case 3: computeLowerDiagonalBlock<3>(constraints[ld_col]->getDegree(), constraints[ld_row]->getDegree(), target, ldtb_source, ldtb_buffer_source); break;
					case 4: computeLowerDiagonalBlock<4>(constraints[ld_col]->getDegree(), constraints[ld_row]->getDegree(), target, ldtb_source, ldtb_buffer_source); break;
					case 5: computeLowerDiagonalBlock<5>(constraints[ld_col]->getDegree(), constraints[ld_row]->getDegree(), target, ldtb_source, ldtb_buffer_source); break;
					case 6: computeLowerDiagonalBlock<6>(constraints[ld_col]->getDegree(), constraints[ld_row]->getDegree(), target, ldtb_source, ldtb_buffer_source); break;
					}
				}
			}

			//copy diagonal elem buffer back
			double* target = buffer + getBlockBufferLocation(col+1, col);
			double blocks_total_size = getBlockBufferLocation(col + 1, col + 1) - getBlockBufferLocation(col + 1, col); //total size in memory of all blocks in the col beneath the diagonal.
			memcpy(target, diagonal_elem_buffer, blocks_total_size);
		}
	}

	int HolonomicSystem::getBlockBufferLocation(int block_row, int block_column) {
		int n = constraints.size();

		assert(block_row >= 0 && block_row <= n);
		assert(block_column <= block_row);

		int table_index = block_row - block_column + ((2 * n + 1) * block_column - block_column * block_column) / 2;

		assert(table_index >= 0 && table_index < block_location_table.size());

		return block_location_table[table_index];
	}

	int HolonomicSystem::getBlockDiagonalElemBufferLocation(int block_row, int block_column) {
		assert(block_row >= 0 && block_row <= constraints.size());
		assert(block_column <= block_row);

		return getBlockBufferLocation(block_row, block_column) - getBlockBufferLocation(block_column, block_column);
	}

	template<int n>
	static void writeTargetDelta(DegreedConstraint<n>* constraint, std::vector<double>* target, int write_index, bool use_psuedo_velocities) {
		assert(write_index >= 0 && write_index + n <= target->size());

		NVec<n> current_val = use_psuedo_velocities? constraint->getConstraintValue(constraint->a_psyedo_velocity_change, constraint->b_psuedo_velocity_change)
			                                       : constraint->getConstraintValue(constraint->a_velocity_change, constraint->b_velocity_change);
		NVec<n> target_val = use_psuedo_velocities ? constraint->psuedo_target_val : constraint->target_val;

		NVec<n> delta = target_val - current_val;
		for (int j = 0; j < n; j++) {
			target->at(write_index + j) = delta.v[j];
		}
	}

	template<int block_width, int block_height>
	static void multMatWithVecDegreed(std::vector<double>* target, int write_index, std::vector<double>& source_vector, int source_index, double* matrix_source) {
		double* vec_source = source_vector.data() + source_index;
		for (int i = 0; i < block_height; i++) {
			target->at(write_index + i) = dotProd<block_width>(matrix_source + i * block_width, vec_source + i);
		}
	}

	static void multMatWithVec(int block_width, int block_height, std::vector<double>* target, int write_index, std::vector<double>& source_vector, int source_index, double* matrix_source) {
		switch (block_width) {
		case 1:
			switch (block_height) {
			case 1: multMatWithVecDegreed<1, 1>(target, write_index, source_vector, source_index, matrix_source); break;
			case 2: multMatWithVecDegreed<1, 2>(target, write_index, source_vector, source_index, matrix_source); break;
			case 3: multMatWithVecDegreed<1, 3>(target, write_index, source_vector, source_index, matrix_source); break;
			case 4: multMatWithVecDegreed<1, 4>(target, write_index, source_vector, source_index, matrix_source); break;
			case 5: multMatWithVecDegreed<1, 5>(target, write_index, source_vector, source_index, matrix_source); break;
			case 6: multMatWithVecDegreed<1, 6>(target, write_index, source_vector, source_index, matrix_source); break;
			}
			break;
		case 2:
			switch (block_height) {
			case 1: multMatWithVecDegreed<2, 1>(target, write_index, source_vector, source_index, matrix_source); break;
			case 2: multMatWithVecDegreed<2, 2>(target, write_index, source_vector, source_index, matrix_source); break;
			case 3: multMatWithVecDegreed<2, 3>(target, write_index, source_vector, source_index, matrix_source); break;
			case 4: multMatWithVecDegreed<2, 4>(target, write_index, source_vector, source_index, matrix_source); break;
			case 5: multMatWithVecDegreed<2, 5>(target, write_index, source_vector, source_index, matrix_source); break;
			case 6: multMatWithVecDegreed<2, 6>(target, write_index, source_vector, source_index, matrix_source); break;
			}
			break;
		case 3:
			switch (block_height) {
			case 1: multMatWithVecDegreed<3, 1>(target, write_index, source_vector, source_index, matrix_source); break;
			case 2: multMatWithVecDegreed<3, 2>(target, write_index, source_vector, source_index, matrix_source); break;
			case 3: multMatWithVecDegreed<3, 3>(target, write_index, source_vector, source_index, matrix_source); break;
			case 4: multMatWithVecDegreed<3, 4>(target, write_index, source_vector, source_index, matrix_source); break;
			case 5: multMatWithVecDegreed<3, 5>(target, write_index, source_vector, source_index, matrix_source); break;
			case 6: multMatWithVecDegreed<3, 6>(target, write_index, source_vector, source_index, matrix_source); break;
			}
			break;
		case 4:
			switch (block_height) {
			case 1: multMatWithVecDegreed<4, 1>(target, write_index, source_vector, source_index, matrix_source); break;
			case 2: multMatWithVecDegreed<4, 2>(target, write_index, source_vector, source_index, matrix_source); break;
			case 3: multMatWithVecDegreed<4, 3>(target, write_index, source_vector, source_index, matrix_source); break;
			case 4: multMatWithVecDegreed<4, 4>(target, write_index, source_vector, source_index, matrix_source); break;
			case 5: multMatWithVecDegreed<4, 5>(target, write_index, source_vector, source_index, matrix_source); break;
			case 6: multMatWithVecDegreed<4, 6>(target, write_index, source_vector, source_index, matrix_source); break;
			}
			break;
		case 5:
			switch (block_height) {
			case 1: multMatWithVecDegreed<5, 1>(target, write_index, source_vector, source_index, matrix_source); break;
			case 2: multMatWithVecDegreed<5, 2>(target, write_index, source_vector, source_index, matrix_source); break;
			case 3: multMatWithVecDegreed<5, 3>(target, write_index, source_vector, source_index, matrix_source); break;
			case 4: multMatWithVecDegreed<5, 4>(target, write_index, source_vector, source_index, matrix_source); break;
			case 5: multMatWithVecDegreed<5, 5>(target, write_index, source_vector, source_index, matrix_source); break;
			case 6: multMatWithVecDegreed<5, 6>(target, write_index, source_vector, source_index, matrix_source); break;
			}
			break;
		case 6:
			switch (block_height) {
			case 1: multMatWithVecDegreed<6, 1>(target, write_index, source_vector, source_index, matrix_source); break;
			case 2: multMatWithVecDegreed<6, 2>(target, write_index, source_vector, source_index, matrix_source); break;
			case 3: multMatWithVecDegreed<6, 3>(target, write_index, source_vector, source_index, matrix_source); break;
			case 4: multMatWithVecDegreed<6, 4>(target, write_index, source_vector, source_index, matrix_source); break;
			case 5: multMatWithVecDegreed<6, 5>(target, write_index, source_vector, source_index, matrix_source); break;
			case 6: multMatWithVecDegreed<6, 6>(target, write_index, source_vector, source_index, matrix_source); break;
			}
			break;
		}
	}

	static void computeBlockInitialValue(double* block_pos, Constraint* c1, Constraint* c2) {
		if (c1 == c2) {
			double* source;
			switch (c1->getDegree()) {
			case 1: source = (double*)((DegreedConstraint<1>*)c1)->impulse_to_value.v; break;
			case 2: source = (double*)((DegreedConstraint<2>*)c1)->impulse_to_value.v; break;
			case 3: source = (double*)((DegreedConstraint<3>*)c1)->impulse_to_value.v; break;
			case 4: source = (double*)((DegreedConstraint<4>*)c1)->impulse_to_value.v; break;
			case 5: source = (double*)((DegreedConstraint<5>*)c1)->impulse_to_value.v; break;
			case 6: source = (double*)((DegreedConstraint<6>*)c1)->impulse_to_value.v; break;
			}
			memcpy(block_pos, source, c1->getDegree() * c1->getDegree() * sizeof(double));
		}

		//lol
		switch (c1->getDegree()) {
		case 1:
		{
			DegreedConstraint<1>* d1 = (DegreedConstraint<1>*)c1;
			switch (c2->getDegree()) {
			case 1: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<1>*)c2); break;
			case 2: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<2>*)c2); break;
			case 3: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<3>*)c2); break;
			case 4: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<4>*)c2); break;
			case 5: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<5>*)c2); break;
			case 6: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<6>*)c2); break;
			}
			break;
		}
		case 2:
		{
			DegreedConstraint<2>* d1 = (DegreedConstraint<2>*)c1;
			switch (c2->getDegree()) {
			case 1: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<1>*)c2); break;
			case 2: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<2>*)c2); break;
			case 3: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<3>*)c2); break;
			case 4: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<4>*)c2); break;
			case 5: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<5>*)c2); break;
			case 6: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<6>*)c2); break;
			}
			break;
		}
		case 3:
		{
			DegreedConstraint<3>* d1 = (DegreedConstraint<3>*)c1;
			switch (c2->getDegree()) {
			case 1: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<1>*)c2); break;
			case 2: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<2>*)c2); break;
			case 3: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<3>*)c2); break;
			case 4: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<4>*)c2); break;
			case 5: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<5>*)c2); break;
			case 6: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<6>*)c2); break;
			}
			break;
		}
		case 4:
		{
			DegreedConstraint<4>* d1 = (DegreedConstraint<4>*)c1;
			switch (c2->getDegree()) {
			case 1: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<1>*)c2); break;
			case 2: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<2>*)c2); break;
			case 3: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<3>*)c2); break;
			case 4: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<4>*)c2); break;
			case 5: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<5>*)c2); break;
			case 6: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<6>*)c2); break;
			}
			break;
		}
		case 5:
		{
			DegreedConstraint<5>* d1 = (DegreedConstraint<5>*)c1;
			switch (c2->getDegree()) {
			case 1: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<1>*)c2); break;
			case 2: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<2>*)c2); break;
			case 3: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<3>*)c2); break;
			case 4: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<4>*)c2); break;
			case 5: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<5>*)c2); break;
			case 6: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<6>*)c2); break;
			}
			break;
		}
		case 6:
		{
			DegreedConstraint<6>* d1 = (DegreedConstraint<6>*)c1;
			switch (c2->getDegree()) {
			case 1: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<1>*)c2); break;
			case 2: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<2>*)c2); break;
			case 3: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<3>*)c2); break;
			case 4: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<4>*)c2); break;
			case 5: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<5>*)c2); break;
			case 6: computeBlockInitialValueDegreed(block_pos, d1, (DegreedConstraint<6>*)c2); break;
			}
			break;
		}
		}
	}

	template<int block_width, int block_height>
	static void calculateLowerTriangleBlockDegreed(double* target, double* lower_triangle_source, double* inverse_diagonal_block) {
		for (int row = 0; row < block_height; row++) { 
			for (int col = 0; col < block_width; col++) { 

				double dot_prod = dotProd<block_width>(lower_triangle_source + block_width * block_sub_row, inverse_diagonal_block + block_width * block_sub_col);
				target[block_sub_col + block_width * block_sub_row] = dot_prod;
			}
		}
	}

	static void calculateLowerTriangleBlock(int block_width, int block_height, double* target, double* lower_triangle_source, double* inverse_diagonal_block) {
		switch (block_width) {
		case 1:
			switch (block_height) {
			case 1: calculateLowerTriangleBlockDegreed<1, 1>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 2: calculateLowerTriangleBlockDegreed<1, 2>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 3: calculateLowerTriangleBlockDegreed<1, 3>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 4: calculateLowerTriangleBlockDegreed<1, 4>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 5: calculateLowerTriangleBlockDegreed<1, 5>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 6: calculateLowerTriangleBlockDegreed<1, 6>(target, lower_triangle_source, inverse_diagonal_block); break;
			}
			break;
		case 2:
			switch (block_height) {
			case 1: calculateLowerTriangleBlockDegreed<2, 1>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 2: calculateLowerTriangleBlockDegreed<2, 2>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 3: calculateLowerTriangleBlockDegreed<2, 3>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 4: calculateLowerTriangleBlockDegreed<2, 4>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 5: calculateLowerTriangleBlockDegreed<2, 5>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 6: calculateLowerTriangleBlockDegreed<2, 6>(target, lower_triangle_source, inverse_diagonal_block); break;
			}
			break;
		case 3:
			switch (block_height) {
			case 1: calculateLowerTriangleBlockDegreed<3, 1>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 2: calculateLowerTriangleBlockDegreed<3, 2>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 3: calculateLowerTriangleBlockDegreed<3, 3>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 4: calculateLowerTriangleBlockDegreed<3, 4>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 5: calculateLowerTriangleBlockDegreed<3, 5>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 6: calculateLowerTriangleBlockDegreed<3, 6>(target, lower_triangle_source, inverse_diagonal_block); break;
			}
			break;
		case 4:
			switch (block_height) {
			case 1: calculateLowerTriangleBlockDegreed<4, 1>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 2: calculateLowerTriangleBlockDegreed<4, 2>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 3: calculateLowerTriangleBlockDegreed<4, 3>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 4: calculateLowerTriangleBlockDegreed<4, 4>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 5: calculateLowerTriangleBlockDegreed<4, 5>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 6: calculateLowerTriangleBlockDegreed<4, 6>(target, lower_triangle_source, inverse_diagonal_block); break;
			}
			break;
		case 5:
			switch (block_height) {
			case 1: calculateLowerTriangleBlockDegreed<5, 1>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 2: calculateLowerTriangleBlockDegreed<5, 2>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 3: calculateLowerTriangleBlockDegreed<5, 3>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 4: calculateLowerTriangleBlockDegreed<5, 4>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 5: calculateLowerTriangleBlockDegreed<5, 5>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 6: calculateLowerTriangleBlockDegreed<5, 6>(target, lower_triangle_source, inverse_diagonal_block); break;
			}
			break;
		case 6:
			switch (block_height) {
			case 1: calculateLowerTriangleBlockDegreed<6, 1>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 2: calculateLowerTriangleBlockDegreed<6, 2>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 3: calculateLowerTriangleBlockDegreed<6, 3>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 4: calculateLowerTriangleBlockDegreed<6, 4>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 5: calculateLowerTriangleBlockDegreed<6, 5>(target, lower_triangle_source, inverse_diagonal_block); break;
			case 6: calculateLowerTriangleBlockDegreed<6, 6>(target, lower_triangle_source, inverse_diagonal_block); break;
			}
			break;
		}
	}

	template<int block_width, int block_height, int edge_width>
	static void computeLowerDiagonalBlockDegreed(double* target, double* ldtb_source, double* ldtb_buffer_source) {
		for (int row = 0; row < block_height; row++) {
			for (int col = 0; col < block_width, col++) {
				target[row][col] -= dotProd<edge_width>(ldtb_source + row * edge_width, ldtb_buffer_source + col * edge_width);
			}
		}
	}

	template<int edge_width>
	static void computeLowerDiagonalBlock(int block_width, int block_height, double* target, double* ldtb_source, double* ldtb_buffer_source) {
		switch (block_width) {
		case 1:
			switch (block_height) {
			case 1: computeLowerDiagonalBlockDegreed<1, 1, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 2: computeLowerDiagonalBlockDegreed<1, 2, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 3: computeLowerDiagonalBlockDegreed<1, 3, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 4: computeLowerDiagonalBlockDegreed<1, 4, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 5: computeLowerDiagonalBlockDegreed<1, 5, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 6: computeLowerDiagonalBlockDegreed<1, 6, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			}
			break;
		case 2:
			switch (block_height) {
			case 1: computeLowerDiagonalBlockDegreed<2, 1, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 2: computeLowerDiagonalBlockDegreed<2, 2, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 3: computeLowerDiagonalBlockDegreed<2, 3, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 4: computeLowerDiagonalBlockDegreed<2, 4, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 5: computeLowerDiagonalBlockDegreed<2, 5, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 6: computeLowerDiagonalBlockDegreed<2, 6, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			}
			break;
		case 3:
			switch (block_height) {
			case 1: computeLowerDiagonalBlockDegreed<3, 1, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 2: computeLowerDiagonalBlockDegreed<3, 2, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 3: computeLowerDiagonalBlockDegreed<3, 3, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 4: computeLowerDiagonalBlockDegreed<3, 4, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 5: computeLowerDiagonalBlockDegreed<3, 5, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 6: computeLowerDiagonalBlockDegreed<3, 6, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			}
			break;
		case 4:
			switch (block_height) {
			case 1: computeLowerDiagonalBlockDegreed<4, 1, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 2: computeLowerDiagonalBlockDegreed<4, 2, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 3: computeLowerDiagonalBlockDegreed<4, 3, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 4: computeLowerDiagonalBlockDegreed<4, 4, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 5: computeLowerDiagonalBlockDegreed<4, 5, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 6: computeLowerDiagonalBlockDegreed<4, 6, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			}
			break;
		case 5:
			switch (block_height) {
			case 1: computeLowerDiagonalBlockDegreed<5, 1, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 2: computeLowerDiagonalBlockDegreed<5, 2, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 3: computeLowerDiagonalBlockDegreed<5, 3, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 4: computeLowerDiagonalBlockDegreed<5, 4, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 5: computeLowerDiagonalBlockDegreed<5, 5, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 6: computeLowerDiagonalBlockDegreed<5, 6, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			}
			break;
		case 6:
			switch (block_height) {
			case 1: computeLowerDiagonalBlockDegreed<6, 1, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 2: computeLowerDiagonalBlockDegreed<6, 2, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 3: computeLowerDiagonalBlockDegreed<6, 3, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 4: computeLowerDiagonalBlockDegreed<6, 4, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 5: computeLowerDiagonalBlockDegreed<6, 5, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			case 6: computeLowerDiagonalBlockDegreed<6, 6, edge_width>(target, ldtb_source, ldtb_buffer_source); break;
			}
			break;
	}
};